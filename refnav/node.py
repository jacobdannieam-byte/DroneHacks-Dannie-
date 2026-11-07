#!/usr/bin/env python3
"""refnav.node - the ROS 2 shell around refnav.core. No flight decisions here.

What it takes from its surroundings, all through standard ROS interfaces (so the simulator, a
replay of recorded data and the aircraft differ only in their launch files):

  clock        the node clock: use_sim_time:=true follows /clock (the simulator publishes it;
               `ros2 bag play --clock` publishes the recording's), false = system time (aircraft)
  depth        sensor_msgs/Image (32FC1 m or 16UC1 mm) + CameraInfo, topics by parameter
  calibration  TF base_link <- the depth image's frame_id (a static transform; REP-103 optical
               frame), looked up at run time - never read from a simulator file
  health       std_msgs/Bool, true = the estimator feeding PX4 is healthy (topic by parameter)
  PX4          /fmu/out/{vehicle_local_position_v1, vehicle_attitude, vehicle_status_v1}
               in; /fmu/in/{offboard_control_mode, trajectory_setpoint, vehicle_command} out
  status       /refnav/status (status_topic; std_msgs/String, JSON): phase, why, leg, counters, the
               latest depth refusal - what a grader, the interactive runtime or an operator reads;
               the node itself never reads simulator truth
  control      optional (control_topic): std_msgs/String JSON {"run_id", "request": "land"} from the
               interactive runtime; a request for this run_id goes through the mission's own landing

Run:  python3 -m refnav.node --ros-args -p route_file:=scenarios/<x>.json -p depth_topic:=... \
          -p use_sim_time:=true
"""
from __future__ import annotations

import json
import math
import os
import sys
from time import perf_counter_ns

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from . import core as C
from . import depth as D

# PX4's uXRCE-DDS endpoints are BEST_EFFORT + TRANSIENT_LOCAL; a different durability never matches
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
CMD_SET_MODE, CMD_ARM, CMD_LAND = 176, 400, 21
PX4_MAIN_MODE_OFFBOARD = 6


def load_route(node):
    path = node.declare_parameter("route_file", "").value
    flat = list(node.declare_parameter("route", [0.0]).value)
    if path:
        with open(path) as f:
            doc = json.load(f)
        pts = doc["route"] if isinstance(doc, dict) else doc
        return [(float(a), float(b)) for a, b in pts], doc if isinstance(doc, dict) else {}
    if len(flat) >= 2 and len(flat) % 2 == 0:
        return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)], {}
    raise SystemExit("refnav: give route_file:=<json> or route:=[f1,l1,f2,l2,...] (forward/left metres)")


class RefNav(Node):
    def __init__(self, *, hybrid_runtime=None, hybrid_approval=None):
        super().__init__("refnav")
        self._hybrid_runtime, self._hybrid_approval = hybrid_runtime, hybrid_approval
        if hybrid_runtime is not None:
            from examples.gazebo_hybrid_adapter import IsolationApproval
            if (hybrid_runtime.strategy != "hybrid_sitl" or hybrid_runtime.adapter is None
                    or not isinstance(hybrid_approval, IsolationApproval)):
                raise ValueError("native owner needs a reviewed Hybrid SITL composition")
        route, doc = load_route(self) if hybrid_runtime is None else ([], {})
        prm = C.Params(route=route)
        for k, v in vars(C.Params()).items():
            if k == "route":
                continue
            default = 3. if hybrid_runtime is not None and k == "hover_s" else doc.get("params", {}).get(k, v)
            val = self.declare_parameter(k, default).value
            setattr(prm, k, type(v)(val))
        self.mission = C.Mission(prm, hybrid_navigation=hybrid_runtime is not None)
        adapter = hybrid_runtime.adapter if hybrid_runtime is not None else None
        depth_default = adapter.native("/camera/depth/image_raw") if adapter else "/camera/depth/image_raw"
        self.depth_topic = self.declare_parameter("depth_topic", depth_default).value
        info = self.declare_parameter("depth_info_topic", "").value
        info_default = adapter.native("/camera/depth/camera_info") if adapter else self.depth_topic.rsplit("/", 1)[0] + "/camera_info"
        self.info_topic = info or info_default
        health_default = adapter.native("/estimator/healthy") if adapter else "/estimator/healthy"
        self.health_topic = self.declare_parameter("health_topic", health_default).value
        base_default = hybrid_runtime.loop.replay.profile.reference.body_frame if adapter else "base_link"
        self.base_frame = self.declare_parameter("base_frame", base_default).value
        if adapter and ((self.depth_topic, self.info_topic, self.health_topic, self.base_frame) !=
                        (depth_default, info_default, health_default, base_default)
                        or not self.get_parameter("use_sim_time").value or not .5 <= prm.hover_s <= 20.
                        or not hybrid_approval.permits(adapter, os.environ.get("ROS_DOMAIN_ID"))):
            raise ValueError("reviewed owner sensor bindings, sim clock and isolation required")
        self.exit_when_done = bool(self.declare_parameter("exit_when_done", True).value)
        control = self.declare_parameter("control_topic", "").value
        self.run_id = self.declare_parameter("run_id", "").value
        # PX4-bound stamps use the SYSTEM clock in every setting: on the aircraft PX4's DDS client
        # converts agent (system) time with its timesync; the simulator's PX4 boots with
        # UXRCE_DDS_SYNCT=0 and clamps a future stamp to its own "now". Simulation time would
        # be read as seconds-old commands by a real PX4, so it is never used for these.
        self.sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        from px4_msgs.msg import (EstimatorStatusFlags, OffboardControlMode, TrajectorySetpoint, VehicleAttitude,
                                  VehicleCommand, VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
        self._OCM, self._TSP, self._CMD = OffboardControlMode, TrajectorySetpoint, VehicleCommand
        self.pub_ocm = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)
        status_topic = self.declare_parameter("status_topic", "/refnav/status").value
        self.pub_status = self.create_publisher(String, status_topic, 10)

        self.v = C.Vehicle()
        owner_topics = adapter.owner_topics if adapter else {}
        state_topics = hybrid_runtime.loop.replay.hybrid.px4_topics if adapter else {}
        for message, topic, default, callback in (
                (VehicleLocalPosition, "vehicle_local_position", "vehicle_local_position_v1", self._on_lpos),
                (VehicleAttitude, "vehicle_attitude", "vehicle_attitude", self._on_att),
                (VehicleStatus, "vehicle_status", "vehicle_status_v1", self._on_status),
                (VehicleLandDetected, "vehicle_land_detected", "vehicle_land_detected", self._on_land),
                (EstimatorStatusFlags, "estimator_status_flags", "estimator_status_flags", self._on_est_flags)):
            name = owner_topics.get(topic, state_topics.get(topic, "/fmu/out/" + default))
            self.create_subscription(message, name, callback, PX4_QOS)
        self._yaw_align = (False, None)
        self.create_subscription(Bool, self.health_topic, self._on_health, 10)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self._on_depth, qos_profile_sensor_data)
        if control:
            # transient-local: a request made a moment before this node subscribed still arrives
            ctl_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
            self.create_subscription(String, control, self._on_control, ctl_qos)

        self.K = None
        self.extrinsic = None           # (frame_id, R_body_cam, t_body_cam)
        self.rejects = {}
        self.done_at = None
        self._wait_log_t = None
        self._hybrid_generation = 0
        self._hybrid_reports = {}
        self._hybrid_clock_ns = None
        if hybrid_runtime is not None:
            from examples.hybrid_launch import attach_observations
            attach_observations(self, hybrid_runtime)
        self.create_timer(0.05, self._tick)            # node clock: pauses with a paused simulator
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(f"route {len(route)} legs; depth {self.depth_topic}; health {self.health_topic}; "
                               f"sim time {self.get_parameter('use_sim_time').value}")

    # ------------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _invalidate_hybrid(self):
        if self._hybrid_runtime is not None:
            self._hybrid_runtime.invalidate()
            self._hybrid_generation += 1

    def _note_hybrid_report(self, key, identity, source_stamp_us):
        now = self.get_clock().now().nanoseconds
        if self._hybrid_runtime is not None:
            clock = self._hybrid_runtime.loop.replay.hybrid.clock
            try:
                capture = clock.capture(source_stamp_us, now)
            except ValueError:
                self._hybrid_reports.pop(key, None)
                self._invalidate_hybrid()
                return
            if now - capture + clock.max_error_ns > 300_000_000:
                self._hybrid_reports.pop(key, None)
                self._invalidate_hybrid()
                return
            now = capture
        previous = self._hybrid_reports.get(key)
        if previous is not None and (now < previous[0] or identity != previous[1]):
            self._invalidate_hybrid()
        self._hybrid_reports[key] = (now, identity)

    def _on_est_flags(self, m):
        self._yaw_align = (bool(m.cs_yaw_align), self._now())

    def _on_lpos(self, m):
        self.v.pos = (m.x, m.y, m.z)
        self.v.heading = m.heading
        # heading: PX4's heading_good_for_control, OR its own yaw alignment. With a COMPASS, PX4 sets
        # heading_good_for_control only after re-aligning the heading in flight above 1.5 m (ekf.h
        # isYawFinalAlignComplete: ground magnetic anomalies), so a compass aircraft would wait on the
        # ground forever (field_two_tags, 2026-10-17); yaw alignment is what PX4's own position
        # controller flies on. A vision heading sets heading_good_for_control on the ground.
        ya, ya_t = self._yaw_align
        heading_ok = bool(m.heading_good_for_control) or (ya and ya_t is not None and self._now() - ya_t <= 3.0)
        self.v.pos_ok = bool(m.xy_valid and m.z_valid and heading_ok
                             and all(math.isfinite(c) for c in (m.x, m.y, m.z, m.heading)))
        if self._hybrid_runtime is not None:
            from onboard.diagnostics.phase1_probe import RESET_FIELDS
            self._note_hybrid_report("position", tuple(getattr(m, field, None) for field in RESET_FIELDS),
                                     getattr(m, "timestamp_sample", None))

    def _on_att(self, m):
        self.v.q = tuple(float(c) for c in m.q)

    def _on_status(self, m):
        self.v.armed = m.arming_state == 2
        self.v.nav_state = int(m.nav_state)
        if self._hybrid_runtime is not None:
            self._note_hybrid_report("authority", (self.v.armed, self.v.nav_state), getattr(m, "timestamp", None))

    def _on_land(self, m):
        self.v.landed = bool(m.landed)
        if self._hybrid_runtime is not None:
            self._note_hybrid_report("landed", self.v.landed, getattr(m, "timestamp", None))

    def _on_health(self, m):
        self.mission.on_health(self._now(), m.data)
        if not m.data:
            self._invalidate_hybrid()

    def _on_info(self, m):
        if m.k[0] > 0 and m.k[4] > 0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_control(self, m):
        try:
            req = json.loads(m.data)
        except ValueError:
            return
        if req.get("request") == "land" and (not self.run_id or req.get("run_id") == self.run_id):
            self.get_logger().info(f"landing requested by the runtime ({req.get('why', 'operator')})")
            self._invalidate_hybrid()
            self.mission.request_land(req.get("why") or "operator")

    def _reject(self, key, detail=None, record=True):
        """Count a refused depth frame by its reason; log the reason WITH its numbers at the 1st, 10th,
        100th and 1000th time; tell the mission (which reports it while it waits)."""
        detail = detail or key
        if record:
            self.mission.depth_refused(key, detail)
        self.rejects[key] = self.rejects.get(key, 0) + 1
        if self.rejects[key] in (1, 10, 100, 1000):
            self.get_logger().warning(f"depth frame not used ({self.rejects[key]}x): {detail}")

    def _calibration(self, frame_id):
        if self.extrinsic is not None and self.extrinsic[0] == frame_id:
            return self.extrinsic
        from rclpy.time import Time
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except Exception as e:                      # not (yet) on the TF tree
            self._reject("no_tf", f"no TF {self.base_frame} <- {frame_id} ({type(e).__name__})")
            return None
        r, t = tf.transform.rotation, tf.transform.translation
        R = D.quat_to_mat(r.w, r.x, r.y, r.z)
        fwd = R[:, 2]                               # the optical axis in the body frame
        ang = math.degrees(math.acos(max(-1.0, min(1.0, fwd[0] / np.linalg.norm(fwd)))))
        if ang > 25.0:
            self._reject("camera_not_forward",
                         f"camera {frame_id} looks {ang:.0f} deg off the body's forward axis: this package needs a forward camera")
            return None
        self.extrinsic = (frame_id, R, np.array([t.x, t.y, t.z]))
        self.get_logger().info(f"calibration from TF: {self.base_frame} <- {frame_id} t=({t.x:.3f},{t.y:.3f},{t.z:.3f}) "
                               f"optical axis {ang:.1f} deg off forward")
        return self.extrinsic

    def _on_depth(self, m):
        if self.K is None:
            return self._reject("no_camera_info", f"no camera_info yet on {self.info_topic}")
        cal = self._calibration(m.header.frame_id)
        if cal is None:
            return None
        if self.v.q is None:
            return self._reject("no_attitude", "no PX4 attitude yet")
        try:
            d = D.decode_depth(m.encoding, m.height, m.width, m.step, bytes(m.data))
        except ValueError as e:
            return self._reject("undecodable", str(e))
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not self.mission.on_depth(stamp, d, self.K, cal[1], cal[2], self.v.q, now=self._now()):
            key, detail = self.mission.depth_reject or ("refused", "refused by the mission")
            self._reject(key, detail, record=False)
        return None

    # ------------------------------------------------------------------ outputs
    def _sys_us(self):
        if self._hybrid_runtime is not None and self._hybrid_runtime.adapter.simulated_estimator:
            clock = self._hybrid_runtime.loop.replay.hybrid.clock
            now = self.get_clock().now().nanoseconds
            # This isolated Gazebo image disables XRCE time synchronization.
            # Convert back to the reviewed PX4 simulation clock only here.
            return clock.source_stamp_us(now)
        return self.sys_clock.now().nanoseconds // 1000

    def _command(self, cmd, p1=0.0, p2=0.0):
        c = self._CMD()
        c.timestamp = self._sys_us()
        c.command, c.param1, c.param2 = cmd, float(p1), float(p2)
        c.target_system = c.target_component = c.source_system = c.source_component = 1
        c.from_external = True
        self.pub_cmd.publish(c)

    def _runtime_owner(self, now_ns):
        from onboard.diagnostics.sitl_graph import inspect as inspect_command_graph

        runtime = self._hybrid_runtime
        try:
            admission = (self.get_parameter("use_sim_time").value is True
                         and self._hybrid_approval.permits(runtime.adapter, os.environ.get("ROS_DOMAIN_ID"))
                         and not inspect_command_graph(self, self.get_fully_qualified_name(),
                                                       strict_motion=runtime.adapter.simulated_estimator)["issues"])
            if admission and runtime.adapter.simulated_estimator:
                try:
                    runtime.loop.replay.hybrid.clock.source_stamp_us(now_ns)
                except ValueError:
                    admission = False
            authority = self._hybrid_reports.get("authority")
            if self.mission.t_cmd is not None or self.mission.phase in C.AIRBORNE + ("land",):
                admission = admission and authority is not None and 0 <= now_ns - authority[0] <= 300_000_000
            now = now_ns * 1e-9
            health = self.mission.health
            ready = (len(self._hybrid_reports) == 3 and
                     all(0 <= now_ns - stamp <= 300_000_000 for stamp, _ in self._hybrid_reports.values())
                     and health.last_ok is True and health.last_t is not None
                     and 0 <= now - health.last_t <= health.stale_s and self.mission.depth_fresh(now)
                     and self.mission.depth_t <= now
                     and self._hybrid_reports["position"][1] == runtime.loop.replay.hybrid.local_reset_counters)
            return runtime.owner_step(self.mission, self.v, now_ns=now_ns, wall_ns=perf_counter_ns(),
                                      generation=self._hybrid_generation, admission=admission, owner_ready=ready)
        except (ValueError, TypeError, KeyError, AttributeError, RuntimeError, ArithmeticError) as error:
            runtime.invalidate()
            runtime.course.abort("owner_adapter_failure:" + type(error).__name__)
            runtime.course.lifecycle("done", yielded=True)
            return self.mission.yield_hybrid_authority(now_ns * 1e-9, runtime.course.land_reason)

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        now = now_ns * 1e-9
        if self._hybrid_runtime is not None:
            if self._hybrid_clock_ns is not None and now_ns < self._hybrid_clock_ns:
                self._hybrid_reports.clear()
                self._invalidate_hybrid()
            self._hybrid_clock_ns = now_ns
        if now <= 0.0:
            return                                   # sim time before the first /clock
        out = (self.mission.step(now, self.v) if self._hybrid_runtime is None else self._runtime_owner(now_ns))
        for e in out.events:
            self.get_logger().info(e)
        if self.mission.phase == "wait" and self.mission.t_phase is not None:
            if self._wait_log_t is None or now - self._wait_log_t >= 10.0:   # say why it is not flying, every 10 s
                if self._wait_log_t is not None:
                    self.get_logger().info(f"not ready after {now - self.mission.t_phase:.0f} s "
                                           f"(gives up at {self.mission.p.wait_s:.0f} s): {self.mission.why}")
                self._wait_log_t = now
        if out.events:
            self._publish_status()
        if not out.stream or out.commands or out.phase != "go":
            self._invalidate_hybrid()
        if out.stream and out.setpoint is not None:
            setpoint = out.setpoint
            o = self._OCM()
            o.timestamp = self._sys_us()
            o.position = True
            self.pub_ocm.publish(o)
            s = self._TSP()
            s.timestamp = o.timestamp
            s.position = [float(c) for c in setpoint[:3]]
            s.velocity = [math.nan] * 3
            s.acceleration = [math.nan] * 3
            s.yaw = float(setpoint[3])
            self.pub_sp.publish(s)
            if self._hybrid_runtime is not None:
                self._hybrid_runtime.issue(now_ns, px4_issue_us=s.timestamp, setpoint=setpoint)
        for c in out.commands:
            if c == "offboard":
                self._command(CMD_SET_MODE, 1.0, PX4_MAIN_MODE_OFFBOARD)
            elif c == "arm":
                self._command(CMD_ARM, 1.0)
            elif c == "land":
                self._command(CMD_LAND)
            elif c == "disarm":
                self._command(CMD_ARM, 0.0)                 # param2 0: PX4 refuses it in the air
        if out.phase == "done" and self.done_at is None:
            self.done_at = now
            self.get_logger().info(f"mission done: {self.mission.result}")

    def _publish_status(self):
        st = self.mission.status(self._now())
        st["rejects"] = dict(self.rejects)
        if self._hybrid_runtime is not None:
            st["hybrid"] = self._hybrid_runtime.status()
        self.pub_status.publish(String(data=json.dumps(st)))
        if self.exit_when_done and self.done_at is not None and self._now() - self.done_at > 1.5:
            raise SystemExit(0 if self.mission.result == "complete" else 1)


def main(argv=None):
    """argv: a full command line (argv[0] then --ros-args ...), default sys.argv - so a program can run
    the node with its own settings (examples/takeoff_land, runtime/program.refnav_args)."""
    rclpy.init(args=list(argv) if argv is not None else sys.argv)
    node = RefNav()
    code = 1
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code or 0)
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
