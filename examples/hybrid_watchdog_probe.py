"""Private ROS fixture for RefNav's steady watchdog. Run without PX4 or Gazebo."""

import json
import os
from time import perf_counter
from uuid import uuid4


def main():
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not domain.isdecimal() or not 1 <= int(domain) <= 232:
        raise SystemExit("Use a dedicated local-only ROS domain in an isolated container")

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
    from rosgraph_msgs.msg import Clock

    from examples.gazebo_hybrid_adapter import IsolationApproval
    from examples.hybrid_mission_smoke import runtime_fixture
    from onboard.diagnostics.sitl_graph import inspect
    from refnav.core import Output
    from refnav.hybrid_execution import Held
    from refnav.node import PX4_QOS, RefNav
    from refnav.stopping_envelope import StoppingEnvelope

    class FixtureOwner(RefNav):
        def _runtime_owner(self, now_ns):
            # Private output fixture only. Motion admission, telemetry and PX4
            # behavior are outside this timer and publication-fence probe.
            held = self._hybrid_runtime.executor.held
            return Output("go", held.target, stream=True) if held is not None else Output("go")

    private = "/qualification/watchdog_" + uuid4().hex[:10]
    outputs = {"offboard_control_mode": OffboardControlMode,
               "trajectory_setpoint": TrajectorySetpoint, "vehicle_command": VehicleCommand}
    args = ["--ros-args", "-p", "use_sim_time:=true", "-p", "exit_when_done:=false",
            "-r", f"/clock:={private}/clock", "-r", f"/refnav/status:={private}/status"]
    for name in outputs:
        args.extend(("-r", f"/fmu/in/{name}:={private}/{name}"))

    runtime = runtime_fixture(monotonic=perf_counter)
    runtime.executor.stopping_envelope = StoppingEnvelope(.16, .1, .5, .3, .1, "a" * 64)
    approval = IsolationApproval("fixture", int(domain), "private_watchdog_probe",
                                 lambda profile, evidence: (profile, evidence) == ("fixture", "private_watchdog_probe"))
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    fixture = owner = None
    try:
        fixture = rclpy.create_node("watchdog_clock_fixture", use_global_arguments=False)
        owner = FixtureOwner(hybrid_runtime=runtime, hybrid_approval=approval)
        executor.add_node(fixture)
        executor.add_node(owner)
        clock = fixture.create_publisher(Clock, private + "/clock", 10)
        seen = {name: [] for name in outputs}
        for name, kind in outputs.items():
            fixture.create_subscription(kind, private + "/" + name, seen[name].append, PX4_QOS)

        def wait_for(predicate, description, timeout=3.):
            deadline = perf_counter() + timeout
            while not predicate():
                if perf_counter() >= deadline:
                    raise RuntimeError("Timed out waiting for " + description)
                executor.spin_once(timeout_sec=.01)

        def advance_to(ns):
            message = Clock()
            message.clock.sec, message.clock.nanosec = divmod(ns, 10**9)
            clock.publish(message)
            wait_for(lambda: owner.get_clock().now().nanoseconds == ns, "private clock delivery")

        def pump_for(seconds):
            deadline = perf_counter() + seconds
            while perf_counter() < deadline:
                executor.spin_once(timeout_sec=.01)

        wait_for(lambda: clock.get_subscription_count() >= 2
                 and owner.pub_sp.get_subscription_count() == 1
                 and owner.pub_ocm.get_subscription_count() == 1
                 and owner.pub_cmd.get_subscription_count() == 1, "private graph discovery")
        assert not inspect(fixture)["issues"], "ordinary command graph must remain idle"
        advance_to(100_000_000)
        pump_for(.08)  # Drain the preflight timer before seeding the target.
        owner.mission.phase = "go"
        runtime.executor.held = Held((.4, 0., -.8, 0.), 0, (), None, "fixture",
                                     400_000_000, perf_counter() + .20)
        advance_to(160_000_000)
        wait_for(lambda: seen["trajectory_setpoint"] and seen["offboard_control_mode"], "initial private target")
        before = {name: len(messages) for name, messages in seen.items()}
        assert before == {"offboard_control_mode": 1, "trajectory_setpoint": 1, "vehicle_command": 0}

        wait_for(lambda: runtime.owner_watchdog_fault is not None, "watchdog while ROS clock is paused")
        assert owner.get_clock().now().nanoseconds == 160_000_000
        assert runtime.executor.held is None and owner.mission.phase == "done"
        assert runtime.owner_watchdog_fault == "owner_ros_clock_stalled"
        expiry = next(record for record in runtime.records if record["event"] == "owner_target_expired")
        assert expiry["wall_expired"] and not expiry["ros_expired"]
        advance_to(260_000_000)
        advance_to(360_000_000)
        pump_for(.2)
        assert not inspect(fixture)["issues"]
        after = {name: len(messages) for name, messages in seen.items()}
        assert after == before
        assert sum(record["event"] == "owner_watchdog" for record in runtime.records) == 1
        print(json.dumps({"scope": "private ROS fixture without PX4 or Gazebo",
                          "paused_ros_ns": 160_000_000, "resumed_ros_ns": 360_000_000,
                          "before": before, "after": after, "records": list(runtime.records)}))
    finally:
        runtime.close()
        executor.shutdown()
        if owner is not None:
            owner.destroy_node()
        if fixture is not None:
            fixture.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
