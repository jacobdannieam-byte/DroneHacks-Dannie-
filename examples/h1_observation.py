"""Single-epoch ROS message adapter for read-only H0 replay."""

from collections import deque
from dataclasses import dataclass, replace
import math
import re
from time import perf_counter_ns

import numpy as np

from examples.spatial_bag_input import BagProfile, RecordedSpatialReplay, TOPICS, _ns
from onboard.adapters.geometry import quat_to_mat
from onboard.diagnostics.phase1_health import static_tf_tree
from onboard.diagnostics.phase1_probe import RESET_FIELDS, payloads
from refnav.dynamics import DroneState, LocalNED
from refnav.hybrid_integration import assess
from refnav.reasoning import DeterministicReasoner, WorldReference, build_context
from refnav.reasoning.schema import MissionGoal
from refnav.trajectory_evaluation import EvaluationPolicy, FrameAlignment, GoalBinding, MetricGoal, VehicleBounds
from refnav.world_model.spatial_input import NS, PoseSample, stamp_ok, vector

TF = "/tf_static"
TRACKING = "/visual_slam/processing_status"
PX4_TYPES = {
    "vehicle_odometry": "px4_msgs/msg/VehicleOdometry",
    "vehicle_local_position": "px4_msgs/msg/VehicleLocalPosition",
    "estimator_status_flags": "px4_msgs/msg/EstimatorStatusFlags",
}
HEALTH_MAX_AGE_NS = 300_000_000
TF_LIMIT = 64


@dataclass(frozen=True)
class ClockMapping:
    """One reviewed offset for the bridged PX4 stamps within a bounded ROS epoch."""

    source_clock_id: str
    ros_clock_id: str
    offset_ns: int
    max_error_ns: int
    valid_from_ns: int
    valid_until_ns: int
    evidence_id: str

    def __post_init__(self):
        if (not all(isinstance(v, str) and 0 < len(v.strip()) <= 128
                    for v in (self.source_clock_id, self.ros_clock_id, self.evidence_id))
                or type(self.offset_ns) is not int or not -(2**63) < self.offset_ns < 2**63
                or not stamp_ok(self.max_error_ns) or self.max_error_ns > 10_000_000
                or not stamp_ok(self.valid_from_ns) or not stamp_ok(self.valid_until_ns)
                or self.valid_until_ns <= self.valid_from_ns):
            raise ValueError("invalid_clock_mapping")

    def capture(self, stamp_us, received_ns):
        if not stamp_ok(stamp_us) or stamp_us == 0:
            raise ValueError("invalid_px4_sample_stamp")
        mapped = stamp_us * 1000 + self.offset_ns
        if not (self.valid_from_ns <= mapped - self.max_error_ns <= mapped + self.max_error_ns
                <= received_ns <= self.valid_until_ns):
            raise ValueError("px4_stamp_outside_reviewed_clock")
        return mapped


def _counter(value):
    return type(value) is int and 0 <= value <= 255


@dataclass(frozen=True)
class HybridBagProfile:
    spatial: BagProfile
    clock: ClockMapping
    alignment: FrameAlignment
    vehicle: VehicleBounds
    objective: MetricGoal
    px4_topics: dict
    ev_reset_counter: int
    local_reset_counters: tuple
    mission_goal: MissionGoal | None = None

    def __post_init__(self):
        ref = self.spatial.reference
        if (self.clock.ros_clock_id != ref.clock_id or self.alignment.map != ref
                or self.alignment.ned.clock_id != ref.clock_id
                or self.alignment.ned.reference_epoch != ref.epoch
                or self.objective.reference != ref or not _counter(self.alignment.estimator_reset_counter)
                or not _counter(self.ev_reset_counter) or len(self.local_reset_counters) != len(RESET_FIELDS)
                or not all(_counter(v) for v in self.local_reset_counters)):
            raise ValueError("inconsistent_reviewed_reference_or_reset_markers")
        if (set(self.px4_topics) != set(PX4_TYPES)
                or any(not isinstance(topic, str) or not re.fullmatch(rf"/fmu/out/{role}(?:_v[0-9]+)?", topic)
                       for role, topic in self.px4_topics.items())):
            raise ValueError("supply_discovered_px4_output_topics")

    @classmethod
    def from_dict(cls, data):
        spatial = BagProfile.from_dict(data["spatial"])
        ref = spatial.reference
        alignment = FrameAlignment(LocalNED(data["ned_frame"], ref.epoch, ref.clock_id), ref,
                                   data["alignment"]["time_origin_ns"], **data["alignment"])
        return cls(spatial, ClockMapping(**data["clock"]), alignment, VehicleBounds(**data["vehicle"]),
                   MetricGoal(ref, data["objective_m"]), data["px4_topics"], data["ev_reset_counter"],
                   tuple(data["local_reset_counters"]),
                   MissionGoal.from_dict(data["mission_goal"]) if data.get("mission_goal") is not None else None)


class HybridBagReplay(RecordedSpatialReplay):
    """Reuse the Phase 2 reader, retaining at most one PX4 state and pending depth."""

    def __init__(self, profile, *, planner_budget_s=.025, monotonic=None):
        super().__init__(profile.spatial)
        self.hybrid = profile
        self.planner_budget_s = planner_budget_s
        self.monotonic = monotonic
        self.topics = {**TOPICS, TF: "tf2_msgs/msg/TFMessage", TRACKING: "diagnostic_msgs/msg/DiagnosticArray",
                       **{topic: PX4_TYPES[role] for role, topic in profile.px4_topics.items()}}
        self._roles = {topic: role for role, topic in profile.px4_topics.items()}
        self._tf = {}
        self._extrinsics_ok = False
        self._flags = self._local = self._tracking = self._state = self._last_px4 = None
        self._last_vio_capture = None
        self._visual = deque(maxlen=64)
        self._state_reason = "missing_px4_state"
        self.fault = None

    def feed(self, topic, msg, received_ns):
        if self.fault is not None:
            raise ValueError(self.fault)
        try:
            if topic in TOPICS:
                if topic == "/fmu/in/vehicle_visual_odometry" and msg.reset_counter != self.hybrid.ev_reset_counter:
                    raise ValueError("ev_reset_requires_new_epoch")
                if topic == "/camera/depth/image_raw":
                    if not self._extrinsics_ok:
                        raise ValueError("missing_reviewed_depth_extrinsics")
                    if self._pending is not None:
                        raise ValueError("depth_queue_overload")
                super().feed(topic, msg, received_ns)
            else:
                self._advance(topic, received_ns)
                if topic == TF:
                    # Transient-local TF can be the first delivery in a bag.
                    self._feed_tf(msg)
                    return
                self._require_clock(received_ns)
                if topic == TRACKING:
                    self._feed_tracking(msg)
                else:
                    self._feed_px4(self._roles[topic], msg)
                self._flush()
        except ValueError as error:
            self.fault = str(error)
            self._state = self._pending = None
            raise

    def _feed_pose(self, msg, received_ns):
        capture = _ns(msg.header.stamp)
        if self._last_vio_capture is not None and capture < self._last_vio_capture:
            raise ValueError("vio_pose_capture_rewind")
        self._last_vio_capture = capture
        super()._feed_pose(msg, received_ns)

    def _feed_tf(self, msg):
        if len(msg.transforms) > TF_LIMIT:
            raise ValueError("static_tf_queue_overload")
        for row in payloads("tf_static", msg):
            key = (row["frame"], row["child_frame"], row["translation_m"], row["rotation_xyzw"])
            self._tf[key] = row
            if len(self._tf) > TF_LIMIT:
                raise ValueError("static_tf_queue_overload")
        cal = self.profile.calibration
        issues = set()
        _, reached = static_tf_tree(self._tf.values(), cal.body_frame, issues)
        if issues:
            raise ValueError("invalid_static_tf:" + ",".join(sorted(issues)))
        if cal.camera_frame in reached:
            translation, rotation = reached[cal.camera_frame]
            if (not np.allclose(translation, cal.t_body_optical_m, rtol=0, atol=1e-6)
                    or not np.allclose(quat_to_mat(*rotation), cal.R_body_optical, rtol=0, atol=1e-6)):
                raise ValueError("depth_extrinsics_mismatch")
            self._extrinsics_ok = True

    def _feed_tracking(self, msg):
        row, = payloads("processing", msg)
        stamp = row["stamp_ns"]
        good = (row["valid"] and row["result"] == "visual" and not row["covariance_rejected"]
                and 0 <= self.now - stamp <= HEALTH_MAX_AGE_NS)
        if self._tracking is not None and stamp < self._tracking[0]:
            raise ValueError("vio_tracking_capture_rewind")
        self._tracking = (stamp, good)
        if good:
            self._visual.append(stamp)
        else:
            self._visual.clear()
            self.world.push_pose(PoseSample(self.profile.reference, self.now, self.now, False))

    def _feed_px4(self, role, msg):
        if role == "estimator_status_flags":
            stamp = self.hybrid.clock.capture(msg.timestamp, self.now)
            report, = payloads("px4_flags", msg)
            if self._flags is not None and stamp < self._flags[0]:
                raise ValueError("px4_estimator_flags_capture_rewind")
            self._flags = (stamp, report["valid"])
            return
        capture = self.hybrid.clock.capture(msg.timestamp_sample, self.now)
        published = self.hybrid.clock.capture(msg.timestamp, self.now)
        if published < capture:
            raise ValueError("px4_sample_after_publish_stamp")
        if role == "vehicle_local_position":
            if tuple(getattr(msg, name, None) for name in RESET_FIELDS) != self.hybrid.local_reset_counters:
                raise ValueError("px4_local_reset_requires_new_epoch")
            report, = payloads("px4_local", msg)
            if self._local is not None and capture < self._local[0]:
                raise ValueError("px4_local_capture_rewind")
            self._local = (capture, report["valid"] and report["heading_good"])
            return
        if msg.reset_counter != self.hybrid.alignment.estimator_reset_counter:
            raise ValueError("px4_odometry_reset_requires_new_epoch")
        self._state = None
        report, = payloads("px4_odom", msg)
        if not report["valid"]:
            self._state_reason = "invalid_px4_frame_or_odometry"
            return
        variances = tuple(vector(getattr(msg, name), 3) for name in
                          ("position_variance", "orientation_variance", "velocity_variance"))
        if (any(value <= 0 for group in variances for value in group)
                or sum(variances[0]) > self.hybrid.vehicle.state_position_error_m ** 2):
            self._state_reason = "invalid_or_excessive_px4_covariance"
            return
        rotation = quat_to_mat(*(np.asarray(msg.q, dtype=float) / math.hypot(*msg.q)))
        forward = rotation[:, 0]
        projection = float(forward[0] ** 2 + forward[1] ** 2)
        if projection < .01:
            self._state_reason = "px4_yaw_singular"
            return
        derivative = rotation @ np.cross(vector(msg.angular_velocity, 3), (1., 0., 0.))
        yaw_rate = (forward[0] * derivative[1] - forward[1] * derivative[0]) / projection
        alignment = self.hybrid.alignment
        state = DroneState(alignment.ned, (capture - alignment.time_origin_ns) / NS,
                           msg.position, msg.velocity, math.atan2(forward[1], forward[0]), yaw_rate)
        if self._last_px4 is not None:
            last_capture, last_position = self._last_px4
            if capture <= last_capture:
                raise ValueError("px4_capture_rewind_or_duplicate")
            if math.dist(state.position_m, last_position) > .5 + 3. * (capture - last_capture) / NS:
                raise ValueError("px4_pose_jump_requires_new_epoch")
        self._last_px4 = (capture, state.position_m)
        self._state = (state, capture, self.now, msg.timestamp_sample, variances)
        self._state_reason = None

    def _flush(self, force=False):
        prior = len(self.rows)
        start = perf_counter_ns()
        super()._flush(force)
        if len(self.rows) == prior:
            return
        row = self.rows[-1]
        row["map_update_ms"] = (perf_counter_ns() - start) / 1e6
        if row["reason"] == "reference_reset_required":
            raise ValueError("vio_pose_reset_requires_new_epoch")
        row["hybrid"] = self.diagnostic() if row["accepted"] else {"reason": row["reason"]}

    def diagnostic(self):
        """Read the most recent synchronized snapshot without storing a trajectory or publisher."""
        if self.fault is not None:
            return {"reason": self.fault}
        if self._pending is not None:
            return {"reason": "awaiting_capture_pose"}
        if not self.rows or not self.rows[-1]["accepted"]:
            return {"reason": "missing_synchronized_depth"}
        snapshot = self.world.snapshot(now_ns=self.now)
        if not snapshot.metric.available:
            return {"reason": "geometry_unavailable"}
        if self.now - snapshot.metric.capture_ns > EvaluationPolicy().map_age_ns:
            return {"reason": "stale_geometry"}
        if (self._tracking is None or not self._tracking[1] or self.now - self._tracking[0] > HEALTH_MAX_AGE_NS
                or any(stamp not in self._visual for stamp in self.rows[-1]["pose_bracket_ns"])):
            return {"reason": "unconfirmed_or_stale_visual_tracking"}
        if (self._health is None or not self._health[1] or self.now - self._health[0] > HEALTH_MAX_AGE_NS
                or self._ev_received is None or self.now - self._ev_received > HEALTH_MAX_AGE_NS
                or any(sample is None or not sample[1] or self.now - sample[0] > HEALTH_MAX_AGE_NS
                       for sample in (self._flags, self._local))):
            return {"reason": "estimator_health_missing_or_stale"}
        if self._state is None:
            return {"reason": self._state_reason}
        state, capture, received, sample_us, variances = self._state
        clock = self.hybrid.clock
        if (self.now > clock.valid_until_ns
                or self.now - capture + clock.max_error_ns > EvaluationPolicy().state_age_ns):
            return {"reason": "stale_state_or_clock_mapping"}
        alignment = replace(self.hybrid.alignment, capture_ns=capture)
        goal = None
        if self.hybrid.mission_goal is not None:
            ref = alignment.map
            context = build_context(snapshot.semantic, self.hybrid.mission_goal,
                                    now=snapshot.semantic["query_timestamp"],
                                    reference=WorldReference(ref.frame_id, ref.epoch, ref.clock_id))
            goal = GoalBinding(context, DeterministicReasoner().propose(context))
        start = perf_counter_ns()
        clock_arg = {} if self.monotonic is None else {"monotonic": self.monotonic}
        result = assess(state, snapshot, now_ns=self.now, alignment=alignment, vehicle=self.hybrid.vehicle,
                        objective=self.hybrid.objective, goal=goal, state_received_ns=received,
                        estimator_reset_counter=alignment.estimator_reset_counter, estimator_healthy=True,
                        budget_s=self.planner_budget_s, **clock_arg)
        plan = result.receipt.plan
        return {"reason": result.validation_reason, "decision": plan.decision, "ranked_ids": plan.ranked_ids,
                "px4_sample_us": sample_us, "state_capture_ns": capture, "state_received_ns": received,
                "depth_capture_ns": snapshot.metric.capture_ns,
                "depth_received_ns": snapshot.metric.received_ns, "assessment_ns": self.now, "px4_variances": variances,
                "goal_id": plan.evaluation.goal_proposal_id if plan.evaluation else None,
                "candidates": [{"id": row.candidate_id, "status": row.status, "reasons": row.reasons}
                               for row in plan.evaluation.candidates] if plan.evaluation else [],
                "h0_ms": (perf_counter_ns() - start) / 1e6}

    def finish(self):
        if self.fault is not None:
            raise ValueError(self.fault)
        try:
            return super().finish()
        except ValueError as error:
            self.fault = str(error)
            self._state = self._pending = None
            raise
