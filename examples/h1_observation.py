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
from refnav.integration_timing import measure
from refnav.reasoning import DeterministicReasoner, WorldReference, build_context
from refnav.reasoning.schema import MissionGoal
from refnav.reasoning.selection import ReasonerConfig, create_reasoner
from refnav.reasoning.inspection import make_inputs, Attempt, HISTORY_SECONDS
from refnav.strategies import StrategyConfig
from refnav.trajectory_evaluation import EvaluationPolicy, FrameAlignment, GoalBinding, MetricGoal, VehicleBounds
from refnav.world_model.spatial_input import NS, PoseSample, stamp_ok, vector
from refnav.world_model.spatial_volume import MapPolicy

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

    def source_stamp_us(self, now_ns):
        if not stamp_ok(now_ns) or not self.valid_from_ns <= now_ns <= self.valid_until_ns:
            raise ValueError("cannot stamp command outside the reviewed Gazebo clock epoch")
        stamp = (now_ns - self.offset_ns) // 1000
        if stamp <= 0:
            raise ValueError("Gazebo PX4 command stamp must be positive")
        return stamp

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
    tracking_source: str = "visual"
    reasoner: ReasonerConfig = ReasonerConfig()
    strategy_config: StrategyConfig | None = None

    def __post_init__(self):
        if self.strategy_config is not None:
            if not isinstance(self.strategy_config, StrategyConfig) or self.reasoner != self.strategy_config.reasoner():
                raise ValueError('Strategy and reasoner configurations disagree')
        if not isinstance(self.reasoner, ReasonerConfig):
            raise ValueError('Supply a validated reasoner configuration.')
        if self.tracking_source not in ("visual", "simulated_estimator"):
            raise ValueError("invalid tracking source")
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
        strategy = StrategyConfig(**data['strategy']) if 'strategy' in data else None
        reasoner = ReasonerConfig(**data['reasoner']) if 'reasoner' in data else (strategy.reasoner() if strategy else ReasonerConfig())
        spatial = BagProfile.from_dict(data["spatial"])
        ref = spatial.reference
        alignment = FrameAlignment(LocalNED(data["ned_frame"], ref.epoch, ref.clock_id), ref,
                                   data["alignment"]["time_origin_ns"], **data["alignment"])
        return cls(spatial, ClockMapping(**data["clock"]), alignment, VehicleBounds(**data["vehicle"]),
                   MetricGoal(ref, data["objective_m"]), data["px4_topics"], data["ev_reset_counter"],
                   tuple(data["local_reset_counters"]),
                   MissionGoal.from_dict(data["mission_goal"]) if data.get("mission_goal") is not None else None,
                   data.get("tracking_source", "visual"), reasoner, strategy)


class HybridBagReplay(RecordedSpatialReplay):
    """Reuse the Phase 2 reader, retaining at most one PX4 state and pending depth."""

    def __init__(self, profile, *, planner_budget_s=.025, monotonic=None, continuous=False, timings=None,
                 retain_aligned_depth=False):
        policy = MapPolicy()
        if profile.tracking_source == "simulated_estimator":
            from examples.gazebo_sensor_contract import native_depth_policy
            policy = native_depth_policy()
        super().__init__(profile.spatial, policy=policy, timings=timings, retain_aligned_depth=retain_aligned_depth)
        self.continuous = continuous
        self.dropped_depth = 0
        self.startup_depth_discarded = 0
        if continuous:
            self.rows = deque(maxlen=1)
        self._snapshot = self._goal = None
        self.hybrid = profile
        self.reasoner = create_reasoner(profile.reasoner)
        self.predictor, self.predictor_id, self.predictor_fallback = (profile.strategy_config.load_predictor()
            if profile.strategy_config else (None, None, None))
        self._inspection_last = None
        self._inspection_scope = None
        self._inspection_attempts = ()
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
        self._health_history = deque(maxlen=64)
        self._state_reason = "missing_px4_state"
        self.fault = None
        self.keep_fresh_snapshot_while_pending = False

    @property
    def _keep_pending_snapshot(self):
        return (self.keep_fresh_snapshot_while_pending is True and self.continuous
                and self.hybrid.tracking_source == 'simulated_estimator')

    def feed(self, topic, msg, received_ns):
        if self.fault is not None:
            raise ValueError(self.fault)
        try:
            if topic in TOPICS:
                if topic == "/estimator/healthy" and self.hybrid.tracking_source == "simulated_estimator":
                    self._health_history.append((received_ns, bool(msg.data)))
                if topic == "/fmu/in/vehicle_visual_odometry" and msg.reset_counter != self.hybrid.ev_reset_counter:
                    raise ValueError("ev_reset_requires_new_epoch")
                if topic == "/camera/depth/image_raw":
                    if self.continuous and (not self._extrinsics_ok or not self._info_ok
                                            or not self._clock_progress or self._ev_reset is None):
                        self._advance(topic, received_ns)
                        self._require_clock(received_ns)
                        self.startup_depth_discarded += 1
                        return
                    if not self._extrinsics_ok:
                        raise ValueError("missing_reviewed_depth_extrinsics")
                    if self._pending is not None:
                        if not self.continuous:
                            raise ValueError("depth_queue_overload")
                        self._pending = None
                        if not self._keep_pending_snapshot:
                            self._snapshot = None
                        self.dropped_depth += 1
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

    def _health_for_pose(self, stamp):
        if self.hybrid.tracking_source == "visual":
            return super()._health_for_pose(stamp)
        # A delayed simulated sample needs health at capture and at receipt.
        # Do not reuse a pose across any intervening loss.
        prior = next((row for row in reversed(self._health_history) if row[0] <= stamp), None)
        return (prior is not None and prior[1] and stamp - prior[0] <= HEALTH_MAX_AGE_NS
                and all(ok for seen, ok in self._health_history if seen >= stamp))

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
        good = (row["valid"] and row["result"] == self.hybrid.tracking_source and not row["covariance_rejected"]
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
        prior = self.rows[-1] if self.rows else None
        start = perf_counter_ns()
        super()._flush(force)
        if not self.rows or self.rows[-1] is prior:
            return
        row = self.rows[-1]
        row["map_update_ms"] = (perf_counter_ns() - start) / 1e6
        if row["reason"] == "reference_reset_required":
            raise ValueError("vio_pose_reset_requires_new_epoch")
        self._snapshot = (self.world.snapshot(now_ns=self.now, immutable_semantic=self.continuous)
                          if row["accepted"] else None)
        self._goal = None
        if not self.continuous:
            row["hybrid"] = self.diagnostic() if row["accepted"] else {"reason": row["reason"]}

    def select_goal(self, snapshot, mission_goal, *, stage_id='inspection'):
        ref = self.hybrid.alignment.map
        context = measure(self.timings, 'reasoning_context', build_context, snapshot.semantic, mission_goal,
                                now=snapshot.semantic["query_timestamp"],
                                reference=WorldReference(ref.frame_id, ref.epoch, ref.clock_id))
        if self._state is None:
            return GoalBinding(context, self.reasoner.propose(context))
        state, capture = self._state[:2]
        alignment = replace(self.hybrid.alignment, capture_ns=capture)
        previous = self._inspection_history(context, snapshot, mission_goal, stage_id)
        auxiliary = measure(self.timings, 'inspection_inputs', make_inputs, context, snapshot, state, alignment, self.profile.calibration,
                                attempts=self._inspection_attempts, previous_entity_id=previous, stage_id=stage_id)
        proposal, selection = measure(self.timings, 'reasoner_inference', self.reasoner.select, context, auxiliary)
        entity = next((e for e in context.entities if e.entity_id == selection.selected_entity_id), None)
        if entity is not None:
            self._inspection_last = (context.reference, mission_goal, snapshot.metric.observation_id,
                                     entity.entity_id, entity.missing_evidence)
        return GoalBinding(context, proposal, selection)

    def _inspection_history(self, context, snapshot, mission_goal, stage_id):
        prior = self._inspection_last
        now = context.context_created_at
        scope = (context.reference, mission_goal, stage_id)
        if self._inspection_scope != scope:
            self._inspection_scope = scope
            self._inspection_attempts = ()
            self._inspection_last = None
            return None
        self._inspection_attempts = tuple(a for a in self._inspection_attempts
                                          if 0 <= now - a.stamp_seconds <= HISTORY_SECONDS)
        if prior is None:
            return None
        if prior[2] != snapshot.metric.observation_id:
            entity = next((e for e in context.entities if e.entity_id == prior[3]), None)
            if entity is not None:
                acquired = bool(set(prior[4]) - set(entity.missing_evidence))
                # A new frame is not a completed inspection: the proposed
                # viewpoint may still be moving, or D3 may have refused it.
                # Only changed evidence supports an observed outcome here.
                # Failed attempts require a separate completed-action outcome;
                # the course inspection timeout still bounds unavailable data.
                if acquired:
                    attempt = Attempt(entity.entity_id, now, 'acquired')
                    self._inspection_attempts = (*self._inspection_attempts, attempt)[-32:]
            self._inspection_last = None
        return prior[3]

    @property
    def px4_sample_us(self):
        return None if self._state is None else self._state[3]

    def visual_pose_available(self, now_ns, bracket_ns):
        return (self._tracking is not None and self._tracking[1] and now_ns - self._tracking[0] <= HEALTH_MAX_AGE_NS
                and all(stamp in self._visual for stamp in bracket_ns))

    def prepare(self, now_ns):
        """Return a refusal reason or one normalized H0 input set."""
        if self.fault is not None:
            return self.fault, None
        if not stamp_ok(now_ns) or self.now is None or now_ns < self.now:
            return "unbound_assessment_time", None
        if self._pending is not None and (not self._keep_pending_snapshot or self._snapshot is None):
            return "awaiting_capture_pose", None
        snapshot = self._snapshot
        if snapshot is None:
            return "missing_synchronized_depth", None
        if not self.world.observations_available:
            return "geometry_unavailable", None
        if now_ns - snapshot.metric.capture_ns > EvaluationPolicy().map_age_ns:
            return "stale_geometry", None
        if not self.visual_pose_available(now_ns, self.rows[-1]["pose_bracket_ns"]):
            return "unconfirmed_or_stale_visual_tracking", None
        if (self._health is None or not self._health[1] or now_ns - self._health[0] > HEALTH_MAX_AGE_NS
                or self._ev_received is None or now_ns - self._ev_received > HEALTH_MAX_AGE_NS
                or self._local is None or not self._local[1]
                or not 0 <= now_ns-self._local[0] <= HEALTH_MAX_AGE_NS
                or not self._flags_fresh(now_ns)):
            return "estimator_health_missing_or_stale", None
        if self._state is None:
            return self._state_reason, None
        state, capture, received, _, _ = self._state
        clock = self.hybrid.clock
        if now_ns > clock.valid_until_ns or now_ns - capture + clock.max_error_ns > EvaluationPolicy().state_age_ns:
            return "stale_state_or_clock_mapping", None
        alignment = replace(self.hybrid.alignment, capture_ns=capture)
        if self._goal is None and self.hybrid.mission_goal is not None:
            self._goal = measure(self.timings, "semantic_goal", self.select_goal, snapshot, self.hybrid.mission_goal)
        return None, dict(state=state, snapshot=snapshot, now_ns=now_ns, alignment=alignment,
                          vehicle=self.hybrid.vehicle, objective=self.hybrid.objective, goal=self._goal,
                          state_received_ns=received, estimator_reset_counter=alignment.estimator_reset_counter,
                          estimator_healthy=True, predictor=self.predictor, model_id=self.predictor_id)

    def _flags_fresh(self, now_ns):
        # Match the already-reviewed Gazebo health bridge's 1 Hz flag cadence.
        # Hardware/visual profiles retain their existing 300 ms requirement;
        # fast pose, local position, EV and health checks are unchanged.
        maximum = HEALTH_MAX_AGE_NS
        if self.hybrid.tracking_source == 'simulated_estimator':
            from examples.gazebo_sensor_contract import ESTIMATOR_FLAGS_MAX_AGE_NS
            maximum = ESTIMATOR_FLAGS_MAX_AGE_NS
        return (self._flags is not None and self._flags[1]
                and 0 <= now_ns-self._flags[0] <= maximum)

    def diagnostic(self):
        """Run one H1 replay assessment and summarize its evidence."""
        reason, inputs = self.prepare(self.now)
        if inputs is None:
            return {"reason": "missing_synchronized_depth" if self.now is None and self.fault is None else reason}
        _, capture, received, sample_us, variances = self._state
        snapshot = inputs["snapshot"]
        start = perf_counter_ns()
        clock_arg = {} if self.monotonic is None else {"monotonic": self.monotonic}
        result = assess(**inputs, budget_s=self.planner_budget_s, timings=self.timings, **clock_arg)
        plan = result.receipt.plan
        return {"reason": result.validation_reason, "decision": plan.decision, "ranked_ids": plan.ranked_ids,
                "strategy_identity": self.hybrid.strategy_config.identity if self.hybrid.strategy_config else None,
                "predictor_fallback": self.predictor_fallback,
                "px4_sample_us": sample_us, "state_capture_ns": capture, "state_received_ns": received,
                "depth_capture_ns": snapshot.metric.capture_ns,
                "depth_received_ns": snapshot.metric.received_ns, "assessment_ns": self.now, "px4_variances": variances,
                "goal_id": plan.evaluation.goal_proposal_id if plan.evaluation else None,
                "candidates": [{"id": row.candidate_id, "status": row.status, "reasons": row.reasons,
                                "inflation_m": row.inflation_m,
                                "occupied_segments": row.occupied_segments,
                                "unknown_segments": row.unknown_segments}
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
