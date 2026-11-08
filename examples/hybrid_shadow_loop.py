"""Rate-limited read-only H2 planning over the latest H1 inputs."""

from collections import deque
from dataclasses import dataclass
from time import perf_counter, perf_counter_ns

from examples.h1_observation import HybridBagReplay
from examples.spatial_bag_input import CLOCK_SKEW_NS, _ns
from refnav.hybrid_integration import HybridAssessment, assess
from refnav.integration_timing import Timings, measure
from refnav.trajectory_evaluation import state_capture_ns
from refnav.world_model.spatial_input import NS, stamp_ok


CLOCK_STALL_NS = 300_000_000


@dataclass(frozen=True)
class ShadowTick:
    now_ns: int
    reason: str
    assessment: HybridAssessment | None = None
    completion_wall_ns: int | None = None
    state_receipt_wall_ns: int | None = None
    depth_receipt_wall_ns: int | None = None
    px4_sample_us: int | None = None
    clock_evidence_id: str | None = None

    def report(self):
        row = {"tick_ns": self.now_ns, "reason": self.reason, "completion_wall_ns": self.completion_wall_ns}
        if self.assessment is None:
            return row
        receipt = self.assessment.receipt
        plan = receipt.plan
        capture = state_capture_ns(receipt.state, receipt.alignment)
        row.update(decision=plan.decision, ranked_ids=plan.ranked_ids, epoch=receipt.metric.reference.epoch,
                   clock=receipt.metric.reference.clock_id, reset_counter=receipt.estimator_reset_counter,
                   state_frame=receipt.state.reference.frame_id, map_frame=receipt.metric.reference.frame_id,
                   px4_sample_us=self.px4_sample_us, clock_evidence_id=self.clock_evidence_id,
                   state_capture_ns=capture, state_received_ns=receipt.state_received_ns,
                   depth_capture_ns=receipt.metric.capture_ns, depth_received_ns=receipt.metric.received_ns,
                   observation_id=receipt.metric.observation_id, calibration_id=receipt.metric.calibration_id,
                   state_age_ns=self.now_ns - capture, depth_age_ns=self.now_ns - receipt.metric.capture_ns,
                   state_receipt_age_ns=self.now_ns - receipt.state_received_ns,
                   depth_receipt_age_ns=self.now_ns - receipt.metric.received_ns,
                   deadline_monotonic_s=plan.deadline_monotonic_s,
                   clearance_identity=receipt.clearance_identity)
        if self.state_receipt_wall_ns is not None:
            row["state_receipt_to_decision_wall_ns"] = self.completion_wall_ns - self.state_receipt_wall_ns
        if self.depth_receipt_wall_ns is not None:
            row["depth_receipt_to_decision_wall_ns"] = self.completion_wall_ns - self.depth_receipt_wall_ns
        if plan.evaluation is not None:
            # Observed cells are useful sensing evidence even when the vehicle's
            # own swept volume remains unknown and motion is correctly refused.
            import numpy as np
            from refnav.world_model.spatial_volume import Cell
            cells = receipt.metric.cells_at(self.now_ns, receipt.metric.reference)
            row['metric_cell_counts'] = {name: int(np.count_nonzero(cells == value))
                for name, value in (('unknown', Cell.UNKNOWN), ('observed_free', Cell.FREE), ('occupied', Cell.OCCUPIED))}
            row["goal_id"] = plan.evaluation.goal_proposal_id
            row["candidates"] = [
                {"id": entry.candidate_id, "status": entry.status, "reasons": entry.reasons,
                 "clearance_lower_bound_m": entry.clearance_lower_bound_m,
                 "occupied_segments": entry.occupied_segments, "unknown_segments": entry.unknown_segments,
                 "inflation_m": entry.inflation_m,
                 "evidence": entry.evidence, "prior_backed_segments": entry.prior_backed_segments,
                 "predicted_peak_speed_mps": entry.predicted_peak_speed_mps,
                 "predicted_end_position_ned_m": entry.predicted_end_position_ned_m,
                 "predicted_end_velocity_ned_mps": entry.predicted_end_velocity_ned_mps}
                for entry in plan.evaluation.candidates]
        return row


class ShadowLoop:
    """Single-consumer observation loop. Missed ticks are skipped, never queued."""

    assessor = staticmethod(assess)

    def __init__(self, profile, *, rate_hz=5, budget_s=.025, monotonic=perf_counter,
                 completion_clock=perf_counter_ns, timings=None, measure_receipt_wall=False,
                 retain_aligned_depth=False):
        if type(rate_hz) not in (int, float) or not 1 <= rate_hz <= 20:
            raise ValueError("shadow rate must be between 1 and 20 Hz")
        if type(budget_s) not in (int, float) or not 0 < budget_s <= .05:
            raise ValueError("invalid shadow planner budget")
        self.timings = timings if timings is not None else Timings()
        self.replay = HybridBagReplay(profile, continuous=True, timings=self.timings,
                                      retain_aligned_depth=retain_aligned_depth)
        self.period_ns = round(NS / rate_hz)
        self.budget_s = budget_s
        self.monotonic = monotonic
        self.completion_clock = completion_clock
        self.measure_receipt_wall = measure_receipt_wall
        self._state_receipt_wall = None
        self._depth_receipts_wall = deque(maxlen=16)
        self._last_poll_wall = self._last_tick_ns = None
        self._clock_ns = self._clock_wall_ns = None
        self._fault = None
        self._closed = False
        self.skipped_ticks = 0
        self.filtered = False

    @property
    def fault(self):
        return self._fault

    @property
    def topics(self):
        return self.replay.topics

    def feed(self, topic, msg, received_ns, wall_ns, *, receipt_wall_ns=None):
        if self._closed or self._fault is not None:
            return
        try:
            self.replay.feed(topic, msg, received_ns)
            if self.measure_receipt_wall:
                receipt_wall_ns = wall_ns if receipt_wall_ns is None else receipt_wall_ns
                if topic == "/camera/depth/image_raw":
                    self._depth_receipts_wall.append(((_ns(msg.header.stamp), received_ns), receipt_wall_ns))
                elif topic == self.replay.hybrid.px4_topics["vehicle_odometry"]:
                    self._state_receipt_wall = receipt_wall_ns
            if topic == "/clock":
                clock_ns = _ns(msg.clock)
                if self._clock_ns is None or clock_ns > self._clock_ns:
                    self._clock_ns, self._clock_wall_ns = clock_ns, wall_ns
        except (ValueError, TypeError, AttributeError) as error:
            self._fault = str(error)

    def close(self):
        self._closed = True

    def observation_reason(self, now_ns, wall_ns):
        if self._closed or self._fault is not None:
            return "shadow_closed" if self._closed else self._fault
        if self._clock_wall_ns is None or not 0 <= wall_ns - self._clock_wall_ns <= CLOCK_STALL_NS:
            return "shadow_clock_stalled"
        if abs(now_ns - self._clock_ns) > CLOCK_SKEW_NS:
            return "shadow_clock_unbound"
        return None

    def poll(self, now_ns, wall_ns, *, inputs_filter=None, map_consumer=None):
        """Return the next diagnostic tick when due, including reason-only stops."""
        self.filtered = False
        if not stamp_ok(now_ns) or not stamp_ok(wall_ns):
            self._fault = "invalid_tick_time"
            return ShadowTick(now_ns if stamp_ok(now_ns) else 0, self._fault)
        if ((self._last_poll_wall is not None and wall_ns < self._last_poll_wall)
                or (self._last_tick_ns is not None and now_ns < self._last_tick_ns)):
            self._fault = "shadow_clock_rewind"
        if self._closed or self._fault is not None:
            return ShadowTick(now_ns, "shadow_closed" if self._closed else self._fault)
        if self._last_poll_wall is not None:
            elapsed = wall_ns - self._last_poll_wall
            if elapsed < self.period_ns:
                return None
            self.skipped_ticks += elapsed // self.period_ns - 1
        self._last_poll_wall, self._last_tick_ns = wall_ns, now_ns
        clock_reason = self.observation_reason(now_ns, wall_ns)
        if clock_reason:
            return ShadowTick(now_ns, clock_reason)
        reason, inputs = self.replay.prepare(now_ns)
        if inputs is None:
            return ShadowTick(now_ns, reason)
        if inputs_filter is not None:
            self.filtered = True
            reason, inputs = inputs_filter(inputs, now_ns)
            if inputs is None:
                return ShadowTick(now_ns, reason)
        result = measure(self.timings, "h0_total", self.assessor, **inputs, budget_s=self.budget_s,
                         map_consumer=map_consumer,
                         monotonic=self.monotonic, timings=self.timings,
                         cancelled=lambda: self._closed or self._fault is not None)
        self.timings.record("state_capture_to_tick_ros", now_ns - state_capture_ns(inputs["state"], inputs["alignment"]))
        self.timings.record("depth_capture_to_tick_ros", now_ns - inputs["snapshot"].metric.capture_ns)
        completed = self.completion_clock()
        state_wall = self._state_receipt_wall
        metric = inputs["snapshot"].metric
        depth_wall = next((wall for key, wall in self._depth_receipts_wall
                           if key == (metric.capture_ns, metric.received_ns)), None)
        if state_wall is not None:
            self.timings.record("state_receipt_to_decision_wall", completed - state_wall)
        if depth_wall is not None:
            self.timings.record("depth_receipt_to_decision_wall", completed - depth_wall)
        return ShadowTick(now_ns, result.validation_reason, result, completed, state_wall, depth_wall,
                          self.replay.px4_sample_us, self.replay.hybrid.clock.evidence_id)
