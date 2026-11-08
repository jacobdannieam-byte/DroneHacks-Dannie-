"""Bounded mission progress over validated H1 observations. Owns no flight commands."""

from dataclasses import dataclass
import math

from onboard.adapters.geometry import R_ENU_NED, rot_z
from refnav.core import HYBRID_COMPLETE as COMPLETE
from refnav.hybrid_gateway import target_sweep
from refnav.reasoning.schema import MissionGoal, token
from refnav.trajectory_evaluation import MetricGoal, state_capture_ns
from refnav.waypoint_scan import WaypointScan
from refnav.world_model.spatial_input import NS, stamp_ok


KINDS = frozenset(("waypoint", "corridor", "maze", "gate", "tunnel_approach",
                   "tunnel", "chicane", "landing"))


def map_position(state, alignment):
    return tuple((rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED) @ state.position_m
                 + alignment.ned_origin_in_map_m)


@dataclass(frozen=True)
class Stage:
    stage_id: str
    kind: str
    goal: MetricGoal
    semantic: MissionGoal | None = None
    metric_fallback: bool = False
    alternatives: tuple = ()
    arrival_m: float = .20
    timeout_s: float = 30.
    landing_site_review: str | None = None
    scan_on_arrival: bool = False

    def __post_init__(self):
        if (not token(self.stage_id) or self.kind not in KINDS or not isinstance(self.goal, MetricGoal)
                or (self.semantic is not None and not isinstance(self.semantic, MissionGoal))
                or type(self.metric_fallback) is not bool
                or type(self.alternatives) is not tuple or len(self.alternatives) > 3
                or any(not isinstance(g, MetricGoal) or g.reference != self.goal.reference
                       for g in self.alternatives)
                or len(set((self.goal, *self.alternatives))) != 1 + len(self.alternatives)
                or (self.alternatives and self.semantic is not None)
                or type(self.scan_on_arrival) is not bool or (self.scan_on_arrival and self.kind != 'waypoint')
                or not .05 <= self.arrival_m <= .40 or not 1 <= self.timeout_s <= 120
                or (self.landing_site_review is not None and not token(self.landing_site_review))):
            raise ValueError("invalid mission stage")


class CourseMission:
    """Checkpoint and recovery policy. The existing core Mission owns takeoff and landing."""

    def __init__(self, stages, *, failure_limit=4, no_progress_s=5., observation_s=2., startup_scan=None):
        self.stages = tuple(stages)
        if (not 1 <= len(self.stages) <= 32 or not all(isinstance(s, Stage) for s in self.stages)
                or len({s.stage_id for s in self.stages}) != len(self.stages)
                or len({s.goal.reference for s in self.stages}) != 1
                or any(s.kind == "landing" for s in self.stages[:-1])
                or self.stages[-1].kind != "landing"
                or (startup_scan is not None and (not isinstance(startup_scan, MetricGoal)
                    or startup_scan.reference != self.stages[0].goal.reference))
                or type(failure_limit) is not int or not 1 <= failure_limit <= 10
                or not 1 <= no_progress_s <= 30 or not .25 <= observation_s <= 10):
            raise ValueError("invalid bounded course")
        self.failure_limit, self.no_progress_ns, self.observation_ns = (
            failure_limit, round(no_progress_s * NS), round(observation_s * NS))
        self.index = self.alternative = self._attempts = 0
        self.state, self.reason = "INITIALIZE", "starting"
        self.land_reason = None
        self._stage_start = self._progress = self._unavailable = self._last_arrival = None
        self._last_failure_observation = self._last_plan_observation = None
        self._arrival_hits = self._failures = 0
        self._best = math.inf
        self.startup_scan = startup_scan
        self._startup_scan_pending = startup_scan is not None
        self._scan = None
        self.scan_report = None

    @property
    def stage(self):
        return self.stages[min(self.index, len(self.stages) - 1)]

    @property
    def goal(self):
        return (self.stage.goal, *self.stage.alternatives)[self.alternative]

    @property
    def key(self):
        return self.index, self.alternative

    @property
    def scans_enabled(self):
        return self.startup_scan is not None or any(stage.scan_on_arrival for stage in self.stages)

    @property
    def scan_pending(self):
        return self.land_reason is None and (self._startup_scan_pending or self._scan is not None)

    def _set(self, state, reason):
        self.state, self.reason = state, reason

    def abort(self, reason):
        if self._scan is not None:
            self._scan.fail(reason)
            self.scan_report = self._scan.report()
            self._scan = None
        if self.land_reason is None:
            self.land_reason = reason
        self._set("MISSION_ABORT", self.land_reason)

    def lifecycle(self, phase, *, landed=False, disarmed=False, yielded=False):
        if self.scan_pending and (yielded or phase in ('done', 'land')):
            self.abort('scan_authority_lost' if yielded else 'scan_course_ended')
        if yielded:
            self._set("AUTHORITY_LOST", "owner_yielded")
        elif phase == "done":
            self._set("DISARMED" if disarmed and landed else "SAFE_HOLD_OR_YIELD",
                      "landed_disarmed" if disarmed and landed else "termination_unconfirmed")
        elif phase == "land":
            self._set("LAND", self.land_reason or "owner_landing")
        elif phase in ("wait", "arm", "climb", "hover"):
            label = {"wait": "PREFLIGHT", "arm": "READY", "climb": "TAKEOFF",
                     "hover": "STABILIZE_OBSERVE"}[phase]
            self._set(label, "existing_mission_owner")

    def select(self, now_ns, inputs, binding, reason=None, *, fault=False, wall_ns=None):
        """Return the reviewed checkpoint once its fresh observation gates permit planning."""
        if self.land_reason is not None:
            return None
        if self._stage_start is None:
            self._stage_start = self._progress = now_ns
        if fault:
            self.abort("reference_fenced:" + (reason or "unknown"))
            return None
        if not self._startup_scan_pending and now_ns - self._stage_start >= self.stage.timeout_s * NS:
            self.abort("goal_timeout")
            return None
        if self._scan is not None:
            if inputs is None:
                self.abort('scan_observation_lost:' + (reason or 'unknown'))
                return None
            return self._scan
        if inputs is None:
            self._arrival_hits = 0
            self._set("ESTIMATOR_UNHEALTHY" if reason and any(
                word in reason for word in ("estimator", "state", "tracking", "clock")) else
                      "OBSERVATION_REQUIRED", reason or "missing_observations")
            self._unavailable = now_ns if self._unavailable is None else self._unavailable
            if now_ns - self._unavailable >= self.observation_ns:
                self.abort("observations_unavailable")
            return None
        if inputs["alignment"].map != self.goal.reference:
            self.abort("mission_reference_changed")
            return None
        if self._startup_scan_pending:
            return self._start_scan('startup', self.startup_scan, .20, now_ns, wall_ns)

        stage, metric = self.stage, inputs["snapshot"].metric
        observation = metric.observation_id
        missing_landing = stage.kind == "landing" and stage.landing_site_review is None
        blocked = binding is not None and binding.proposal.intent == "reject_blocked_candidate"
        grounded = (binding is not None and binding.context.context_valid and
                    binding.proposal.target_entity_id is not None and
                    binding.proposal.intent in ("observe_more", "investigate_candidate", "review_alternative") and
                    any(entity.entity_id == binding.proposal.target_entity_id
                        and entity.status not in ("invalidated", "stale", "uncertain")
                        and math.dist(entity.geometry.center, self.goal.position_m) <= 2.
                        for entity in binding.context.entities))
        if missing_landing or blocked or (stage.semantic is not None and not grounded and not stage.metric_fallback):
            self._arrival_hits = 0
            self._unavailable = now_ns if self._unavailable is None else self._unavailable
            self._set("BLOCKED_PATH" if blocked else "OBSERVATION_REQUIRED",
                      "semantic_target_blocked" if blocked else
                      "landing_site_unverified" if missing_landing else "semantic_goal_unavailable")
            if now_ns - self._unavailable >= self.observation_ns:
                self.abort(self.reason)
            return None
        if binding is not None and binding.proposal.intent == 'observe_more':
            # An inspection is never checkpoint arrival, even at the checkpoint.
            from refnav.reasoning.bridge import resolve_inspection
            from refnav.reasoning.validation import validate_proposal
            context = binding.context
            semantic_now = (now_ns - inputs['snapshot'].semantic_time_origin_ns) / NS
            valid = validate_proposal(binding.proposal, context, now=semantic_now, reference=context.reference)
            point, refusal = resolve_inspection(binding, inputs['snapshot'], inputs['state'],
                                                inputs['alignment'], now_ns) if valid.valid else (None, 'invalid_proposal')
            self._arrival_hits = 0
            self._unavailable = now_ns if self._unavailable is None else self._unavailable
            if now_ns - self._unavailable >= self.observation_ns:
                self.abort('inspection_exhausted')
                return None
            self._set('OBSERVATION_REQUIRED', refusal or 'inspection_goal')
            return None if point is None else MetricGoal(self.goal.reference, point)
        self._unavailable = None
        position = map_position(inputs["state"], inputs["alignment"])
        distance = math.dist(position, self.goal.position_m)
        # Require new state and map evidence at two successive arrivals.
        sample = state_capture_ns(inputs["state"], inputs["alignment"])
        if self._arrival_supported(distance, inputs, now_ns):
            prior = self._last_arrival
            if prior is None or (observation != prior[0] and sample > prior[1]):
                self._arrival_hits += 1
                self._last_arrival = (observation, sample)
            if self._arrival_hits >= 2:
                if stage.scan_on_arrival and not self.alternative:
                    return self._start_scan(stage.stage_id, self.goal, stage.arrival_m, now_ns, wall_ns)
                self._advance_arrival(now_ns)
                return None
            self._set("VERIFY_GOAL_COMPLETION", "awaiting_fresh_arrival_confirmation")
            return None
        self._arrival_hits = 0
        if distance < self._best - .04:
            self._best, self._progress = distance, now_ns
        if now_ns - self._progress >= self.no_progress_ns:
            self._recover(now_ns, "no_progress", "NO_PROGRESS")
            return None
        self._last_plan_observation = observation
        self._set("PLAN_AND_NAVIGATE", "goal_grounded" if grounded else "reviewed_metric_goal")
        return self.goal

    def _start_scan(self, scope, goal, arrival_m, now_ns, wall_ns):
        if not stamp_ok(now_ns) or not stamp_ok(wall_ns):
            self.abort('scan_clock_unavailable')
            return None
        self._scan = WaypointScan(scope, goal, arrival_m, now_ns, wall_ns)
        return self._scan

    def _arrival_supported(self, distance, inputs, now_ns):
        state, bounds, alignment = inputs['state'], inputs['vehicle'], inputs['alignment']
        return (distance + bounds.state_position_error_m + alignment.position_error_m <= self.stage.arrival_m
                and target_sweep(state, state.position_m, inputs['snapshot'], alignment, bounds, now_ns) is None)

    def update_scan(self, scan, now_ns):
        if scan is not self._scan:
            raise ValueError('scan does not belong to the active course')
        if scan.state == 'SCAN_REFUSED':
            self.abort(scan.reason)
            return
        self.scan_report = scan.report()
        self._set(scan.state, scan.reason)
        if scan.state == 'SCAN_COMPLETE':
            self._scan = None
            if self._startup_scan_pending:
                self._startup_scan_pending = False
                self._clear_progress(now_ns)
            else:
                self._advance_arrival(now_ns)

    def _advance_arrival(self, now_ns):
        if self.alternative:
            # A reviewed detour is a subgoal. Keep its attempt budget.
            self.alternative = 0
            self._clear_progress(now_ns, keep_attempts=True)
            self._set("REPLAN", "detour_complete")
        elif self.index + 1 == len(self.stages):
            self.land_reason = COMPLETE
            self._set("MISSION_COMPLETE", COMPLETE)
        else:
            self.index += 1
            self._clear_progress(now_ns)
            self._set("SELECT_NEXT_GOAL", "checkpoint_confirmed")

    def _clear_progress(self, now_ns, *, keep_attempts=False):
        self._progress = now_ns
        if not keep_attempts:
            self._stage_start = now_ns
            self._attempts = 0
        self._best, self._arrival_hits, self._failures = math.inf, 0, 0
        self._last_arrival = self._last_failure_observation = self._last_plan_observation = None
        self._unavailable = None

    def _recover(self, now_ns, reason, state):
        self._set(state, reason)
        if self._attempts < len(self.stage.alternatives):
            self._attempts += 1
            self.alternative = self._attempts
            self._clear_progress(now_ns, keep_attempts=True)
        else:
            self.abort(reason)

    def result(self, now_ns, reason):
        if reason == 'observation_wait_verified':
            # Waiting is neither checkpoint progress nor a failed movement.
            # Keep the original inspection start time across fresh frames.
            self._unavailable = now_ns if self._unavailable is None else self._unavailable
            if now_ns - self._unavailable >= self.observation_ns:
                self.abort('inspection_exhausted')
            elif self.land_reason is None:
                self._set('OBSERVATION_WAIT', reason)
            return
        if reason == "eligible_for_offline_review":
            self._failures = 0
            return
        observation = self._last_plan_observation
        self._set("REPLAN" if reason == "deadline_exceeded" else "BLOCKED_PATH", reason)
        if observation is not None and self._last_failure_observation != observation:
            self._failures += 1
            self._last_failure_observation = observation
        if self._failures >= self.failure_limit:
            self._recover(now_ns, reason, "REPLAN")
