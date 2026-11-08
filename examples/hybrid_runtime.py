"""Single-executor composition of H1/H2, checkpoint policy and the existing mission owner."""

from collections import deque
from dataclasses import replace
import math
from time import perf_counter, perf_counter_ns

from examples.hybrid_shadow_loop import CLOCK_STALL_NS, ShadowLoop
from refnav.core import NAV_OFFBOARD, Output
from refnav.hybrid_execution import HeldExecutor
from refnav.hybrid_mission import CourseMission, map_position
from refnav.integration_timing import measure
from refnav.trajectory_evaluation import MetricGoal
from refnav.world_model.spatial_input import NS


STRATEGIES = ("classical", "hybrid_shadow", "hybrid_sitl")


class HybridRuntime:
    """One input stream, one map and one owner. Modes never create a publisher here."""

    def __init__(self, profile, course, *, strategy="hybrid_shadow", adapter=None, plan_hz=8,
                 semantic_hz=5, budget_s=.025, lease_s=.25, record_limit=2048, monotonic=perf_counter,
                 completion_clock=perf_counter_ns):
        if (strategy not in STRATEGIES[1:] or not isinstance(course, CourseMission)
                or not 1 <= semantic_hz <= plan_hz <= 20
                or type(record_limit) is not int or not 128 <= record_limit <= 8192):
            raise ValueError("invalid Hybrid strategy or update rates")
        if strategy == "hybrid_sitl" and (adapter is None or not adapter.can_attach_owner):
            raise ValueError("hybrid_sitl needs the reviewed isolated native adapter")
        if course.stage.goal.reference != profile.spatial.reference:
            raise ValueError("course must use the reviewed H1 map reference")
        self.strategy, self.course, self.adapter = strategy, course, adapter
        from examples.sim_clock_fence import SimulationClockFence
        self.clock_fence = SimulationClockFence(profile.clock) if profile.tracking_source == 'simulated_estimator' else None
        if strategy == 'hybrid_sitl' and profile.reasoner.variant != 'G0':
            raise ValueError('Experimental goal reasoners are restricted to read-only shadow.')
        self.loop = ShadowLoop(replace(profile, mission_goal=None), rate_hz=plan_hz, budget_s=budget_s,
                               monotonic=monotonic, completion_clock=completion_clock,
                               measure_receipt_wall=True)
        self.executor = HeldExecutor(enabled=strategy == "hybrid_sitl", lease_s=lease_s,
                                     monotonic=monotonic, timings=self.loop.timings)
        self.semantic_period_ns = round(NS / semantic_hz)
        self._semantic_sample = self._semantic_key = self._semantic_metric = self._binding = None
        self._objective = None
        self._motion_allowed = False
        self.records = deque(maxlen=record_limit)
        self.record_count = 0
        self._course_started = False
        self.last_issued = None
        self._last_status = None
        self._owner_clock = None
        self.owner_watchdog_fault = None

    @property
    def timings(self):
        return self.loop.timings

    @property
    def topics(self):
        return self.loop.topics if self.adapter is None else self.adapter.topics

    @property
    def fault(self):
        failure = self.clock_fence.failure if self.clock_fence else None
        return self.loop.fault or (failure['reason'] if failure else None)

    def feed(self, topic, message, received_ns, wall_ns, *, receipt_wall_ns=None):
        canonical = topic if self.adapter is None else self.adapter.canonical(topic)
        try:
            rows = (self.clock_fence.accept(canonical, message, received_ns, wall_ns)
                    if self.clock_fence else [(canonical, message, received_ns, wall_ns)])
            for name, data, ingestion_ns, ingestion_wall in rows:
                self.loop.feed(name, data, ingestion_ns, ingestion_wall, receipt_wall_ns=receipt_wall_ns)
        except ValueError:
            self.invalidate()
            self.loop.close()
            raise

    def close(self):
        if self.clock_fence: self.clock_fence.close()
        self.executor.invalidate()
        self.loop.close()

    def _semantic(self, inputs, now_ns):
        stage = self.course.stage
        if stage.semantic is None:
            return None
        snapshot = inputs["snapshot"]
        key = self.course.key
        if snapshot.metric is self._semantic_metric and key == self._semantic_key:
            return self._binding
        if (self._semantic_sample is not None and key == self._semantic_key
                and now_ns - self._semantic_sample < self.semantic_period_ns):
            return None
        self._semantic_key, self._semantic_metric, self._semantic_sample = key, snapshot.metric, now_ns
        self._binding = measure(self.timings, "semantic_goal", self.loop.replay.select_goal, snapshot, stage.semantic,
                                stage_id=stage.stage_id)
        return self._binding

    def _select(self, inputs, now_ns):
        self._motion_allowed = False
        self._course_started = True
        binding = self._semantic(inputs, now_ns)
        if self.course.stage.semantic is not None and self._semantic_metric is not inputs["snapshot"].metric:
            self.course.select(now_ns, None, None, "waiting_for_semantic_refresh")
            return self.course.reason, None
        goal = self.course.select(now_ns, inputs, binding)
        if goal is None:
            return self.course.reason, None
        state, alignment = inputs["state"], inputs["alignment"]
        start = map_position(state, alignment)
        distance = math.dist(start, goal.position_m)
        fraction = min(1., .50 / distance) if distance else 1.
        local = tuple(a + (b - a) * fraction for a, b in zip(start, goal.position_m))
        self._objective = MetricGoal(goal.reference, local)
        self._motion_allowed = True
        return None, {**inputs, "objective": self._objective,
                      "goal": binding if self.course.reason in ("goal_grounded", 'inspection_goal') else None}

    def poll(self, now_ns, wall_ns):
        """Run the course in shadow, or once per due cycle during existing-owner navigation."""
        row = self.loop.poll(now_ns, wall_ns, inputs_filter=self._select)
        if row is not None:
            if row.assessment is not None:
                self.course.result(now_ns, row.reason)
                self._motion_allowed &= row.reason == "eligible_for_offline_review"
            elif not self.loop.filtered:
                self._motion_allowed = False
                if self._course_started or self.fault is not None:
                    self.course.select(now_ns, None, None, row.reason, fault=self.fault is not None)
                elif row.reason != "shadow_closed":
                    self.course.lifecycle("wait")
            details = row.report()
            if self._binding is not None and self._semantic_key == self.course.key:
                details["semantic_proposal"] = self._binding.proposal.to_dict()
            self._record("plan", now_ns, **details)
        return row

    def _record(self, event, now_ns, **detail):
        self.record_count += 1
        self.records.append({"sequence": self.record_count, "event": event, **self.status(),
                             "time_ns": now_ns, **detail})

    def status(self):
        stage = self.course.stage
        return {"strategy": self.strategy, "mission_state": self.course.state,
                "reasoning_strategy": (self.loop.replay.hybrid.strategy_config.identifier
                                       if self.loop.replay.hybrid.strategy_config else 'legacy_reasoner_config'),
                "strategy_identity": (self.loop.replay.hybrid.strategy_config.identity
                                      if self.loop.replay.hybrid.strategy_config else None),
                "predictor_fallback": self.loop.replay.predictor_fallback,
                "mission_reason": self.course.reason, "stage_id": stage.stage_id, "stage_kind": stage.kind,
                "goal_m": self.course.goal.position_m, "goal_slot": self.course.alternative,
                "landing_request": self.course.land_reason}

    def issue(self, now_ns, *, px4_issue_us, setpoint):
        # Native owner calls after publication. A fake sink can label its own timestamps separately.
        self.last_issued = {"ros_issue_ns": now_ns, "px4_issue_us": px4_issue_us, "setpoint": setpoint}
        self._record("issue", now_ns, **self.last_issued)

    def invalidate(self):
        self.executor.invalidate()

    def expire_owner_target(self, now_ns, wall_ns):
        held = self.executor.held
        if held is None:
            return False
        ros_expired = now_ns >= held.ros_expires_ns
        wall_expired = self.executor.clock() >= held.wall_expires_s
        if not (ros_expired or wall_expired):
            return False
        self.invalidate()
        self._record("owner_target_expired", now_ns, wall_ns=wall_ns,
                     ros_expired=ros_expired, wall_expired=wall_expired)
        return True

    def watch_owner_clock(self, mission, now_ns, wall_ns, *, control_pending=False):
        """Latch a clock fault once the native owner has begun priming or flight."""
        if self.owner_watchdog_fault is not None or (mission.phase == "done" and not control_pending):
            return False
        previous = self._owner_clock
        reason = None
        if previous is not None:
            previous_ros_ns, progress_wall_ns = previous
            if now_ns < previous_ros_ns:
                reason = "owner_ros_clock_rewind"
            elif wall_ns - progress_wall_ns >= CLOCK_STALL_NS:
                # Check before accepting new progress in case the executor
                # missed the watchdog deadline and /clock just resumed.
                reason = "owner_ros_clock_stalled"
        if previous is None or now_ns != previous[0]:
            self._owner_clock = (now_ns, wall_ns) if now_ns > 0 else None
        if reason is None or mission.phase == "wait":
            return False
        self.owner_watchdog_fault = reason
        self.close()
        self.course.abort(reason)
        self.course.lifecycle("done", yielded=True)
        mission.yield_hybrid_authority(now_ns / NS, reason)
        self._record("owner_watchdog", now_ns, reason=reason, wall_ns=wall_ns,
                     owner_phase=mission.phase, result=mission.result)
        return True

    def owner_step(self, mission, vehicle, *, now_ns, wall_ns, generation, admission, owner_ready):
        """Pure mission integration. Admission and ready come from the existing ROS owner."""
        if self.strategy != "hybrid_sitl" or not mission.hybrid_navigation:
            raise ValueError("command path is not attached to Hybrid SITL")
        now = now_ns / NS
        clock_reason = self.loop.observation_reason(now_ns, wall_ns)
        reason, inputs = ((clock_reason, None) if clock_reason else self.loop.replay.prepare(now_ns))
        sensors_ready = admission and owner_ready and inputs is not None
        priming_ready = sensors_ready and vehicle.landed is True
        # After the first arming request, losing a verified source stops startup.
        # The core always handles authority loss before landing or streaming.
        lost_startup = mission.phase == "arm" and mission.t_cmd is not None and not sensors_ready
        lost_takeoff = mission.phase in ("climb", "hover") and not sensors_ready
        lost_admission = not admission and (mission.phase in ("climb", "hover", "go", "land")
                                           or (mission.phase == "arm" and mission.t_cmd is not None))
        if lost_admission:
            out = mission.yield_hybrid_authority(now, "isolated owner admission lost")
        else:
            if (lost_startup or lost_takeoff) and not (
                    mission.offboard_seen and vehicle.nav_state != NAV_OFFBOARD):
                self.course.abort(reason or "fresh owner inputs unavailable")
                mission.request_land(self.course.land_reason)
            out = mission.step(now, vehicle, hybrid_ready=priming_ready)
        yielded = (mission.result or "").startswith("yielded:")
        self.course.lifecycle(out.phase, landed=vehicle.landed is True,
                              disarmed=vehicle.armed is False, yielded=yielded)
        phase_identity = (out.phase, mission.result)
        if phase_identity != self._last_status:
            self._last_status = phase_identity
            self._record("owner_phase", now_ns, owner_phase=out.phase, result=mission.result,
                         armed=vehicle.armed, nav_state=vehicle.nav_state, landed=vehicle.landed)
        if out.phase != "go" or out.commands:
            self.invalidate()
            return out

        key = self.course.key
        row = self.poll(now_ns, wall_ns)
        assessment = row.assessment if row is not None else None
        if not self._motion_allowed or self.course.key != key:
            self.invalidate()
            assessment = None
        if self.course.land_reason is None:
            if inputs is not None:
                metric = inputs["snapshot"].metric
                binding_current = self._semantic_metric is metric and self._semantic_key == self.course.key
                if self.course.stage.semantic is not None and not binding_current:
                    self.invalidate()
                inputs = {**inputs, "objective": self._objective or self.course.goal,
                          "goal": self._binding if binding_current and self._motion_allowed
                          and self.course.reason == "goal_grounded" else None}
            # Owner review needs an Output in its existing schema. This marker never reaches PX4.
            owner_contract = Output("go", (*vehicle.pos, vehicle.heading) if vehicle.pos is not None else None,
                                    stream=True)
            decision = self.executor.step(inputs=inputs, assessment=assessment, key=self.course.key,
                                          now_ns=now_ns, owner_output=owner_contract, vehicle=vehicle,
                                          ready=admission and owner_ready, generation=generation)
            self._record("owner_decision", now_ns, reason=decision.reason, target=decision.setpoint,
                         armed=vehicle.armed, nav_state=vehicle.nav_state, landed=vehicle.landed,
                         reset_counter=inputs["estimator_reset_counter"] if inputs else None)
            if decision.setpoint is not None:
                return replace(out, setpoint=decision.setpoint, stream=True)
            self.course.abort(reason or decision.reason)
        self.invalidate()
        self._record("landing_request", now_ns)
        mission.request_land(self.course.land_reason)
        return mission.step(now, vehicle)
