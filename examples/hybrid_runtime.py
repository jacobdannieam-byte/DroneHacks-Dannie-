"""Single-executor composition of H1/H2, checkpoint policy and the existing mission owner."""

from collections import deque
from dataclasses import replace
import math
from time import perf_counter, perf_counter_ns

from examples.hybrid_shadow_loop import CLOCK_STALL_NS, ShadowLoop
from refnav.core import NAV_OFFBOARD, Output
from refnav.dynamics import PositionSetpoint
from refnav.dynamics.history import STEP_SECONDS
from refnav.hybrid_execution import HeldExecutor
from refnav.hybrid_gateway import target_sweep
from refnav.hybrid_mission import CourseMission, map_position
from refnav.integration_timing import measure
from refnav.predictive_planner import STEPS
from refnav.trajectory_evaluation import CLEARANCE_PROBE_M, Candidate, MetricGoal, evaluate_candidates
from refnav.waypoint_scan import WaypointScan
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
        if course.scans_enabled and strategy == 'hybrid_sitl':
            raise ValueError('Waypoint scans are shadow-only until native yaw execution is reviewed')
        if course.scans_enabled and plan_hz < 5:
            raise ValueError('Waypoint scans need a diagnostic planning rate of at least 5 Hz')
        if course.stage.goal.reference != profile.spatial.reference:
            raise ValueError("course must use the reviewed H1 map reference")
        self.strategy, self.course, self.adapter = strategy, course, adapter
        self.launch_guard = None
        self.map_certificate = self.map_clearance = None
        self.map_source_valid = None
        self.recovery_guard = None
        self._publication_inputs = self._publication_phase = None
        from examples.sim_clock_fence import SimulationClockFence
        self.clock_fence = SimulationClockFence(profile.clock) if profile.tracking_source == 'simulated_estimator' else None
        if strategy == 'hybrid_sitl' and profile.reasoner.variant != 'G0':
            raise ValueError('Experimental goal reasoners are restricted to read-only shadow.')
        self.loop = ShadowLoop(replace(profile, mission_goal=None), rate_hz=plan_hz, budget_s=budget_s,
                               monotonic=monotonic, completion_clock=completion_clock,
                               measure_receipt_wall=True, retain_aligned_depth=course.scans_enabled)
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
            self.close()
            raise

    def close(self):
        if self.clock_fence: self.clock_fence.close()
        self.executor.invalidate()
        self.loop.close()
        if self.course.scan_pending and self.course.land_reason is None:
            self.course.abort('scan_observation_ended')

    def install_map_clearance(self, lease, verify_exclusion, *, monotonic_ns=perf_counter_ns):
        """Deployment-only lease attachment; configuration alone grants nothing."""
        from refnav.map_assisted_clearance import MapAssistedClearance
        if self.map_certificate is None or not callable(self.map_source_valid) or not callable(verify_exclusion):
            raise ValueError('bound participant map and live exclusion verifier required')
        if self.executor.stopping_envelope is None:
            raise ValueError('reviewed stopping envelope required for map-assisted clearance')
        def verified(current):
            return self.map_source_valid() is True and verify_exclusion(current) is True
        self.map_clearance = MapAssistedClearance(self.map_certificate, lease, verified,
            stopping_envelope=self.executor.stopping_envelope, monotonic_ns=monotonic_ns)
        self.invalidate()

    def _map_inputs(self, inputs, now_ns):
        if self.map_certificate is None:
            return None, inputs
        if self.map_clearance is None:
            return 'map_clearance_lease_unavailable', None
        if inputs is None:
            return 'map_clearance_inputs_unavailable', None
        reason = self.map_clearance.reason(inputs['alignment'], now_ns)
        return (reason, None) if reason else (None, self.map_clearance.attach(inputs))

    def publication_reason(self, mission, vehicle, output, now_ns, *, wall_ns=None):
        """Final deployment fence immediately before native output publication."""
        if not output.commands and not (output.stream and output.setpoint is not None):
            return None
        inputs = self._publication_inputs
        phase = self._publication_phase
        if phase in ('wait', 'arm', 'climb', 'hover') and self.launch_guard is not None:
            reason = self.launch_guard.check_output(mission, vehicle, inputs, output, phase=phase,
                now_ns=now_ns, wall_ns=perf_counter_ns() if wall_ns is None else wall_ns)
            if reason:
                return reason
        if self.map_certificate is None:
            return None
        reason, inputs = self._map_inputs(inputs, now_ns)
        if reason:
            return reason
        if output.phase == 'land' or 'land' in output.commands:
            if self.recovery_guard is None:
                return 'map_assisted_recovery_unqualified'
            return self.recovery_guard(mission, vehicle, inputs, output, now_ns)
        if output.phase == 'go' and output.setpoint is not None:
            from refnav.hybrid_integration import _input_reason
            reason = _input_reason(inputs['state'], inputs['snapshot'].metric, inputs['alignment'], now_ns,
                inputs['state_received_ns'], inputs['estimator_reset_counter'], inputs['estimator_healthy'],
                inputs.get('objective') or self.course.goal)
            if reason:
                return reason
            if self.executor.stopping_envelope is None:
                return 'map_stopping_envelope_unavailable'
            return target_sweep(inputs['state'], output.setpoint[:3], inputs['snapshot'],
                inputs['alignment'], inputs['vehicle'], now_ns, stopping_envelope=self.executor.stopping_envelope)
        return None

    def publication_freshness_reason(self, now_ns, *, wall_ns=None, vehicle=None):
        """Cheap freshness fence after potentially slow source/geometry checks."""
        if (self._publication_phase in ('wait', 'arm', 'climb', 'hover')
                and self.launch_guard is not None and vehicle is not None):
            reason = self.launch_guard.freshness_reason(vehicle, self._publication_inputs,
                now_ns=now_ns, wall_ns=perf_counter_ns() if wall_ns is None else wall_ns)
            if reason:
                return reason
        if self.map_certificate is None:
            return None
        from refnav.trajectory_evaluation import state_capture_ns
        inputs = self._publication_inputs
        if inputs is None or self.map_clearance is None:
            return 'publication_inputs_unavailable'
        if not 0 <= now_ns - state_capture_ns(inputs['state'], inputs['alignment']) <= 50_000_000:
            return 'publication_state_stale'
        reason = self.map_clearance.observation_reason(inputs['snapshot'].metric, inputs['alignment'], now_ns)
        if reason:
            return reason
        lease = self.map_clearance.lease
        wall_ns = self.map_clearance.monotonic_ns()
        if not lease.valid_from_ns <= now_ns < lease.valid_until_ns or not lease.wall_from_ns <= wall_ns < lease.wall_until_ns:
            return 'publication_clearance_expired'
        return None

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

    def _scan_clearance(self, inputs, now_ns, target):
        state, snapshot, alignment = inputs['state'], inputs['snapshot'], inputs['alignment']
        envelope, bounds = self.executor.stopping_envelope, inputs['vehicle']
        if envelope is None:
            return 'scan_stopping_envelope_unavailable'
        if self.loop.replay.predictor_fallback is not None:
            return 'scan_predictor_unavailable'
        if math.hypot(*state.velocity_mps) > envelope.max_speed_mps:
            return 'scan_stopping_speed_unverified'
        radius = max(bounds.radius_m, envelope.body_radius_m) + envelope.stop_distance_m
        if radius + CLEARANCE_PROBE_M > 2.:
            return 'scan_stopping_volume_outside_bound'
        bounds = replace(bounds, radius_m=radius)
        clock = self.loop.monotonic
        deadline = clock() + self.loop.budget_s

        def interrupted():
            return 'scan_clearance_deadline' if clock() >= deadline else None

        command = PositionSetpoint(state.reference, state.timestamp_s, state.timestamp_s + STEPS * STEP_SECONDS,
                                   target[:3], target[3])
        candidate = Candidate('waypoint-scan-sector', (command,) * STEPS)
        row = evaluate_candidates(state, (candidate,), snapshot, now_ns=now_ns, alignment=alignment,
                                  vehicle=bounds, predictor=inputs.get('predictor'), model_id=inputs.get('model_id'),
                                  interrupted=interrupted, timings=self.timings).candidates[0]
        if row.status != 'feasible' or row.clearance_lower_bound_m is None:
            return 'scan_clearance_refused:' + next(iter(row.reasons), 'unverified')
        if row.clearance_lower_bound_m < CLEARANCE_PROBE_M:
            return 'scan_clearance_refused:insufficient_clearance'
        reason = target_sweep(state, target[:3], snapshot, alignment, bounds, now_ns)
        return interrupted() or ('scan_clearance_refused:' + reason if reason else None)

    def _select(self, inputs, now_ns, wall_ns):
        self._motion_allowed = False
        self._course_started = True
        reason, inputs = self._map_inputs(inputs, now_ns)
        if reason:
            self.course.abort(reason)
            self.invalidate()
            return reason, None
        binding = None if self.course.scan_pending else self._semantic(inputs, now_ns)
        if (not self.course.scan_pending and self.course.stage.semantic is not None
                and self._semantic_metric is not inputs["snapshot"].metric):
            self.course.select(now_ns, None, None, "waiting_for_semantic_refresh", wall_ns=wall_ns)
            return self.course.reason, None
        goal = self.course.select(now_ns, inputs, binding, wall_ns=wall_ns)
        if isinstance(goal, WaypointScan):
            goal.step(now_ns, wall_ns, inputs, self.loop.replay.aligned_depth,
                      lambda target: self._scan_clearance(inputs, now_ns, target))
            self.course.update_scan(goal, now_ns)
            return goal.reason, None
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
        row = self.loop.poll(now_ns, wall_ns, inputs_filter=lambda inputs, tick: self._select(inputs, tick, wall_ns))
        if row is not None:
            if row.assessment is not None:
                self.course.result(now_ns, row.reason)
                self._motion_allowed &= row.reason == "eligible_for_offline_review"
            elif not self.loop.filtered:
                self._motion_allowed = False
                if self._course_started or self.fault is not None:
                    self.course.select(now_ns, None, None, row.reason, fault=self.fault is not None, wall_ns=wall_ns)
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
        status = {"strategy": self.strategy, "mission_state": self.course.state,
                "reasoning_strategy": (self.loop.replay.hybrid.strategy_config.identifier
                                       if self.loop.replay.hybrid.strategy_config else 'legacy_reasoner_config'),
                "strategy_identity": (self.loop.replay.hybrid.strategy_config.identity
                                      if self.loop.replay.hybrid.strategy_config else None),
                "predictor_fallback": self.loop.replay.predictor_fallback,
                "mission_reason": self.course.reason, "stage_id": stage.stage_id, "stage_kind": stage.kind,
                "goal_m": self.course.goal.position_m, "goal_slot": self.course.alternative,
                "landing_request": self.course.land_reason}
        if self.course.scan_report is not None:
            status['waypoint_scan'] = self.course.scan_report
        if self.map_certificate is not None:
            status['clearance_mode'] = 'controlled_sitl_participant_map'
            status['clearance_lease_installed'] = self.map_clearance is not None
        return status

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
        if self.course.scans_enabled:
            raise ValueError('Waypoint scans have no native command authority')
        now = now_ns / NS
        clock_reason = self.loop.observation_reason(now_ns, wall_ns)
        observation_reason, inputs = ((clock_reason, None) if clock_reason else self.loop.replay.prepare(now_ns))
        map_reason, inputs = self._map_inputs(inputs, now_ns)
        reason = observation_reason or map_reason
        self._publication_inputs, self._publication_phase = inputs, mission.phase
        sensors_ready = admission and owner_ready and inputs is not None
        launch_reason = None
        if self.launch_guard is not None and mission.phase in ('wait','arm','climb','hover'):
            launch_reason = self.launch_guard.check_owner(mission, vehicle, inputs, now_ns=now_ns, wall_ns=wall_ns)
            if launch_reason is not None:
                reason = launch_reason
                sensors_ready = False
                self.invalidate()
        priming_ready = sensors_ready and vehicle.landed is True
        self._owner_readiness(now_ns, observation_reason=observation_reason, map_reason=map_reason,
                              launch_reason=launch_reason, owner_ready=owner_ready, admission=admission,
                              priming_ready=priming_ready)
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

    def _owner_readiness(self, now_ns, **checks):
        """Optional composition diagnostics. The standard owner gates stay local."""
