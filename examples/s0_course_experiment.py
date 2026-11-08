"""Run the isolated, unqualified S0 participant-map course experiment."""

import argparse
from hashlib import sha256
import json
import math
import os
from pathlib import Path
from time import perf_counter, perf_counter_ns

import numpy as np

from examples.build_s0_course_ablation import SCOPE, participant_geometry
from examples.gazebo_container import _ports, isolated, verify_profile
from examples.gazebo_hybrid_adapter import IsolationApproval, OWNER_ROLES
from examples.hybrid_launch import configured
from examples.hybrid_runtime import HybridRuntime
from examples.s0_experiment_geometry import AuthoredObstacles, EXPERIMENT_EVIDENCE
from examples.s0_owner_clock import ExperimentalOwnerClockMixin
from onboard.adapters.geometry import R_ENU_NED, rot_z
from refnav.dynamics import DynamicsModel, PositionSetpoint
from refnav.dynamics.history import STEP_SECONDS
from refnav.dynamics.validation import wrap_yaw
from refnav.hybrid_execution import Held, HeldExecutor
from refnav.hybrid_gateway import GatewayDecision
from refnav.hybrid_integration import HybridAssessment, _input_reason
from refnav.hybrid_mission import CourseMission
from refnav.predictive_planner import (
    STEPS, Plan, _generate, candidate_for_id, has_progress, inspection_objective, inspection_yaw,
)
from refnav.sitl_review import ProposalReceipt
from refnav.trajectory_evaluation import Candidate, CandidateDiagnostic, Evaluation, _forecast, _resolve_goal, state_capture_ns
from refnav.world_model.spatial_input import NS, stamp_ok


# From the pinned simulator model's reviewed body, camera and rotating propeller
# collision envelopes (0.303078 m rounded upward). This is geometry, not stopping evidence.
NOMINAL_BODY_RADIUS_M = .304


def _digest(path):
    return sha256(Path(path).read_bytes()).hexdigest()


class ExperimentCourse(CourseMission):
    def __init__(self, stages, geometry, **settings):
        super().__init__(stages, **settings)
        self.geometry = geometry

    def _arrival_supported(self, distance, inputs, now_ns):
        return (distance + self.geometry.reserve_m <= self.stage.arrival_m
                and self.geometry.sweep(inputs['snapshot'].metric, now_ns).check(
                    (inputs['state'].position_m,), inputs['alignment']) is None)


def _path(candidate, state, alignment, sweep, *, timings=None):
    points, effort, end, speed = _forecast(candidate, state, DynamicsModel(), None, timings)
    reason = sweep.check(points, alignment) or sweep.check((state.position_m, candidate.schedule[0].position_m), alignment)
    return reason, points, effort, end, speed


def experimental_assess(geometry, identity, state, snapshot, *, now_ns, alignment, vehicle, objective,
                        state_received_ns, estimator_reset_counter, estimator_healthy, goal=None,
                        budget_s=.05, monotonic=perf_counter, cancelled=None, timings=None, **_):
    """Use S0's own G0 binding, D5 candidate schedules and D0 rollouts with nominal obstacles."""
    metric = snapshot.metric
    reason = _input_reason(state, metric, alignment, now_ns, state_received_ns,
                           estimator_reset_counter, estimator_healthy, objective)
    deadline = monotonic() + budget_s
    decision = Plan('safe_stop_requested', reason or 'no_experimental_progress')
    point, _, reasons = _resolve_goal(snapshot, goal or objective, now_ns, state, alignment) if not reason else (None, None, ())
    if reason is None and point is None:
        reason = next(iter(reasons), 'experimental_goal_unavailable')
        decision = Plan('safe_stop_requested', reason)
    if reason is None:
        target = inspection_objective(state, objective, alignment, goal, snapshot, now_ns)
        candidates = tuple(_generate(state, target, alignment))
        yaw = inspection_yaw(state, alignment, goal, snapshot, now_ns)
        if yaw is not None:
            candidates = (candidates[0], yaw)
        transform = rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED
        sweep = geometry.sweep(metric, now_ns)
        rows = []
        for candidate in candidates:
            if monotonic() >= deadline or (cancelled is not None and cancelled()):
                break
            refusal, points, effort, end, speed = _path(candidate, state, alignment, sweep, timings=timings)
            mapped = points[[0, -1]] @ transform.T + alignment.ned_origin_in_map_m
            progress = math.dist(mapped[0], point) - math.dist(mapped[1], point)
            inflation = (vehicle.state_position_error_m + vehicle.tracking_error_m + alignment.position_error_m
                         + 2 * math.sin(alignment.angle_error_rad / 2) * max(math.hypot(*p) for p in points))
            angular = None
            if yaw is not None and candidate == yaw:
                heading = yaw.schedule[0].yaw_rad
                angular = abs(wrap_yaw(heading - state.yaw_rad)) - abs(wrap_yaw(heading - end.yaw_rad))
            rows.append(CandidateDiagnostic(candidate.candidate_id, 'rejected' if refusal else 'feasible',
                (refusal or EXPERIMENT_EVIDENCE,), 'occupied' if refusal else EXPERIMENT_EVIDENCE,
                progress_m=progress, effort=effort, score=progress - .02 * effort, inflation_m=inflation,
                predicted_end_position_ned_m=end.position_m, predicted_end_velocity_ned_mps=end.velocity_mps,
                predicted_end_yaw_rad=end.yaw_rad, predicted_peak_speed_mps=speed, inspection_yaw_progress_rad=angular))
        eligible = sorted((row for row in rows if row.status == 'feasible' and row.candidate_id != 'hold'
                           and has_progress(row)), key=lambda row: (-(row.score - .10 * row.inflation_m), row.candidate_id))
        ranked = tuple(row.candidate_id for row in eligible)
        if monotonic() >= deadline or (cancelled is not None and cancelled()):
            ranked, reason = (), 'experimental_planner_interrupted'
        elif not ranked:
            reason = 'no_experimental_progress'
        evaluation = Evaluation(tuple(rows), ranked, reason, 'd0', metric.reference, metric.capture_ns,
                                metric.observation_id, metric.calibration_id, alignment.provenance,
                                goal.proposal.proposal_id if goal else None, identity)
        decision = Plan('candidate_proposed' if ranked else 'safe_stop_requested',
                        EXPERIMENT_EVIDENCE if ranked else reason, ranked, evaluation, deadline)
    receipt = ProposalReceipt(decision, state, alignment, metric, now_ns, state_received_ns,
                              estimator_reset_counter, identity)
    return HybridAssessment(receipt, reason or 'eligible_for_offline_review', objective, goal, vehicle)


class ExperimentExecutor(HeldExecutor):
    def __init__(self, geometry, identity, **kwargs):
        super().__init__(enabled=True, **kwargs)
        self.geometry, self.identity = geometry, identity

    def _safe(self, inputs, target, now_ns):
        state, metric = inputs['state'], inputs['snapshot'].metric
        cmd = PositionSetpoint(state.reference, state.timestamp_s, state.timestamp_s + STEPS * STEP_SECONDS,
                               target[:3], target[3])
        candidate = Candidate('experimental-held', (cmd,) * STEPS)
        return _path(candidate, state, inputs['alignment'], self.geometry.sweep(metric, now_ns))[0]

    def step(self, *, inputs, assessment, key, now_ns, owner_output, vehicle, ready, generation):
        reason = self._owner_reason(inputs, now_ns, vehicle, ready)
        if reason:
            self.invalidate()
            return GatewayDecision(reason, None)
        state, alignment, metric = inputs['state'], inputs['alignment'], inputs['snapshot'].metric
        if assessment is not None:
            self.invalidate()
            receipt = assessment.receipt
            plan = receipt.plan
            if (receipt.clearance_identity == self.identity and receipt.metric is metric
                    and receipt.state == state and receipt.alignment == alignment and receipt.decision_ns == now_ns
                    and receipt.state_received_ns == inputs['state_received_ns']
                    and receipt.estimator_reset_counter == inputs['estimator_reset_counter']
                    and assessment.objective == inputs['objective'] and assessment.goal == inputs['goal']
                    and plan.ranked_ids and self.clock() < plan.deadline_monotonic_s):
                candidate = candidate_for_id(state, assessment.objective, alignment, plan.ranked_ids[0],
                    goal=assessment.goal, snapshot=inputs['snapshot'], now_ns=now_ns)
                if candidate is not None:
                    command = candidate.schedule[0]
                    target = (*command.position_m, command.yaw_rad)
                    reason = self._safe(inputs, target, now_ns)
                    if reason is None and self.clock() < plan.deadline_monotonic_s:
                        self.held = Held(target, generation, key, metric.reference, metric.calibration_id,
                                         now_ns + round(self.lease_s * NS), self.clock() + self.lease_s, self.identity)
                        return GatewayDecision('experimental_s0_candidate_selected', target)
        elif self.held is not None:
            held = self.held
            if ((held.generation, held.key, held.reference, held.calibration, held.clearance_identity)
                    == (generation, key, metric.reference, metric.calibration_id, self.identity)
                    and now_ns < held.ros_expires_ns and self.clock() < held.wall_expires_s):
                reason = self._safe(inputs, held.target, now_ns)
                if reason is None and self.clock() < held.wall_expires_s:
                    return GatewayDecision('experimental_s0_held_revalidated', held.target)
            self.invalidate()
        target = (*state.position_m, state.yaw_rad)
        reason = self._safe(inputs, target, now_ns)
        return (GatewayDecision('experimental_s0_stationary_hold', target) if reason is None
                else GatewayDecision('experimental_obstacle_refused:' + reason, None))


class _GroundAlignmentGuard:
    """Verify that the frozen paired translation still fits the landed vehicle."""

    def __init__(self, replay):
        self.replay = replay

    def check_owner(self, mission, vehicle, inputs, *, now_ns, wall_ns):
        if mission.phase not in ('wait', 'arm') or vehicle.armed is not False:
            return None
        if inputs is None or vehicle.landed is not True or vehicle.pos is None:
            return 'experimental_ground_state_unavailable'
        state, alignment, metric = inputs['state'], inputs['alignment'], inputs['snapshot'].metric
        depth = self.replay.aligned_depth
        if (depth is None or depth.reference != metric.reference or depth.observation_id != metric.observation_id
                or depth.capture_ns != metric.capture_ns
                or abs(state_capture_ns(state, alignment) - depth.capture_ns) > 150_000_000
                or math.hypot(*state.velocity_mps) > .05):
            return 'experimental_ground_alignment_unsettled'
        transform = rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED
        native = transform @ np.asarray(state.position_m) + alignment.ned_origin_in_map_m
        if (math.dist(native, depth.pose.position_m) > .03 or math.hypot(*native) > .04
                or math.dist(state.position_m, vehicle.pos) > .03):
            return 'experimental_ground_alignment_disagrees'
        return None


class ExperimentalRuntime(HybridRuntime):
    def __init__(self, profile, course, adapter, geometry, identity, **settings):
        settings.setdefault('record_limit', 8192)
        super().__init__(profile, course, strategy='hybrid_sitl', adapter=adapter, **settings)
        self.geometry, self.identity = geometry, identity
        self.executor = ExperimentExecutor(geometry, identity, lease_s=self.executor.lease_s,
                                           monotonic=self.executor.clock, timings=self.timings)
        self.loop.assessor = lambda **inputs: experimental_assess(geometry, identity, **inputs)
        self.loop.replay._retain_aligned_depth = True
        self.loop.replay.keep_fresh_snapshot_while_pending = True
        self.launch_guard = _GroundAlignmentGuard(self.loop.replay)
        self._checking_publication_wall_ns = None
        self.native_readiness = None
        self._readiness = None

    def _owner_readiness(self, now_ns, **checks):
        row = {**checks, 'native': self.native_readiness}
        if row != self._readiness:
            self._readiness = row
            self._record('experimental_owner_readiness', now_ns)

    def status(self):
        return {**super().status(), 'experimental_owner_readiness': self._readiness}

    def publication_reason(self, mission, vehicle, output, now_ns, *, wall_ns=None):
        inputs = self._publication_inputs
        self._checking_publication_wall_ns = None
        if not output.stream and not output.commands:
            return None
        self._checking_publication_wall_ns = perf_counter_ns() if wall_ns is None else wall_ns
        if inputs is None:
            return 'experimental_publication_inputs_unavailable'
        state, snapshot, alignment = inputs['state'], inputs['snapshot'], inputs['alignment']
        reason = _input_reason(state, snapshot.metric, alignment, now_ns, inputs['state_received_ns'],
                               inputs['estimator_reset_counter'], inputs['estimator_healthy'], self.course.goal)
        if reason:
            return reason
        sweep = self.geometry.sweep(snapshot.metric, now_ns)
        if 'land' in output.commands:
            # PX4 owns descent; check the intended vertical descent column.
            ground_down = alignment.ned_origin_in_map_m[2] + self.geometry.course_from_odom_translation_m[2] - .04
            target = (*state.position_m[:2], ground_down)
            return sweep.check((state.position_m, target), alignment, support_center_ned=target)
        if not output.stream or output.setpoint is None:
            return 'experimental_setpoint_unavailable'
        if self._publication_phase in ('wait', 'arm', 'climb', 'hover'):
            ground_reason = self.launch_guard.check_owner(mission, vehicle, inputs, now_ns=now_ns, wall_ns=wall_ns)
            if ground_reason:
                return ground_reason
            if mission.home is None or tuple(output.setpoint[:2]) != mission.home[:2] or output.setpoint[2] > mission.home[2]:
                return 'experimental_launch_outside_home_column'
            climb = (*mission.home[:2], mission.home[2] - mission.p.height_m)
            support = mission.home[:3] if self._publication_phase != 'hover' else None
            return (sweep.check((state.position_m, climb), alignment, support_center_ned=support)
                    or sweep.check((state.position_m, output.setpoint[:3]), alignment, support_center_ned=support))
        return self.executor._safe(inputs, output.setpoint, now_ns)

    def publication_freshness_reason(self, now_ns, *, wall_ns=None, vehicle=None):
        if self._checking_publication_wall_ns is None:
            return None
        wall_ns = perf_counter_ns() if wall_ns is None else wall_ns
        if not 0 <= wall_ns - self._checking_publication_wall_ns <= 50_000_000:
            return 'experimental_publication_wall_budget'
        inputs = self._publication_inputs
        if inputs is None:
            return 'experimental_publication_inputs_unavailable'
        state, metric, alignment = inputs['state'], inputs['snapshot'].metric, inputs['alignment']
        if not 0 <= now_ns - state_capture_ns(state, alignment) <= 50_000_000:
            return 'experimental_publication_state_stale'
        return _input_reason(state, metric, alignment, now_ns, inputs['state_received_ns'],
                             inputs['estimator_reset_counter'], inputs['estimator_healthy'], self.course.goal)


def load_experiment(profile_path, world_path, *, verify_native=False):
    profile_path, world_path = Path(profile_path), Path(world_path)
    settings = json.loads(profile_path.read_text(encoding='utf-8'))
    digest = _digest(world_path)
    if (settings.get('experiment_scope') != SCOPE or settings.get('participant_map_sha256') != digest
            or settings.get('strategy') != 'hybrid_shadow'
            or any(key in settings for key in ('map_assisted', 'stopping_envelope', 'simulator_launch'))):
        raise ValueError('explicit experiment profile and exact participant world required')
    base = verify_profile(profile_path, 'hybrid_shadow') if verify_native else configured(profile_path)[1]
    profile = base.loop.replay.hybrid
    try:
        if (not base.adapter.simulated_estimator or not base.adapter.can_attach_owner or base.course.scans_enabled
                or profile.strategy_config is None or profile.strategy_config.identifier != 'hybrid_existing'
                or (verify_native and base.adapter.owner_topics != {role: _ports()[role] for role in OWNER_ROLES})):
            raise ValueError('exact S0 G0/D0 native experiment inputs required')
        map_source = participant_geometry(world_path)
        yaw = settings['course_from_odom_yaw_rad']
        origin = tuple(settings['course_from_odom_translation_m'])
        if (profile.alignment.map_from_enu_yaw_rad != -math.pi / 2
                or yaw != math.pi / 2 or origin != (*map(float, map_source.launch_mark_centre_metres), .04)):
            raise ValueError('experimental odom/course axes or origin mismatch')
        pairing = settings['ground_alignment']
        transform = rot_z(profile.alignment.map_from_enu_yaw_rad) @ R_ENU_NED
        native = np.asarray(pairing['native_position_ned_m'], dtype=float)
        visual = np.asarray(pairing['visual_position_odom_m'], dtype=float)
        stamp, bracket = pairing['visual_capture_ns'], pairing['native_bracket_ns']
        offset = tuple(pairing['ned_origin_in_map_m'])
        if (pairing['method'] != 'stationary-native-to-visual-translation-v1'
                or pairing['native_reset_counter'] != profile.alignment.estimator_reset_counter
                or native.shape != (3,) or visual.shape != (3,) or not np.isfinite((native, visual)).all()
                or not stamp_ok(stamp) or len(bracket) != 2 or not all(stamp_ok(v) for v in bracket)
                or not bracket[0] <= stamp <= bracket[1] or bracket[1] - bracket[0] > 40_000_000
                or offset != profile.alignment.ned_origin_in_map_m
                or not np.allclose(visual - transform @ native, offset, atol=1e-10, rtol=0.)):
            raise ValueError('experimental profile lacks the matching stationary native/visual alignment')
        reserve = settings['nominal_obstacle_reserve_m']
        geometry = AuthoredObstacles.from_course(map_source, NOMINAL_BODY_RADIUS_M, reserve,
                    course_from_odom_yaw_rad=yaw, course_from_odom_translation_m=origin)
        course_file = profile_path.parent / settings['mission_profile']
        route = json.loads(course_file.read_text(encoding='utf-8'))
        if route['participant_route']['map_sha256'] != digest:
            raise ValueError('course route must use the identical participant map')
        course = ExperimentCourse(base.course.stages, geometry, **route.get('recovery', {}))
        identity = (SCOPE, digest, _digest(course_file), _digest(profile_path))
        return ExperimentalRuntime(profile, course, base.adapter, geometry, identity,
                                   **settings.get('runtime', {}), **settings.get('logging', {}))
    finally:
        base.close()


def run(runtime, output, ros_args):
    if not isolated():
        raise ValueError('experimental owner requires a network-none isolated simulator without serial devices')
    if Path(output).exists():
        raise ValueError('choose a new experiment log')
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.utilities import try_shutdown
    from onboard.diagnostics.sitl_graph import inspect
    from refnav.node import RefNav

    class ExperimentalOwner(ExperimentalOwnerClockMixin, RefNav):
        def _validate_native_qualification(self, current):
            if current is not runtime or type(current) is not ExperimentalRuntime or not isolated():
                raise ValueError('isolated experiment owner context changed')

    permit = IsolationApproval(runtime.adapter.profile_id, int(os.environ['ROS_DOMAIN_ID']), SCOPE,
                               lambda *_: isolated())
    height = runtime.course.stage.goal.position_m[2]
    rclpy.init(args=['s0_course_experiment', '--ros-args', '-p', 'use_sim_time:=true',
                     '-p', f'height_m:={height}', '-p', 'hover_s:=1.0',
                     '-p', 'control_topic:=/gazebo_hybrid/control', '-p', f'run_id:={SCOPE}', *ros_args])
    node, code = None, 1
    executor = SingleThreadedExecutor()
    try:
        probe = rclpy.create_node('s0_experiment_graph_probe', enable_rosout=False, start_parameter_services=False)
        try:
            until = perf_counter() + 4.
            while rclpy.ok() and perf_counter() < until:
                rclpy.spin_once(probe, timeout_sec=.1)
            if not rclpy.ok() or inspect(probe, strict_motion=True)['issues']:
                raise ValueError('motion graph is not idle')
        finally:
            probe.destroy_node()
        node = ExperimentalOwner(hybrid_runtime=runtime, hybrid_approval=permit)
        executor.add_node(node)
        executor.spin()
    except SystemExit as stop:
        code = int(stop.code or 0)
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    finally:
        runtime.close()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        try_shutdown()
        callback_clock = (None if node is None else dict(fault=node._owner_clock_fault,
                         pending=len(node._owner_clock_pending), retained_releases=node._owner_clock_audit))
        with Path(output).open('x', encoding='utf-8') as stream:
            json.dump(dict(scope=SCOPE, qualification=False, obstacle_backend=EXPERIMENT_EVIDENCE,
                           identity=runtime.identity, records=list(runtime.records), total_records=runtime.record_count,
                           status=runtime.status(),
                           last_issue=runtime.last_issued, timing=runtime.timings.summary(), exit_code=code,
                           callback_clock=callback_clock,
                           h1_clock=runtime.clock_fence.report() if runtime.clock_fence else None),
                      stream, indent=2, allow_nan=False)
    return code


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--execute-isolated-sitl', action='store_true')
    args, ros_args = parser.parse_known_args(argv)
    try:
        if args.execute_isolated_sitl and (args.output is None or not isolated()):
            raise ValueError('execution requires explicit output and network-none isolated SITL')
        runtime = load_experiment(args.profile, args.world, verify_native=args.execute_isolated_sitl)
        if args.execute_isolated_sitl:
            return run(runtime, args.output, ros_args[1:] if ros_args[:1] == ['--ros-args'] else ros_args)
        print(json.dumps(dict(scope=SCOPE, execution=False, stages=len(runtime.course.stages), identity=runtime.identity)))
        runtime.close()
        return 0
    except (ValueError, OSError, KeyError, RuntimeError, TypeError, AttributeError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    raise SystemExit(main())
