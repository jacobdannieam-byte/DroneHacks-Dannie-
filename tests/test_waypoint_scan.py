"""Synthetic telemetry through the real course scheduler and shadow runtime."""

from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np
import pytest

from examples.h0_hybrid_smoke import smoke_inputs
from examples.hybrid_mission_smoke import runtime_fixture
from examples.hybrid_runtime import HybridRuntime
from examples.unified_spatial_replay import calibration
from onboard.adapters.geometry import R_ENU_NED, R_FRD_FLU, rot_z
from refnav.dynamics.validation import wrap_yaw
from refnav.hybrid_mission import CourseMission, Stage, map_position
from refnav.stopping_envelope import StoppingEnvelope
from refnav.trajectory_evaluation import MetricGoal
from refnav.waypoint_scan import SECTOR_RAD, SECTORS
from refnav.world_model.spatial_input import AlignedDepth, Alignment, NS
from refnav.world_model.spatial_volume import Cell


class ScanReplay:
    def __init__(self, monkeypatch, *, startup=True, arrival=True, timeout_s=30.):
        self.inputs = smoke_inputs()
        self.state, self.alignment = self.inputs['state'], self.inputs['alignment']
        self.snapshot = self.inputs['snapshot']
        self.now = NS
        self.frame = 0
        self.last_depth = None
        self.prepared = None
        self.observation_failure = None
        base = runtime_fixture(strategy='hybrid_shadow')
        profile = base.loop.replay.hybrid
        self.goal = MetricGoal(self.alignment.map, map_position(self.state, self.alignment))
        self.course = CourseMission((Stage('observe', 'waypoint', self.goal, timeout_s=timeout_s,
                                           scan_on_arrival=arrival),
                                     Stage('land', 'landing', self.goal, landing_site_review='fixture')),
                                    startup_scan=self.goal if startup else None)
        # This fixture deliberately injects aligned observations at the H1-to-course seam.
        self.runtime = HybridRuntime(replace(profile, spatial=replace(profile.spatial, reference=self.goal.reference)),
                                     self.course, monotonic=lambda: 0.)
        self.runtime.executor.stopping_envelope = StoppingEnvelope(.05, .03, .2, .1, .02, 'a' * 64)
        monkeypatch.setattr(self.runtime.loop.replay, 'prepare', lambda _: (None, self.prepared))
        monkeypatch.setattr(self.runtime.loop, 'observation_reason', lambda *_: self.observation_failure)

    def tick(self, yaw=0., *, yaw_rate=0., speed=0., fresh_depth=True, depth_yaw=None, state_age_ns=0,
             reset=0, cell=Cell.FREE, wall_ns=None, position=None):
        self.now += 125_000_000
        capture = self.now - state_age_ns
        state = replace(self.state, timestamp_s=capture / NS, yaw_rad=wrap_yaw(yaw), yaw_rate_rad_s=yaw_rate,
                        position_m=self.state.position_m if position is None else position,
                        velocity_mps=(speed, 0., 0.))
        alignment = replace(self.alignment, capture_ns=capture, estimator_reset_counter=reset)
        if fresh_depth:
            self.frame += 1
            rotation = (rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED
                        @ rot_z(yaw if depth_yaw is None else depth_yaw) @ R_FRD_FLU)
            pose = Alignment(map_position(state, alignment), rotation, .01, .01, (capture, capture))
            self.last_depth = AlignedDepth(alignment.map, capture, capture, capture, f'scan-{self.frame}',
                                           calibration(), np.ones((2, 2)), pose)
        depth = self.last_depth
        metric = replace(self.snapshot.metric, capture_ns=depth.capture_ns, query_ns=self.now,
                         received_ns=depth.received_ns, observation_id=depth.observation_id,
                         _cells=np.full_like(self.snapshot.metric._cells, cell),
                         _stamps=np.full_like(self.snapshot.metric._stamps, depth.capture_ns))
        self.prepared = self.inputs | {'state': state, 'alignment': alignment, 'estimator_reset_counter': reset,
                                       'snapshot': replace(self.snapshot, metric=metric), 'now_ns': self.now}
        self.runtime.loop.replay.aligned_depth = depth
        row = self.runtime.poll(self.now, self.now if wall_ns is None else wall_ns)
        assert row is not None and row.assessment is None
        assert self.runtime.executor.held is None and not self.runtime._motion_allowed
        return row

    @property
    def report(self):
        return self.runtime.status()['waypoint_scan']


def test_startup_and_selected_waypoint_each_gate_progress_through_measured_revolution(monkeypatch):
    replay = ScanReplay(monkeypatch, timeout_s=20.)
    start_yaw = math.radians(170.)
    for scope in ('startup', 'observe'):
        for _ in range(7):
            replay.tick(start_yaw)
        assert replay.report['scope'] == scope
        assert replay.report['state'] == 'SCAN_OBSERVE'
        observed = set()
        for sector in range(SECTORS):
            assert replay.course.index == 0
            if sector:
                replay.tick(start_yaw + sector * SECTOR_RAD)
            for quarter in range(1, 5):
                yaw = start_yaw + (sector + quarter / 4) * SECTOR_RAD
                replay.tick(yaw, yaw_rate=.52 if quarter < 4 else 0.)
            assert replay.report['completed_sectors'] == sector
            if sector == 0:
                replay.tick(yaw, fresh_depth=False)
                replay.tick(yaw, depth_yaw=yaw - SECTOR_RAD)
                assert replay.report['completed_sectors'] == sector
            replay.tick(yaw)
            assert replay.report['completed_sectors'] == sector + 1
            observed.add(replay.report['observation_id'])
        assert replay.report['state'] == 'SCAN_COMPLETE'
        assert replay.report['measured_turn_rad'] == pytest.approx(math.tau)
        assert replay.report['command_authority'] is False and replay.report['target_ned'] is None
        assert len(observed) == SECTORS
        assert replay.course.index == (scope == 'observe')


def test_startup_requires_measured_stability_and_cannot_reuse_state(monkeypatch):
    replay = ScanReplay(monkeypatch)
    for _ in range(6):
        replay.tick(speed=.08)
    assert replay.report['state'] == 'SCAN_STABILIZE'
    for _ in range(4):
        replay.tick()
    assert replay.report['target_ned'] is None
    # A replayed state is stale at the configured 8 Hz tick, even with fresh depth.
    replay.tick(state_age_ns=125_000_000)
    assert replay.report['state'] == 'SCAN_REFUSED'
    assert replay.report['reason'] == 'scan_state_stale_or_rewound'
    assert replay.course.index == 0
    replay.tick()
    assert not replay.course.scan_pending


@pytest.mark.parametrize(('failure', 'reason'), (
    ('envelope', 'scan_stopping_envelope_unavailable'),
    ('unknown', 'scan_clearance_refused:unobserved_swept_volume'),
    ('low_clearance', 'scan_clearance_refused:insufficient_clearance'),
    ('reset', 'scan_reference_changed'),
    ('observation', 'scan_observation_lost:estimator_unhealthy'),
    ('wall', 'scan_timeout'),
    ('authority', 'scan_authority_lost'),
))
def test_scan_refuses_and_does_not_reenter(monkeypatch, failure, reason):
    replay = ScanReplay(monkeypatch)
    for _ in range(4):
        replay.tick()
    if failure == 'envelope':
        replay.runtime.executor.stopping_envelope = None
    if failure == 'low_clearance':
        import examples.hybrid_runtime as runtime_module

        row = SimpleNamespace(status='feasible', clearance_lower_bound_m=.05, reasons=())
        monkeypatch.setattr(runtime_module, 'evaluate_candidates', lambda *_, **__: SimpleNamespace(candidates=(row,)))
    replay.tick(cell=Cell.UNKNOWN if failure == 'unknown' else Cell.FREE)
    if failure == 'observation':
        replay.observation_failure = 'estimator_unhealthy'
        replay.tick()
    if failure == 'reset':
        replay.tick(reset=1)
    if failure == 'wall':
        replay.tick(wall_ns=replay.now + 36 * NS)
    if failure == 'authority':
        replay.course.lifecycle('done', yielded=True)
    assert replay.report['reason'] == reason
    assert replay.report['state'] == 'SCAN_REFUSED' and replay.report['target_ned'] is None
    assert replay.course.index == 0 and not replay.course.scan_pending


def test_waypoint_scan_cannot_extend_stage_deadline(monkeypatch):
    replay = ScanReplay(monkeypatch, startup=False, timeout_s=1.)
    for _ in range(9):
        replay.tick()
    assert replay.report['reason'] == 'goal_timeout'
    assert replay.report['state'] == 'SCAN_REFUSED' and replay.course.index == 0


def test_scan_enabled_course_cannot_attach_native_owner(monkeypatch):
    replay = ScanReplay(monkeypatch)
    native = runtime_fixture()
    with pytest.raises(ValueError, match='shadow-only'):
        HybridRuntime(replay.runtime.loop.replay.hybrid, replay.course, strategy='hybrid_sitl', adapter=native.adapter)
    with pytest.raises(ValueError, match='command path is not attached'):
        replay.runtime.owner_step(None, None, now_ns=NS, wall_ns=NS, generation=1, admission=True, owner_ready=True)
    with pytest.raises(ValueError, match='invalid mission stage'):
        Stage('land', 'landing', replay.goal, scan_on_arrival=True)
