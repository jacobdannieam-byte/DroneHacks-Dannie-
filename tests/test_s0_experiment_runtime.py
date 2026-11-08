from dataclasses import replace

import numpy as np
import pytest

from examples.build_s0_course_ablation import build
from examples.h0_hybrid_smoke import smoke_inputs
from examples.s0_course_experiment import experimental_assess, load_experiment, main
from examples.s0_experiment_geometry import EXPERIMENT_EVIDENCE
from refnav.core import Output
from refnav.sitl_review import validate_receipt
from refnav.world_model.spatial_volume import Cell
from test_s0_course_ablation import MAP, MISSION, PROBE


@pytest.fixture
def setup(tmp_path):
    profile = build(PROBE, tmp_path / 'profile', map_path=MAP, mission_path=MISSION)
    runtime = load_experiment(profile, MAP)
    inputs = smoke_inputs()
    now = inputs['now_ns']
    alignment = replace(runtime.loop.replay.hybrid.alignment, capture_ns=now)
    state = replace(inputs['state'], reference=alignment.ned, position_m=(0., 0., -.76))
    raw = inputs['snapshot'].metric
    metric = replace(raw, reference=alignment.map, origin_index=(-10, -10, -5),
                     _cells=np.zeros_like(raw._cells), _stamps=np.full_like(raw._stamps, now))
    inputs.update(state=state, alignment=alignment, goal=None, snapshot=replace(inputs['snapshot'], metric=metric),
                  objective=runtime.course.stages[1].goal, estimator_reset_counter=alignment.estimator_reset_counter)
    return profile, runtime, inputs


def occupy(metric, point):
    index = np.floor(np.asarray(point) / metric.policy.resolution_m).astype(int) - metric.origin_index
    metric._cells[tuple(index)] = Cell.OCCUPIED


def test_experimental_d0_ranking_and_live_occupied_veto(setup):
    _, runtime, inputs = setup
    metric, now = inputs['snapshot'].metric, inputs['now_ns']
    planned = experimental_assess(runtime.geometry, runtime.identity, **inputs)
    assert planned.receipt.plan.ranked_ids
    assert planned.receipt.plan.evaluation.model_id == 'd0'
    assert any(row.evidence == EXPERIMENT_EVIDENCE for row in planned.receipt.plan.evaluation.candidates)
    assert validate_receipt(planned.receipt, state=inputs['state'], alignment=inputs['alignment'], metric=metric,
                            now_ns=now) == 'proposal_clearance_changed'
    assert np.all(metric.cells_at(now, metric.reference) == Cell.UNKNOWN)
    occupy(metric, (0., 0., .76))
    refused = experimental_assess(runtime.geometry, runtime.identity, **inputs).receipt.plan
    assert not refused.ranked_ids
    previously_clear = {row.candidate_id for row in planned.receipt.plan.evaluation.candidates if row.status == 'feasible'}
    assert all(row.reasons == ('live_occupied_voxel_intersection',)
               for row in refused.evaluation.candidates if row.candidate_id in previously_clear)


def test_nominal_support_exemption_keeps_above_floor_and_outside_column_veto(setup):
    _, runtime, inputs = setup
    metric, now = inputs['snapshot'].metric, inputs['now_ns']
    alignment = inputs['alignment']
    launch = ((0., 0., 0.), (0., 0., -.76))
    occupy(metric, (0., 0., -.04))
    assert runtime.geometry.sweep(metric, now).check(launch, alignment, support_center_ned=launch[0]) is None
    assert runtime.geometry.sweep(metric, now).check(launch, alignment) == 'participant_enclosure_intersection'
    assert runtime.geometry.sweep(metric, now).check(launch, alignment, support_center_ned=(.2, 0., 0.)) == (
        'experimental_support_outside_intended_column')
    occupy(metric, (0., 0., .4))
    assert runtime.geometry.sweep(metric, now).check(launch, alignment, support_center_ned=launch[0]) == (
        'live_occupied_voxel_intersection')


def test_post_geometry_publication_freshness_and_isolation(setup, monkeypatch):
    profile, runtime, inputs = setup
    now = inputs['now_ns']
    runtime._publication_inputs, runtime._publication_phase = inputs, 'go'
    target = (*inputs['state'].position_m, 0.)
    assert runtime.publication_reason(None, None, Output('go', target, stream=True), now, wall_ns=now) is None
    assert runtime.publication_freshness_reason(now + 50_000_001, wall_ns=now) == 'experimental_publication_state_stale'
    assert runtime.publication_freshness_reason(now, wall_ns=now + 50_000_001) == 'experimental_publication_wall_budget'
    monkeypatch.setattr('examples.s0_course_experiment.isolated', lambda: False)
    with pytest.raises(SystemExit, match='2'):
        main(['--profile', str(profile), '--world', str(MAP), '--output', str(profile.parent / 'never.json'),
              '--execute-isolated-sitl'])
    assert not (profile.parent / 'never.json').exists()
