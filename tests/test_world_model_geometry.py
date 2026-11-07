"""Synthetic sensor geometry, including adversarial unknown-space cases."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from examples.semantic_world_model_demo import H, W, K, MOUNT, observation, render, scene
from onboard.adapters.geometry import R_RIG_OPTICAL, rot_x, rot_z
from refnav.world_model import Config, WorldState, openings
from refnav.world_model.geometry import extract
from refnav.world_model.observations import project


def test_projection_uses_full_world_pose_and_camera_translation():
    obs = observation(position=(2., 3., 1.), yaw=0.7)
    obs.R_world_body = rot_z(0.7) @ rot_x(0.15)
    rays = project(obs, Config())
    np.testing.assert_allclose(rays.origin, obs.t_world_body + obs.R_world_body @ MOUNT)
    np.testing.assert_allclose(rays.directions[0], obs.R_world_body @ R_RIG_OPTICAL @ np.array([-K[2]/K[0], -K[3]/K[1], 1.]))
    np.testing.assert_allclose(rays.points()[0], rays.origin + rays.directions[rays.valid][0] * rays.depths[rays.valid][0])


def test_opening_is_supported_by_two_walls_and_finite_returns_behind():
    result = extract(observation(), Config())
    assert sum(d.semantic_type == 'wall' for d in result.detections) >= 2
    detected = [d for d in result.detections if d.semantic_type == 'opening']
    assert len(detected) == 1
    candidate = detected[0]
    assert abs(candidate.width_m - 1.2) < 0.13
    np.testing.assert_allclose(candidate.center[:2], [4., 0.], atol=0.08)
    assert candidate.height_m is None
    assert candidate.support['behind_fraction'] >= 0.9
    assert 'upper_boundary' in candidate.missing_evidence


@pytest.mark.parametrize('yaw,position', [(0.12, (.25,.15,1.2)), (-.2, (.1,-.2,1.2)), (.25, (0.,.1,1.2))])
def test_world_coordinates_stay_consistent_across_perspectives(yaw, position):
    result = extract(observation(position=position, yaw=yaw), Config())
    candidates = [d for d in result.detections if d.semantic_type == 'opening']
    assert len(candidates) == 1
    np.testing.assert_allclose(candidates[0].center[:2], [4., 0.], atol=.1)
    assert abs(candidates[0].width_m - 1.2) < .14


@pytest.mark.parametrize('rectangles', [scene(blocked=True), scene(rear=False),
                                       [(0,4.,(-3.,-.6),(-1.,3.)), (0,7.,(-5.,5.),(-1.,4.))]])
def test_no_opening_from_solid_wall_unknown_gap_or_single_boundary(rectangles):
    result = extract(observation(rectangles=rectangles), Config())
    assert not any(d.semantic_type == 'opening' for d in result.detections)


@pytest.mark.parametrize('value', [np.nan, np.inf, -np.inf, 0., -1., 100.])
def test_invalid_or_out_of_range_depth_never_becomes_free(value):
    state = WorldState()
    assert state.process(observation(depth=np.full((H, W), value)), 0.)['accepted']
    assert not state.snapshot()['entities']
    assert state.space_at([2.,0.,1.2], 0.) == 'unknown'


def test_missing_patch_in_solid_wall_is_not_a_passage():
    depth = render(scene(blocked=True))
    depth[:, 64:96] = np.nan
    result = extract(observation(depth=depth), Config())
    assert not any(d.semantic_type == 'opening' for d in result.detections)
    assert result.diagnostics['rejected_gaps'] >= 1


def test_small_blocking_obstacle_inside_gap_rejects_passage():
    obstacles = scene() + [(0,3.9,(-.12,.12),(.8,1.6))]
    result = extract(observation(rectangles=obstacles), Config())
    assert not any(d.semantic_type == 'opening' for d in result.detections)


def test_two_distinct_openings_in_one_frame():
    rectangles = [(0,4.,(-3.,-1.2),(-1.,3.)), (0,4.,(-.5,.5),(-1.,3.)),
                  (0,4.,(1.2,3.),(-1.,3.)), (0,7.,(-5.,5.),(-1.,4.))]
    state = WorldState()
    state.process(observation(rectangles=rectangles), 0.)
    first = openings(state)
    assert len(first) == 2
    state.process(observation(.2, rectangles=rectangles, yaw=.05), .2)
    second = openings(state)
    assert len(second) == 2
    assert {e['entity_id'] for e in first} == {e['entity_id'] for e in second}


def test_sampled_ray_metric_evidence_expires_and_does_not_fill_unknown_volume():
    state = WorldState()
    state.process(observation(), 0.)
    assert state.space_at([2.,0.,1.2], 0.) == 'observed_free'
    assert state.space_at([7.,0.,1.2], 0.) == 'occupied'
    assert state.space_at([8.,0.,1.2], 0.) == 'unknown'
    assert state.space_at([2.,5.,1.2], 0.) == 'unknown'
    assert state.space_at([2.,0.,1.2], 3.) == 'unknown'


def test_floor_alone_is_not_a_vertical_wall():
    result = extract(observation(rectangles=[(2,0.,(-10.,10.),(-10.,10.))]), Config())
    assert not result.detections


def test_projection_has_hard_ray_cap():
    config = Config(max_rays=100)
    assert len(project(observation(), config).valid) <= 100


def test_parallel_sides_and_observed_depth_support_corridor_hypothesis():
    rectangles = [(1,-1.,(.3,6.),(-1.,3.)), (1,1.,(.3,6.),(-1.,3.)),
                  (0,7.,(-4.,4.),(-1.,3.))]
    result = extract(observation(rectangles=rectangles), Config())
    corridors = [d for d in result.detections if d.semantic_type == 'corridor']
    assert len(corridors) == 1
    assert abs(corridors[0].width_m - 2.) < .1
    assert corridors[0].height_m is None
    assert 'full_volume_coverage' in corridors[0].missing_evidence


def test_single_side_surface_does_not_support_corridor():
    rectangles = [(1,-1.,(.3,6.),(-1.,3.)), (0,7.,(-4.,4.),(-1.,3.))]
    result = extract(observation(rectangles=rectangles), Config())
    assert not any(d.semantic_type == 'corridor' for d in result.detections)
