from dataclasses import replace
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from examples.semantic_world_model_demo import H, W, observation, replay, scene
from refnav.world_model import Config, WorldState, openings


def test_identity_survives_movement_occlusion_staleness_and_blocking():
    state, frames = replay()
    assert not frames[0]['openings']
    ids = [f['openings'][0]['entity_id'] for f in frames[1:]]
    assert len(set(ids)) == 1
    assert frames[3]['openings'][0]['last_observed'] == .4
    assert frames[4]['openings'][0]['status'] == 'stale'
    assert frames[4]['openings'][0]['geometric_confidence'] < frames[3]['openings'][0]['geometric_confidence']
    assert openings(state)[0]['status'] == 'invalidated'
    assert openings(state)[0]['traversability'] == 'blocked'
    assert openings(state)[0]['contradictions'] == 1


def test_repeated_observation_updates_and_corrects_geometry_without_duplicate():
    state = WorldState()
    state.process(observation(), 0.)
    first = openings(state)[0]
    state.process(observation(.2, rectangles=scene(width=1.0)), .2)
    second = openings(state)[0]
    assert first['entity_id'] == second['entity_id']
    assert second['observations'] == 2
    assert second['width_m'] < first['width_m']
    assert second['source_timestamp'] == 0.
    assert second['last_observed'] == .2


def test_invalidated_opening_can_recover_only_on_new_support():
    state = WorldState()
    for stamp, rectangles in [(0.,scene()), (.2,scene(blocked=True)), (.4,scene())]:
        state.process(observation(stamp, rectangles=rectangles), stamp)
    assert len(openings(state)) == 1
    assert openings(state)[0]['status'] == 'provisional'
    assert openings(state)[0]['contradictions'] == 1


def test_unknown_capture_does_not_invalidate_or_refresh_old_entity():
    state = WorldState()
    state.process(observation(), 0.)
    state.process(observation(.2, depth=np.full((H,W), np.nan)), .2)
    entity = openings(state)[0]
    assert entity['last_observed'] == 0.
    assert entity['status'] == 'provisional'
    assert entity['contradictions'] == 0


@pytest.mark.parametrize('changes,now,reason', [
    ({'pose_valid':False},0.,'invalid_pose'), ({'stamp':-1.,'pose_stamp':-1.},0.,'stale_capture'),
    ({'stamp':1.,'pose_stamp':1.},0.,'future_capture'), ({'pose_stamp':-.1},0.,'unsynchronized_pose'),
    ({'position_sigma_m':1.},0.,'pose_uncertainty'), ({'angle_sigma_rad':float('nan')},0.,'invalid_uncertainty'),
    ({'R_world_body':np.zeros((3,3))},0.,'invalid_rotation'), ({'R_body_cam':None},0.,'invalid_rotation'),
    ({'R_world_body':'bad'},0.,'invalid_rotation'), ({'K':None},0.,'invalid_intrinsics'),
    ({'depth_m':np.full((H,W),'bad')},0.,'invalid_depth_type'),
    ({'t_world_body':np.array([np.nan,0.,0.])},0.,'invalid_translation'),
    ({'K':(0.,100.,80.,50.)},0.,'invalid_intrinsics'), ({'observation_id':''},0.,'invalid_provenance'),
    ({'depth_m':np.zeros((0,0))},0.,'invalid_image_size'), ({'stamp':np.nan},0.,'nonfinite_time')])
def test_bad_inputs_do_not_create_geometry(changes, now, reason):
    state = WorldState()
    report = state.process(replace(observation(), **changes), now)
    assert report['rejection'] == reason
    assert not state.snapshot()['entities']


def test_duplicate_and_reordered_captures_do_not_inflate_confidence():
    state = WorldState()
    state.process(observation(.2), .2)
    expected = state.snapshot(.3)
    assert state.process(observation(.2), .3)['rejection'] == 'duplicate_or_out_of_order'
    assert state.process(observation(.1), .3)['rejection'] == 'duplicate_or_out_of_order'
    assert state.snapshot() == expected
    assert openings(state)[0]['observations'] == 1


@pytest.mark.parametrize('changes', [{'epoch':1}, {'frame_id':'map'}, {'clock_id':'system'}])
def test_reference_changes_latch_until_explicit_reset(changes):
    state = WorldState()
    state.process(observation(), 0.)
    assert state.process(replace(observation(.1), **changes), .1)['rejection'] == 'reference_reset_required'
    assert state.process(observation(.2), .2)['rejection'] == 'reference_reset_required'
    assert openings(state)[0]['status'] == 'uncertain'
    assert state.space_at([2.,0.,1.2], .2) == 'unknown'
    state.reset_reference('odom', 2)
    assert not state.snapshot()['entities']
    assert state.process(observation(0., epoch=2), 0.)['accepted']


def test_pose_jump_and_backward_clock_require_reset():
    state = WorldState()
    state.process(observation(), 0.)
    assert state.process(observation(.1, position=(5.,0.,1.2)), .1)['rejection'] == 'pose_jump_reset_required'
    state.reset_reference('odom', 1)
    state.process(observation(1., epoch=1), 1.)
    assert state.process(observation(.5, epoch=1), .5)['rejection'] == 'reference_reset_required'


def test_forgetting_and_snapshot_detachment():
    state = WorldState()
    state.process(observation(), 0.)
    snap = state.snapshot()
    snap['entities'][0]['geometry']['center'][0] = 1000.
    assert state.snapshot()['entities'][0]['geometry']['center'][0] != 1000.
    state.tick(121.)
    assert not state.snapshot()['entities']
    assert not state._beams


def test_checkpoint_roundtrip_and_continued_association():
    state = WorldState()
    state.process(observation(), 0.)
    restored = WorldState.from_json(state.to_json())
    assert restored.snapshot() == state.snapshot()
    assert restored.space_at([2.,0.,1.2], 0.) == state.space_at([2.,0.,1.2], 0.)
    for owner in (state, restored):
        owner.process(observation(.2, position=(.2,.1,1.2), yaw=.08), .2)
    assert restored.snapshot() == state.snapshot()
    assert restored.to_json() == state.to_json()


def test_schema_versions_and_oversized_checkpoint_rejected():
    state = WorldState()
    state.process(observation(), 0.)
    doc = json.loads(state.to_json())
    doc['schema_version'] = 99
    with pytest.raises(ValueError):
        WorldState.from_json(json.dumps(doc))
    doc['schema_version'] = 1
    doc['config']['max_entities'] = 1
    with pytest.raises(ValueError):
        WorldState.from_json(json.dumps(doc))


def test_deterministic_replay():
    state1, frames1 = replay()
    state2, frames2 = replay()
    assert frames1 == frames2
    assert state1.to_json() == state2.to_json()


@pytest.mark.parametrize('capacity', [1, 3, 8])
def test_memory_bounds_under_repeated_and_distinct_features(capacity):
    config = Config(max_entities=capacity, max_evidence=3, beam_frames=2)
    state = WorldState(config)
    for i in range(25):
        stamp = i * .1
        state.process(observation(stamp, rectangles=scene(width=.6 + (i % 6)*.2)), stamp)
        assert len(state.snapshot()['entities']) <= capacity
        assert len(state._beams) <= 2
        assert all(len(e.evidence) <= 3 for e in state._entities.values())
        assert all(len(r.valid) <= config.max_rays for _,r in state._beams)
    assert len(state.to_json()) < 2500000


def test_rejected_sensor_data_still_ages_memory_on_the_same_clock():
    state = WorldState()
    state.process(observation(), 0.)
    report = state.process(replace(observation(6.), pose_valid=False), 6.)
    assert report['rejection'] == 'invalid_pose'
    assert openings(state)[0]['status'] == 'stale'
    assert state.space_at([2.,0.,1.2], 6.) == 'unknown'
