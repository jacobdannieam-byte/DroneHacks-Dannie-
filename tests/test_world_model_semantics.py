from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from examples.semantic_world_model_demo import observation
from refnav.world_model import Config, VehicleEnvelope, WorldState, entities, openings


def test_query_reports_geometry_provenance_missing_evidence_and_conditional_affordance():
    state = WorldState()
    state.process(observation(), 0.)
    assert entities(state, semantic_type='wall')
    result = openings(state, .2)[0]
    assert result['semantic_type'] == 'candidate_passage'
    assert result['status'] == 'provisional'
    assert result['height_m'] is None
    assert result['connected_regions'] == []
    assert len(result['relations']) == 2
    assert all(r['type'] == 'bounded_by' and r['target'].startswith('wall_') for r in result['relations'])
    assert result['traversability'] == 'unknown'
    assert result['age_s'] == .2
    assert result['evidence'][0]['source'] == 'synthetic_depth'
    assert 'vehicle_propeller_envelope' in result['missing_evidence']
    assert 'upper_boundary' in result['missing_evidence']


def test_width_fit_cannot_certify_a_path():
    state = WorldState()
    state.process(observation(), 0.)
    small = openings(state, envelope=VehicleEnvelope(.3,.2,.03,.03))[0]
    assert small['geometrically_wide_enough'] is True
    assert small['geometrically_tall_enough'] is None
    assert small['traversability'] == 'unknown'
    assert 'validated_swept_path' in small['missing_evidence']
    large = openings(state, envelope=VehicleEnvelope(1.5,.3,.1,.1))[0]
    assert large['geometrically_wide_enough'] is False
    assert large['traversability'] == 'insufficient_clearance'


def test_many_views_never_promote_unbounded_opening_to_confirmed_passage():
    state = WorldState()
    for i in range(6):
        state.process(observation(i*.1), i*.1)
    assert openings(state)[0]['status'] == 'provisional'
    assert openings(state)[0]['traversability'] == 'unknown'
    assert any(e['status'] == 'confirmed' for e in entities(state, semantic_type='wall'))


@pytest.mark.parametrize('kwargs', [{'stride':0}, {'max_entities':1.5}, {'min_coverage':1.1}, {'max_age_s':float('nan')}])
def test_invalid_config_rejected(kwargs):
    with pytest.raises(ValueError):
        Config(**kwargs)


def test_invalid_vehicle_config_and_query_time_rejected():
    with pytest.raises(ValueError):
        VehicleEnvelope(.4,.3,-.1,.1)
    state = WorldState()
    state.process(observation(1.), 1.)
    with pytest.raises(ValueError):
        openings(state, .5)
