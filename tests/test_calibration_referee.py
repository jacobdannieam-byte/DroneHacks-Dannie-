from pathlib import Path
import pytest

from simulator.gazebo_trial.calibration_referee import (
    LANE_RELATIVE_PATH, ROOT, calibration_observer_enabled, geometry_sample,
)


def test_stock_referee_remains_default_even_for_lane():
    assert not calibration_observer_enabled({'SIMULATOR_WORLD_MAP': str(ROOT / LANE_RELATIVE_PATH)})
    assert calibration_observer_enabled({'CALIBRATION_LANE_RUN': '1',
                                         'SIMULATOR_WORLD_MAP': str(ROOT / LANE_RELATIVE_PATH)})


def test_calibration_requires_exact_path_and_bytes(tmp_path):
    source = ROOT / LANE_RELATIVE_PATH
    duplicate = tmp_path / 'duplicate.json'
    duplicate.write_bytes(source.read_bytes())
    with pytest.raises(ValueError):
        calibration_observer_enabled({'CALIBRATION_LANE_RUN': '1', 'SIMULATOR_WORLD_MAP': str(duplicate)})
    expected = tmp_path / LANE_RELATIVE_PATH
    expected.parent.mkdir(parents=True)
    expected.write_bytes(source.read_bytes() + b'\n')
    with pytest.raises(ValueError):
        calibration_observer_enabled({'CALIBRATION_LANE_RUN': '1', 'SIMULATOR_WORLD_MAP': str(expected)}, tmp_path)
    with pytest.raises(ValueError):
        calibration_observer_enabled({'CALIBRATION_LANE_RUN': 'yes'})


def test_ground_contact_is_not_reported_as_lateral_collision_or_course_score():
    sample = geometry_sample((0., 0., 0.), (0., 0., 0.))
    assert sample['floor_envelope_overlap']
    assert not sample['side_or_roof_envelope_intrusion']
    assert not sample['center_below_modeled_floor']
    assert not sample['navigation_center_box']
    assert 'course_score' not in sample


def test_native_ned_geometry_captures_speed_and_outside_bounds():
    sample = geometry_sample((1.5, .1, -1.), (.2, 0., 0.))
    assert sample['position_world_enu_m'] == (.1, 1.5, 1.)
    assert sample['navigation_center_box'] and sample['speed_3d_mps'] == .2
    assert not sample['side_or_roof_envelope_intrusion']
    assert geometry_sample((5.8, 0., -1.), (0., 0., 0.))['side_or_roof_envelope_intrusion']
    assert geometry_sample((0., 0., -2.9), (0., 0., -.6))['side_or_roof_envelope_intrusion']
    with pytest.raises(ValueError):
        geometry_sample((0., float('nan'), 0.), (0., 0., 0.))
