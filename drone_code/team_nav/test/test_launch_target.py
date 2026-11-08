"""Offline height-contract regressions using the preserved organizer adapter.

These tests send no messages and make no claim about the deployed adapter.
"""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from team_nav.launch_target import position_target_metres

ROOT = Path(__file__).resolve().parents[3]
ADAPTER = ROOT / 'simulator/organizers_code/skybeta_flight_interface/skybeta_flight_interface/frames.py'
spec = importlib.util.spec_from_file_location('recorded_adapter_frames', ADAPTER)
frames = importlib.util.module_from_spec(spec)
spec.loader.exec_module(frames)


@pytest.mark.parametrize('camera_up,px4_down', [
    (0.0, 0.0), (-0.002452939050272107, -0.15490590035915375),
    (2.4, -0.3), (-1.2, 0.7),
])
def test_nonzero_origins_through_unanchored_adapter(camera_up, px4_down):
    camera = np.array([1.2, -0.4, camera_up])
    displacement = np.array([0.5, -0.2, 0.8])
    for _ in range(2):
        target = position_target_metres(camera, px4_down, displacement)
        converted = frames.setpoint_position_enu_to_ned(target)
        assert converted[:2] == pytest.approx((-0.6, 1.7))
        assert px4_down - converted[2] == pytest.approx(0.8)
    assert camera == pytest.approx([1.2, -0.4, camera_up])
    assert displacement == pytest.approx([0.5, -0.2, 0.8])


def test_recorded_old_target_cannot_reach_climb_tolerance():
    camera_up = -0.002452939050272107
    px4_down = -0.15490590035915375
    old_target = (0.0, 0.0, camera_up + 0.8)
    old_down = frames.setpoint_position_enu_to_ned(old_target)[2]
    assert px4_down - old_down == pytest.approx(0.6426411605905741)
    assert abs(0.8 - (px4_down - old_down)) > 0.10


def test_anchored_adapter_would_apply_origin_twice():
    camera = (0.0, 0.0, 2.4)
    px4_origin = (0.0, 0.0, -0.3)
    target = position_target_metres(camera, px4_origin[2], (0.0, 0.0, 0.8))
    converted = frames.setpoint_position_enu_to_ned(target, camera, px4_origin)
    assert px4_origin[2] - converted[2] != pytest.approx(0.8)


@pytest.mark.parametrize('camera,displacement,down', [
    ([0, 0], [0, 0, .8], 0), ([0, 0, 0], [.8], 0),
    ([0, 0, float('nan')], [0, 0, .8], 0),
    ([0, 0, 0], [0, 0, .8], float('inf')),
])
def test_invalid_origins_and_displacements_are_refused(camera, displacement, down):
    with pytest.raises(ValueError):
        position_target_metres(camera, down, displacement)
