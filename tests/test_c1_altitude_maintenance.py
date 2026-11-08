from examples.calibration_trial_probe import Fixture
from refnav.core import NAV_OFFBOARD
from examples.c1_altitude_rehearsal import simulate
import refnav.calibration_trial as implementation
import json
from pathlib import Path
import pytest

ACTIVE = dict(nav=NAV_OFFBOARD, armed=True, landed=False)


def climbing():
    fixture = Fixture()
    for _ in range(11):
        fixture.step()
    fixture.step(nav=NAV_OFFBOARD)
    fixture.step(nav=NAV_OFFBOARD, armed=True)
    return fixture


def test_lower_navigation_boundary_is_not_a_hover_target():
    fixture = climbing()
    for _ in range(10):
        out = fixture.step(position=(0., 0., -.81), **ACTIVE)
        assert fixture.trial.phase == 'climb'
        assert out.velocity_ned_mps == pytest.approx((0., 0., -.19))
    for _ in range(6):
        out = fixture.step(position=(0., 0., -.98), **ACTIVE)
    assert fixture.trial.phase == 'transit'
    assert out.velocity_ned_mps == pytest.approx((.2, 0., -.02))


@pytest.mark.parametrize('height,vertical', [(.81, -.19), (.95, -.05), (1., 0.), (1.05, .05), (1.19, .19)])
def test_transit_corrects_height_in_both_directions_without_changing_horizontal_command(height, vertical):
    fixture = Fixture()
    fixture.start_transit()
    out = fixture.step(position=(.5, 0., -height), velocity=(.2, 0., 0.), **ACTIVE)
    assert out.velocity_ned_mps == pytest.approx((.2, 0., vertical))
    assert abs(out.velocity_ned_mps[2]) <= .2


@pytest.mark.parametrize('height', [.799, 1.201])
def test_navigation_band_violation_still_lands_immediately(height):
    fixture = Fixture()
    fixture.start_transit()
    out = fixture.step(position=(.9, 0., -height), **ACTIVE)
    assert out.command == 'land' and out.velocity_ned_mps is None
    assert fixture.trial.recovery_reason == 'left_navigation_height_band'


def test_altitude_feedback_does_not_contaminate_three_axis_braking_measurement():
    fixture = Fixture()
    fixture.start_transit()
    out = fixture.step(position=(1.5, 0., -.98), velocity=(.2, 0., -.02), **ACTIVE)
    assert out.event == 'brake_command' and out.velocity_ned_mps == (0., 0., 0.)
    out = fixture.step(position=(1.55, 0., -.98), velocity=(.1, 0., 0.), **ACTIVE)
    assert out.velocity_ned_mps == (0., 0., 0.)


def test_settling_restarts_if_height_leaves_centering_window():
    fixture = climbing()
    for _ in range(4):
        fixture.step(position=(0., 0., -.99), **ACTIVE)
    fixture.step(position=(0., 0., -.94), **ACTIVE)
    for _ in range(4):
        fixture.step(position=(0., 0., -.99), **ACTIVE)
        assert fixture.trial.phase == 'climb'


@pytest.mark.parametrize('drift', [-.02, 0., .01, .02])
@pytest.mark.parametrize('tau', [.20, .35, .60])
def test_closed_loop_synthetic_bias_reaches_brake_and_normal_landing(drift, tau):
    profile = json.loads((Path(__file__).resolve().parents[1] / 'scenarios/calibration/c1_stop_020.json').read_text())
    result = simulate(implementation, profile, drift_mps=drift, tau_s=tau)
    assert result['result'] == 'complete'
    assert [e['event'] for e in result['events']] == ['brake_command', 'land_command']
    assert result['max_observed_speed_mps'] <= .5
    for row in result['rows']:
        if row['phase'] == 'transit':
            assert .8 <= -row['position_ned_m'][2] <= 1.2
            assert row['command_velocity_mps'][:2] == [.2, 0.]
            assert abs(row['command_velocity_mps'][2]) <= .2
        if row['phase'] == 'brake':
            assert row['command_velocity_mps'] == [0., 0., 0.]
