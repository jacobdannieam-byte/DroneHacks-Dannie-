from dataclasses import FrozenInstanceError, replace
from itertools import repeat
from math import inf, nan, pi

import numpy as np
import pytest

from refnav.dynamics import DroneState, LocalNED, NominalParameters, PositionSetpoint


REFERENCE = LocalNED("local_ned", 0, "replay")


def test_arrays_are_normalized_to_immutable_si_values():
    position = np.array([1.0, -2.0, 3.0], dtype=np.float32)
    state = DroneState(REFERENCE, np.float64(0.5), position, [0, 0, 0], 3 * pi, 0)
    position[0] = 50
    assert state.position_m == (1.0, -2.0, 3.0)
    assert state.yaw_rad == -pi
    with pytest.raises(FrozenInstanceError):
        state.timestamp_s = 1.0


@pytest.mark.parametrize("field, value", [
    ("timestamp_s", nan), ("position_m", (inf, 0, 0)),
    ("velocity_mps", (0, nan, 0)), ("yaw_rad", inf), ("yaw_rate_rad_s", nan),
    ("reference", "enu"),
])
def test_invalid_state_is_rejected(field, value):
    state = DroneState(REFERENCE, 0, (0, 0, 0), (0, 0, 0), 0, 0)
    with pytest.raises(ValueError):
        replace(state, **{field: value})


@pytest.mark.parametrize("position", [(), (1, 2), (1, 2, 3, 4), [[1], [2], [3]],
                                      "123", {1, 2, 3}, (True, 2, 3), (1, nan, 3)])
def test_invalid_command_shapes_and_values(position):
    with pytest.raises(ValueError):
        PositionSetpoint(REFERENCE, 0, 1, position, 0)


def test_command_vector_consumes_only_four_items_on_oversized_input():
    values = repeat(0.0)
    with pytest.raises(ValueError, match="exactly three"):
        PositionSetpoint(REFERENCE, 0, 1, values, 0)


@pytest.mark.parametrize("start, end", [(0, 0), (1, 0), (nan, 1), (0, inf)])
def test_effective_interval_is_finite_and_ordered(start, end):
    with pytest.raises(ValueError):
        PositionSetpoint(REFERENCE, start, end, (0, 0, 0), 0)


@pytest.mark.parametrize("kwargs", [{"reference_epoch": -1}, {"reference_epoch": True},
                                     {"frame_id": ""}, {"clock_id": " "}])
def test_reference_has_clock_frame_and_reset_epoch(kwargs):
    with pytest.raises(ValueError):
        replace(REFERENCE, **kwargs)


@pytest.mark.parametrize("kwargs", [{"velocity_tau_s": 0}, {"yaw_gain_per_s": nan},
                                     {"speed_limits_mps": (1, 2, 0)},
                                     {"acceleration_limits_mps2": (1, -1, 1)}])
def test_parameters_are_finite_positive_si_values(kwargs):
    with pytest.raises(ValueError):
        NominalParameters(**kwargs)
