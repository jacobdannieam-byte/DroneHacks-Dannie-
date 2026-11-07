"""Independent high-rate RK4 truth generator for the D0 offline experiment."""

from dataclasses import dataclass
from math import pi

import numpy as np


@dataclass(frozen=True)
class SyntheticResponse:
    velocity_tau_s: float = 0.35
    acceleration_scale: float = 1.0
    effective_drag_per_s: float = 0.0
    yaw_delay_steps: int = 0


def simulate(initial_vector, targets, *, dt, response, substeps=10):
    """Integrate [north, east, down, vn, ve, vd, yaw, yaw_rate] continuously.

    Targets are tuples of NED position and yaw. Reference gains are specified
    here as fixture constants. The predictor has no access to this generator.
    """
    state = np.array(initial_vector, dtype=float)
    output = [state.copy()]
    speed_limits = np.array([1.2, 1.2, 0.7])
    acceleration_limits = np.array([2.0, 2.0, 1.2]) * response.acceleration_scale
    step = dt / substeps

    def derivative(vector, position_target, heading_target):
        velocity_demand = np.clip(1.2 * (position_target - vector[:3]), -speed_limits, speed_limits)
        velocity = vector[3:6]
        acceleration = np.clip((velocity_demand - velocity) / response.velocity_tau_s,
                               -acceleration_limits, acceleration_limits)
        # Net perturbation outside the nominal tracking law, in m/s².
        acceleration -= response.effective_drag_per_s * velocity
        heading_error = (heading_target - vector[6] + pi) % (2 * pi) - pi
        rate_demand = max(-1.5, min(1.5, 2.0 * heading_error))
        yaw_acceleration = max(-4.0, min(4.0, (rate_demand - vector[7]) / 0.18))
        return np.array((*velocity, *acceleration, vector[7], yaw_acceleration))

    for index, (position_target, _) in enumerate(targets):
        delayed_index = index - response.yaw_delay_steps
        heading_target = targets[delayed_index][1] if delayed_index >= 0 else initial_vector[6]
        position_target = np.asarray(position_target)
        for _ in range(substeps):
            k1 = derivative(state, position_target, heading_target)
            k2 = derivative(state + step * k1 / 2, position_target, heading_target)
            k3 = derivative(state + step * k2 / 2, position_target, heading_target)
            k4 = derivative(state + step * k3, position_target, heading_target)
            state += step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        output.append(state.copy())
    return np.array(output)
