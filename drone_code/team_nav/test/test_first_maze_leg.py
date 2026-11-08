"""Run each first-leg movement through the real control loop with a pretend clock.

No ROS messages are sent. These checks catch callback failures and verify the requested
positions and turn rate; Gazebo must still check the aircraft's physical response.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from team_nav.practice_flights.first_maze_leg import FirstMazeLegFlight
from test_flight_steps import PretendLink


@pytest.mark.parametrize("method,start,end,heading_start,heading_end", [
    ("fly_corridor_1", [-2.51, 0.91, 0.75], [-2.51, 2.18, 0.75], 90, 90),
    ("turn_90_degrees_east", [-2.51, 2.18, 0.75], [-2.51, 2.18, 0.75], 90, 0),
    ("enter_maze", [-2.51, 2.18, 0.75], [-0.50, 2.18, 0.75], 0, 0),
])
def test_movement_callbacks_run_with_the_shared_loop(method, start, end, heading_start, heading_end):
    link = PretendLink(height_metres=0.75)
    # A nonzero origin catches callbacks that accidentally use absolute uptime as progress.
    link.seconds = 100.0
    sent = []
    checks = []

    def send(position, heading_on_course_radians):
        sent.append((link.seconds, np.array(position), math.degrees(heading_on_course_radians)))

    def check():
        checks.append(link.seconds)
        return False

    flight = object.__new__(FirstMazeLegFlight)
    flight.link = link
    flight.log = link.log
    flight.watch = SimpleNamespace(check=check, send_target_on_course=send)
    getattr(flight, method)()

    assert len(sent) > 20
    assert len(checks) == len(sent)
    np.testing.assert_allclose(sent[0][1], start)
    np.testing.assert_allclose(sent[-1][1], end, atol=0.031)
    assert sent[0][2] == pytest.approx(heading_start)
    assert sent[-1][2] == pytest.approx(heading_end)
    for before, after in zip(sent, sent[1:]):
        elapsed = after[0] - before[0]
        assert np.linalg.norm(after[1] - before[1]) <= 0.30 * elapsed + 1e-9
        assert abs(after[2] - before[2]) <= 15.0 * elapsed + 1e-9
