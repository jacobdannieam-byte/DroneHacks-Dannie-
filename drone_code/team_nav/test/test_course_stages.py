"""course_stages.py with pretend detector reports: a flat world of wall segments seen from above, a
pretend line detector, pad detector and depth program, and a drone that flies to its setpoint.
Every stage is walked through, and every rule that makes the drone hold or land is checked.

The pretend world, seen from above, in the camera tracking frame (x East, y North, metres):
the line runs East from the start for 2 m, then turns right (South) for 2 m and ends. A barrier
crosses at y = -5 with a 0.8 m wide tunnel entrance at x = 1.6 to 2.4; the tunnel is 2 m long and
has a dark stretch where the depth camera sees nothing. A wall at y = -9 has Gate B in it, and the
pad lies behind that.
"""
import math

import numpy
import pytest

from team_nav.course_stages import COURSE_COMPLETE, STAGES, Course
from team_nav.mission_inputs import Clearances, MissionInputs
from team_nav.mission_settings import MissionSettings

INFINITY = math.inf
STEP_SECONDS = 0.05
CRUISE_HEIGHT_METRES = 0.8
DEPTH_RANGE_METRES = 6.0
SIDE_PROBE_DEGREES = 20.0

BARRIER_Y = -5.0
ENTRANCE_X = (1.6, 2.4)
WORLD_WALLS = [((-5.0, BARRIER_Y), (ENTRANCE_X[0], BARRIER_Y)), ((ENTRANCE_X[1], BARRIER_Y), (9.0, BARRIER_Y)),
               ((ENTRANCE_X[0], BARRIER_Y), (ENTRANCE_X[0], -7.0)), ((ENTRANCE_X[1], BARRIER_Y), (ENTRANCE_X[1], -7.0)),
               ((-5.0, -9.0), (2.3, -9.0)), ((3.9, -9.0), (9.0, -9.0))]      # the chicane wall, with Gate B
DARK_STRETCH_Y = (-6.2, -5.6)
PAD = (3.0, -11.5)                     # the pretend pad detector sees it from under 2.5 m away


def wrap(angle_radians):
    return (angle_radians + math.pi) % (2 * math.pi) - math.pi


def clearances(ahead=INFINITY, left=INFINITY, right=INFINITY, low=INFINITY):
    return Clearances(ahead, left, right, low)


def ray_distance(start, heading_radians, walls, longest=DEPTH_RANGE_METRES):
    """Distance from start along heading to the nearest wall segment; infinity beyond longest."""
    direction_x, direction_y = math.cos(heading_radians), math.sin(heading_radians)
    nearest = INFINITY
    for (a_x, a_y), (b_x, b_y) in walls:
        along_x, along_y = b_x - a_x, b_y - a_y
        denominator = direction_x * along_y - direction_y * along_x
        if abs(denominator) < 1e-12:
            continue
        to_x, to_y = a_x - start[0], a_y - start[1]
        distance = (to_x * along_y - to_y * along_x) / denominator
        fraction = (to_x * direction_y - to_y * direction_x) / denominator
        if distance > 0 and 0 <= fraction <= 1:
            nearest = min(nearest, distance)
    return nearest if nearest <= longest else INFINITY


def ahead_distance(start, heading_radians, walls, half_width_metres=0.3):
    """The way ahead is a corridor: the nearest of three parallel rays across its width."""
    left_x, left_y = -math.sin(heading_radians), math.cos(heading_radians)
    distances = []
    for offset in (-half_width_metres, 0.0, half_width_metres):
        distances.append(ray_distance((start[0] + left_x * offset, start[1] + left_y * offset), heading_radians, walls))
    return min(distances)


def distance_to_walls(point, walls):
    nearest = INFINITY
    for (a_x, a_y), (b_x, b_y) in walls:
        along_x, along_y = b_x - a_x, b_y - a_y
        fraction = ((point[0] - a_x) * along_x + (point[1] - a_y) * along_y) / (along_x ** 2 + along_y ** 2)
        fraction = max(0.0, min(1.0, fraction))
        nearest = min(nearest, math.hypot(point[0] - a_x - fraction * along_x, point[1] - a_y - fraction * along_y))
    return nearest


class Flight:
    """The Course and a drone that flies toward the setpoint at up to 1 m/s and turns at up to
    60 degrees per second. feed(flight, seconds) writes the pretend detector reports."""

    def __init__(self, settings=None, feed=None, heading_radians=0.0):
        self.course = Course(settings or MissionSettings())
        self.inputs = MissionInputs()
        self.position = numpy.array([0.0, 0.0, CRUISE_HEIGHT_METRES])
        self.heading_radians = heading_radians
        self.feed = feed or (lambda flight, seconds: None)
        self.seconds = 0.0
        self.trace = []                           # [(seconds, stage, reason, position, setpoint)]
        self.ended = None                         # (seconds, reason) when the course asks to land
        self.course.start(0.0, (0.0, 0.0, CRUISE_HEIGHT_METRES, heading_radians))

    def step(self):
        self.feed(self, self.seconds)
        decision = self.course.step(self.seconds, STEP_SECONDS, tuple(self.position), self.heading_radians, self.inputs)
        self.trace.append((self.seconds, decision.stage, decision.reason, self.position.copy(), decision.setpoint))
        if decision.land:
            self.ended = (self.seconds, decision.reason)
            return
        self.fly_toward(decision.setpoint)
        self.seconds = round(self.seconds + STEP_SECONDS, 9)

    def fly_toward(self, setpoint):
        offset = numpy.array(setpoint[:3]) - self.position
        distance = numpy.linalg.norm(offset)
        if distance > 1e-9:
            self.position += offset * min(1.0, 1.0 * STEP_SECONDS / distance)
        largest_turn = math.radians(60) * STEP_SECONDS
        self.heading_radians = wrap(self.heading_radians + max(-largest_turn, min(largest_turn,
                                                                                 wrap(setpoint[3] - self.heading_radians))))

    def run(self, seconds, until=None):
        end = self.seconds + seconds
        while self.seconds < end and self.ended is None and not (until and until(self)):
            self.step()
        return self

    def stages(self):
        visited = []
        for _, stage, *_rest in self.trace:
            if not visited or visited[-1] != stage:
                visited.append(stage)
        return visited

    def reasons(self):
        return [reason for _, _, reason, _, _ in self.trace]

    def first_time(self, reason):
        return next(seconds for seconds, _, said, _, _ in self.trace if said == reason)

    def held_positions_after(self, reason, settle_seconds=0.3):
        start = self.first_time(reason)
        return numpy.array([position[:2] for seconds, _, said, position, _ in self.trace
                            if said == reason and seconds > start + settle_seconds])


# ---------------------------------------------------------------- pretend detectors
def healthy_tracking_and_laser(flight, seconds):
    flight.inputs.set_vision(seconds, True)
    flight.inputs.set_height(seconds, CRUISE_HEIGHT_METRES)


def line_detector(flight, seconds):
    x, y, _ = flight.position
    heading = flight.heading_radians
    if x < 1.7:                                   # the first leg runs East along y = 0
        offset_left = -y * math.cos(heading)
        flight.inputs.set_line(seconds, True, offset_left, wrap(0.0 - heading), end_seen=False)
    else:                                         # then South along x = 2, ending at y = -2
        offset_left = (2.0 - x) * -math.sin(heading)
        flight.inputs.set_line(seconds, True, offset_left, wrap(-math.pi / 2 - heading), end_seen=y <= -2.0)


def depth_program(flight, seconds):
    x, y, _ = flight.position
    heading = flight.heading_radians
    ahead = ahead_distance((x, y), heading, WORLD_WALLS)
    if DARK_STRETCH_Y[0] <= y <= DARK_STRETCH_Y[1] and ENTRANCE_X[0] < x < ENTRANCE_X[1]:
        ahead = None
    side = math.radians(SIDE_PROBE_DEGREES)
    flight.inputs.set_clearances(seconds, clearances(ahead, ray_distance((x, y), heading + side, WORLD_WALLS),
                                                     ray_distance((x, y), heading - side, WORLD_WALLS)))


def pad_detector(flight, seconds):
    x, y, _ = flight.position
    east, north = PAD[0] - x, PAD[1] - y
    bearing_left = wrap(math.atan2(north, east) - flight.heading_radians)
    distance = math.hypot(east, north)
    in_view = abs(bearing_left) < math.radians(35) and 0.8 < distance < 2.5
    unblocked = ray_distance((x, y), math.atan2(north, east), WORLD_WALLS) > distance
    if in_view and unblocked:
        flight.inputs.set_pad(seconds, True, (distance * math.cos(bearing_left), distance * math.sin(bearing_left)))
    else:
        flight.inputs.set_pad(seconds, False)


def whole_course(flight, seconds):
    healthy_tracking_and_laser(flight, seconds)
    line_detector(flight, seconds)
    depth_program(flight, seconds)
    pad_detector(flight, seconds)


def straight_line_in_open_space(end_at_x=None, valid=lambda seconds: True):
    """A line straight East from the start, everything clear; it ends at x = end_at_x if given."""
    def feed(flight, seconds):
        healthy_tracking_and_laser(flight, seconds)
        x, y, _ = flight.position
        end_seen = end_at_x is not None and x >= end_at_x
        flight.inputs.set_line(seconds, valid(seconds), -y * math.cos(flight.heading_radians),
                               wrap(-flight.heading_radians), end_seen=end_seen)
        flight.inputs.set_clearances(seconds, clearances())
    return feed


TUNNEL_ON = MissionSettings(tunnel_enabled=True)


# ---------------------------------------------------------------- walking the stages
def test_walks_all_seven_stages_with_the_tunnel_on_and_ends_over_the_pad():
    flight = Flight(TUNNEL_ON, whole_course).run(150)
    assert flight.stages() == list(STAGES), (flight.stages(), flight.ended)
    assert flight.ended[1] == COURSE_COMPLETE
    assert math.hypot(flight.position[0] - PAD[0], flight.position[1] - PAD[1]) < 0.3
    assert all(flight.reasons())                                          # every decision has a reason
    said = set(zip([stage for _, stage, *_r in flight.trace], flight.reasons()))
    for expected in (("follow_line", "turning onto the line"), ("tunnel_cross", "depth back after the dark stretch"),
                     ("chicane", "gate B: blocked ahead, turning left"), ("chicane", "pad in view")):
        assert expected in said, expected
    crossed_dark_stretch = [position for _, _, reason, position, _ in flight.trace
                            if reason == "crossing the tunnel" and DARK_STRETCH_Y[0] <= position[1] <= DARK_STRETCH_Y[1]]
    assert crossed_dark_stretch                                           # on space it had seen free
    assert min(distance_to_walls(position, WORLD_WALLS) for _, _, _, position, _ in flight.trace) > 0.3


def test_by_default_it_hovers_at_the_tunnel_entrance_then_lands():
    flight = Flight(None, whole_course).run(150)
    assert flight.stages() == ["follow_line", "gate_a", "tunnel_align"]
    assert flight.ended[1] == "tunnel disabled: landing at the tunnel entrance"
    hover = flight.held_positions_after("tunnel disabled: hovering at the tunnel entrance", settle_seconds=0.0)
    assert len(hover) * STEP_SECONDS >= MissionSettings().hover_at_tunnel_entrance_seconds - 0.1
    assert numpy.ptp(hover, axis=0).max() < 0.15                          # it held still
    assert flight.position[1] > BARRIER_Y + 0.9                           # never went in


def test_the_chicane_turns_toward_the_open_side_and_window_says_it_uses_gate_b():
    cases = [(MissionSettings(), "gate B: "), (MissionSettings(window_enabled=True), "no window detector yet, gate B: ")]
    for settings, prefix in cases:
        for left, right, word, sign in ((INFINITY, 1.2, "left", 1), (1.2, INFINITY, "right", -1)):
            def feed(flight, seconds, left=left, right=right):
                healthy_tracking_and_laser(flight, seconds)
                flight.inputs.set_clearances(seconds, clearances(1.0, left, right))
            flight = Flight(settings, feed)
            flight.course.enter_stage("chicane", 0.0)
            flight.run(1.0)
            assert flight.reasons()[-1] == prefix + f"blocked ahead, turning {word}"
            assert sign * flight.heading_radians > math.radians(20)        # turned that way
            assert numpy.linalg.norm(flight.position[:2]) < 1e-9           # without moving


# ---------------------------------------------------------------- holds and landings
def test_line_lost_holds_then_lands():
    flight = Flight(None, straight_line_in_open_space(valid=lambda seconds: seconds < 4.0)).run(30)
    lost_at = flight.first_time("line lost, holding")
    assert lost_at == pytest.approx(4.0, abs=0.06)
    assert numpy.ptp(flight.held_positions_after("line lost, holding"), axis=0).max() < 1e-6
    assert flight.ended == (pytest.approx(lost_at + 5.0, abs=0.06), "line lost for 5 s, landing")


def test_line_back_within_the_hold_resumes():
    flight = Flight(None, straight_line_in_open_space(valid=lambda seconds: not 4.0 <= seconds < 6.0)).run(12)
    assert flight.ended is None and flight.reasons()[-1] == "following the line"
    assert flight.position[0] > 4.0


def test_a_stage_past_its_time_limit_hovers_then_lands():
    flight = Flight(MissionSettings(follow_line_time_limit_seconds=10.0), straight_line_in_open_space()).run(30)
    limit_at = flight.first_time("follow_line ran out of time after 10 s, hovering")
    assert limit_at == pytest.approx(10.0, abs=0.06)
    assert numpy.ptp(flight.held_positions_after("follow_line ran out of time, hovering"), axis=0).max() < 1e-6
    assert flight.ended == (pytest.approx(limit_at + 5.0, abs=0.06), "follow_line ran out of time; hovered 5 s, landing")


def test_the_finishing_test_still_counts_while_hovering_after_the_time_limit():
    flight = Flight(MissionSettings(follow_line_time_limit_seconds=10.0), straight_line_in_open_space(end_at_x=0.0))
    flight.course.line_end_sightings = -10 ** 6                           # not believed before the time limit
    flight.run(10.5)
    flight.course.line_end_sightings = 0
    flight.run(5, until=lambda f: f.course.stage == "gate_a")
    assert flight.course.stage == "gate_a"
    assert "line end seen (while hovering after the time limit)" in flight.reasons()


def test_camera_tracking_unsure_holds_and_stops_advancing_then_resumes():
    def feed(flight, seconds):
        straight_line_in_open_space()(flight, seconds)
        flight.inputs.set_vision(seconds, not 3.0 <= seconds < 5.0)
    flight = Flight(None, feed).run(8)
    during = [setpoint for _, _, reason, _, setpoint in flight.trace if reason == "camera tracking lost, holding"]
    assert len(during) > 30 and len(set(during[1:])) == 1                 # the setpoint, heading too, froze
    assert flight.ended is None and flight.reasons()[-1] == "following the line"


def test_camera_tracking_unsure_for_too_long_lands():
    def feed(flight, seconds):
        straight_line_in_open_space()(flight, seconds)
        flight.inputs.set_vision(seconds, seconds < 3.0)
    flight = Flight(None, feed).run(20)
    assert flight.ended[1] == "camera tracking lost for 5 s, landing"
    assert flight.ended[0] - flight.first_time("camera tracking lost, holding") == pytest.approx(5.0, abs=0.06)


SPOILED_INPUTS = {
    "line stale": (lambda inputs, s: inputs.set_line(s - 1.0, True, 0.0, 0.0), "line stale, holding"),
    "line from the future": (lambda inputs, s: inputs.set_line(s + 1.0, True, 0.0, 0.0), "line from the future, holding"),
    "line missing": (lambda inputs, s: inputs.set_line(None, True, 0.0, 0.0), "line missing, holding"),
    "line offset not a number": (lambda inputs, s: inputs.set_line(s, True, math.nan, 0.0), "line lost, holding"),
    "clearances stale": (lambda inputs, s: inputs.set_clearances(s - 1.0, clearances()), "depth clearances stale, holding"),
    "clearances missing": (lambda inputs, s: inputs.set_clearances(None, None), "depth clearances missing, holding"),
    "way ahead unknown": (lambda inputs, s: inputs.set_clearances(s, clearances(ahead=None)), "way ahead unknown, holding"),
    "obstacle ahead": (lambda inputs, s: inputs.set_clearances(s, clearances(ahead=0.8)), "obstacle ahead, holding"),
    "vision stale": (lambda inputs, s: inputs.set_vision(s - 1.0, True), "camera tracking stale, holding"),
    "vision missing": (lambda inputs, s: inputs.set_vision(None, True), "camera tracking missing, holding"),
}


@pytest.mark.parametrize("case", list(SPOILED_INPUTS))
def test_a_stale_missing_or_blocking_input_never_moves_the_drone_and_lands(case):
    spoil, reason = SPOILED_INPUTS[case]

    def feed(flight, seconds):
        straight_line_in_open_space()(flight, seconds)
        spoil(flight.inputs, seconds)
    flight = Flight(None, feed).run(20)
    assert flight.reasons()[0] == reason
    assert numpy.linalg.norm(flight.position[:2]) < 1e-9
    assert flight.ended[1] == reason.replace(", holding", " for 5 s, landing")


def test_the_default_inputs_are_all_invalid():
    flight = Flight(None, None).run(20)
    assert flight.reasons()[0] == "camera tracking missing, holding"
    assert flight.ended[1] == "camera tracking missing for 5 s, landing"
    assert numpy.linalg.norm(flight.position[:2]) < 1e-9


def test_the_carrot_stops_short_of_an_obstacle_it_sees():
    wall = [((3.0, -5.0), (3.0, 5.0))]

    def feed(flight, seconds):
        straight_line_in_open_space()(flight, seconds)
        x, y, _ = flight.position
        flight.inputs.set_clearances(seconds, clearances(ray_distance((x, y), flight.heading_radians, wall)))
    flight = Flight(None, feed).run(30)
    furthest = max(position[0] for _, _, _, position, _ in flight.trace)
    assert furthest <= 3.0 - MissionSettings().stopping_distance_metres + 0.05
    assert flight.ended[1] == "obstacle ahead for 5 s, landing"


class StuckFlight(Flight):
    """A drone that does not follow its setpoint at all, so the carrot runs as far ahead as allowed."""

    def fly_toward(self, setpoint):
        pass


def test_the_carrot_is_never_more_than_half_a_metre_from_the_drone():
    def line_far_to_the_left(flight, seconds):
        healthy_tracking_and_laser(flight, seconds)
        flight.inputs.set_line(seconds, True, 2.0, 0.0)              # wants forward and sideways at once
        flight.inputs.set_clearances(seconds, clearances())
    flight = StuckFlight(None, line_far_to_the_left).run(4)
    distances = [math.hypot(setpoint[0] - position[0], setpoint[1] - position[1])
                 for _, _, _, position, setpoint in flight.trace]
    assert max(distances) <= MissionSettings().carrot_lead_metres + 1e-9
    assert max(distances) > 0.45


def test_drop_world_goals_forgets_the_pad():
    flight = Flight(TUNNEL_ON, whole_course)
    flight.run(150, until=lambda f: f.course.stage == "find_pad")
    assert flight.course.pad_position is not None
    flight.course.drop_world_goals()
    assert flight.course.pad_position is None and flight.course.free_space_seen_until is None
