"""Tests flight_steps.py with a pretend drone: the touchdown rules, the landing sequence, and
holding still when the safety checks say so. No ROS messages are sent.

The pretend drone is a point that moves up and down at the commanded speed, one tenth of a second
per cycle, and stops at the floor. PX4 "switches its motors off" a set time after the push-down
starts, or a longer time after the drone reaches the floor without one. It is far simpler than the real drone; the simulator flights test the real behaviour.

How to run these tests is in the package's README.
"""
import pytest

from team_nav import flight_steps
from team_nav.flight_steps import (
    GENTLE_DESCENT_BEFORE_A_STALL_COUNTS_SECONDS, GENTLE_DESCENT_METRES_PER_SECOND, LASER_RAW_READING,
    MEDIUM_DESCENT_METRES_PER_SECOND, NO_HEIGHT_READING, PUSH_DOWN_METRES_PER_SECOND, PX4_DISTANCE_TO_GROUND,
    STALLED_DESCENT_SECONDS, ControlIsNoLongerOurs, RunMustStop, TouchdownDetector, choose_landing_height_source,
)

# Made up for the pretend drone, roughly as in the simulator (section 7).
SECONDS_FROM_PUSH_DOWN_TO_MOTORS_OFF = 3.0
SECONDS_ON_THE_FLOOR_WITHOUT_PUSH_DOWN_TO_MOTORS_OFF = 7.0


class PretendLog:
    def __init__(self):
        self.events = []

    def event(self, kind, text, **details):
        self.events.append((kind, text))

    def sample(self, table_name, **values):
        pass


class PretendTargets:
    def __init__(self):
        self.sent = []          # ("speed", downward m/s) or ("resend", None) or ("position", value)

    def send_downward_speed(self, metres_per_second):
        self.sent.append(("speed", metres_per_second))

    def resend_last(self):
        self.sent.append(("resend", None))


class PretendReports:
    """The organizers' summary, which the landing only quotes in its reminders."""

    def state(self):
        return {"armed": True, "nav_state_name": "OFFBOARD", "landed": False}


class PretendLink:
    """Just enough of DroneLink for the landing and fly_until."""

    def __init__(self, height_metres, laser_works=True):
        self.log = PretendLog()
        self.reports = PretendReports()
        self.targets = PretendTargets()
        self.stage = "test"
        self.seconds = 0.0
        self.height_metres = height_metres
        self.laser_works = laser_works
        self.downward_speed = 0.0
        self.armed = True
        self.offboard = True
        self.push_down_began_seconds = None
        self.reached_floor_seconds = None
        self.raw_laser_available = laser_works
        self.px4_distance_usable = True
        # How far PX4's distance to the ground reads above the truth, as when PX4 has stopped
        # using the laser.
        self.px4_height_error_metres = 0.0

    def now_seconds(self):
        return self.seconds

    def wait_one_cycle(self):
        commanded = self.targets.sent[-1][1] if self.targets.sent and self.targets.sent[-1][0] == "speed" else 0.0
        if commanded == PUSH_DOWN_METRES_PER_SECOND and self.push_down_began_seconds is None:
            self.push_down_began_seconds = self.seconds
        new_height = max(0.0, self.height_metres - commanded * flight_steps.CYCLE_SECONDS)
        self.downward_speed = (self.height_metres - new_height) / flight_steps.CYCLE_SECONDS
        self.height_metres = new_height
        self.seconds += flight_steps.CYCLE_SECONDS
        if new_height == 0.0 and self.reached_floor_seconds is None:
            self.reached_floor_seconds = self.seconds
        if self.push_down_began_seconds is not None and self.seconds - self.push_down_began_seconds >= SECONDS_FROM_PUSH_DOWN_TO_MOTORS_OFF:
            self.armed = False
        if self.reached_floor_seconds is not None and self.seconds - self.reached_floor_seconds >= SECONDS_ON_THE_FLOOR_WITHOUT_PUSH_DOWN_TO_MOTORS_OFF:
            self.armed = False

    def px4_distance_to_ground_is_usable(self):
        return self.px4_distance_usable

    def px4_height_above_floor_metres(self):
        return self.height_metres + self.px4_height_error_metres

    def laser_height_risen_metres(self):
        return self.height_metres if self.laser_works else None

    def downward_speed_metres_per_second(self):
        return self.downward_speed

    def is_armed(self):
        return self.armed

    def control_is_ours(self):
        return self.offboard and self.armed

    def is_landed(self):
        return self.height_metres == 0.0

    def record_drone_condition(self):
        pass


def speeds_sent(link):
    speeds = []
    for kind, value in link.targets.sent:
        if kind == "speed" and (not speeds or speeds[-1] != value):
            speeds.append(value)
    return speeds


# ---- Touchdown ---------------------------------------------------------------------------------

def test_touchdown_needs_the_floor_and_no_descent_for_a_moment():
    detector = TouchdownDetector(start_seconds=0.0)
    assert detector.check(1.0, 0.01, 0.0) is None
    assert detector.check(1.2, 0.01, 0.0) is None
    assert "back to the launch mark" in detector.check(1.3, 0.01, 0.0)


def test_one_still_moment_away_from_the_floor_is_not_touchdown():
    detector = TouchdownDetector(start_seconds=0.0)
    assert detector.check(1.0, 0.20, 0.0) is None
    assert detector.check(1.5, 0.20, 0.3) is None
    assert detector.check(1.9, 0.01, 0.3) is None   # Still moving down.


def test_a_stalled_descent_counts_as_touchdown_only_after_the_descent_has_had_time_to_start():
    detector = TouchdownDetector(start_seconds=0.0)
    for tenth in range(int(GENTLE_DESCENT_BEFORE_A_STALL_COUNTS_SECONDS * 10)):
        assert detector.check(tenth / 10.0, None, 0.0) is None
    stall_time = GENTLE_DESCENT_BEFORE_A_STALL_COUNTS_SECONDS + STALLED_DESCENT_SECONDS
    assert "stopped moving down" in detector.check(stall_time, None, 0.0)


# ---- The landing --------------------------------------------------------------------------------

def test_the_landing_goes_medium_then_gentle_then_pushes_down_until_the_motors_are_off():
    link = PretendLink(height_metres=0.8)
    flight_steps.land(link)

    assert speeds_sent(link) == [MEDIUM_DESCENT_METRES_PER_SECOND, GENTLE_DESCENT_METRES_PER_SECOND, PUSH_DOWN_METRES_PER_SECOND]
    assert not link.armed
    kinds = [kind for kind, _ in link.log.events]
    assert "touchdown" in kinds and kinds[-1] == "motors_off"


def test_a_landing_whose_height_reading_stops_descends_gently_and_still_finishes():
    link = PretendLink(height_metres=0.8, laser_works=False)
    flight_steps.land(link, LASER_RAW_READING)

    assert speeds_sent(link) == [GENTLE_DESCENT_METRES_PER_SECOND, PUSH_DOWN_METRES_PER_SECOND]
    assert not link.armed


def test_a_landing_with_no_height_reading_never_pushes_down():
    link = PretendLink(height_metres=0.8, laser_works=False)
    flight_steps.land(link, NO_HEIGHT_READING)

    assert speeds_sent(link) == [GENTLE_DESCENT_METRES_PER_SECOND]
    assert not link.armed


@pytest.mark.parametrize('height_source', [LASER_RAW_READING, NO_HEIGHT_READING])
def test_landing_speed_limit_keeps_airborne_descent_slow(height_source):
    link = PretendLink(height_metres=0.8)
    flight_steps.land(link, height_source, descent_speed_limit=0.15)
    expected = [0.15] if height_source == NO_HEIGHT_READING else [0.15, PUSH_DOWN_METRES_PER_SECOND]
    assert speeds_sent(link) == expected
    assert not link.armed


@pytest.mark.parametrize('limit', [0, -1, float('nan')])
def test_invalid_landing_speed_limit_sends_nothing(limit):
    link = PretendLink(height_metres=0.8)
    with pytest.raises(ValueError, match='positive finite'):
        flight_steps.land(link, descent_speed_limit=limit)
    assert not link.targets.sent


@pytest.mark.parametrize("in_doubt, raw_laser, px4_usable, expected", [
    (False, True, True, PX4_DISTANCE_TO_GROUND),
    (True, True, True, LASER_RAW_READING),
    (True, False, True, PX4_DISTANCE_TO_GROUND),     # The real drone has no raw laser reading.
    (True, False, False, NO_HEIGHT_READING),
    (False, True, False, NO_HEIGHT_READING),
])
def test_the_landing_finds_the_floor_with_the_best_reading_it_has(in_doubt, raw_laser, px4_usable, expected):
    link = PretendLink(height_metres=0.8, laser_works=raw_laser)
    link.px4_distance_usable = px4_usable
    assert choose_landing_height_source(link, in_doubt) == expected


def test_losing_control_during_the_descent_stops_the_landing():
    link = PretendLink(height_metres=0.8)
    link.offboard = False
    with pytest.raises(ControlIsNoLongerOurs, match="Offboard"):
        flight_steps.land(link)


def test_the_kill_switch_before_touchdown_stops_the_landing():
    link = PretendLink(height_metres=0.8)
    link.armed = False
    with pytest.raises(ControlIsNoLongerOurs, match="switched off"):
        flight_steps.land(link)


# ---- The safety checks during the descent -----------------------------------------------------

class WatchThatFindsPx4sHeightWrong:
    """Stands in for SafetyWatch during a landing. Its checks find PX4's height wrong from the
    given cycle on, and it notes the drone's height at every check."""

    def __init__(self, link, wrong_from_check_number):
        self.link = link
        self.wrong_from_check_number = wrong_from_check_number
        self.heights_when_checked = []
        self.checks_while_pushing_down = 0

    def check_during_landing(self):
        self.heights_when_checked.append(self.link.height_metres)
        if self.link.push_down_began_seconds is not None:
            self.checks_while_pushing_down += 1
        return len(self.heights_when_checked) >= self.wrong_from_check_number


def landing_events(link):
    return [text for kind, text in link.log.events if kind in ("landing", "touchdown")]


def test_a_height_problem_during_the_descent_switches_the_landing_to_the_raw_laser():
    link = PretendLink(height_metres=0.8)
    link.px4_height_error_metres = 0.40     # PX4 thinks the drone is 40 cm higher than it is.
    watch = WatchThatFindsPx4sHeightWrong(link, wrong_from_check_number=3)
    flight_steps.land(link, PX4_DISTANCE_TO_GROUND, watch)

    events = landing_events(link)
    assert any("raw reading" in text for text in events)
    # With the raw laser, touchdown is found the normal way, not by the drone stalling on the floor.
    assert any("back to the launch mark" in text for text in events)
    assert speeds_sent(link)[-1] == PUSH_DOWN_METRES_PER_SECOND and not link.armed


def test_without_a_raw_laser_a_height_problem_during_the_descent_changes_nothing():
    link = PretendLink(height_metres=0.8, laser_works=False)
    watch = WatchThatFindsPx4sHeightWrong(link, wrong_from_check_number=1)
    flight_steps.land(link, PX4_DISTANCE_TO_GROUND, watch)

    assert not any("raw reading" in text for text in landing_events(link))
    assert not link.armed


def test_the_checks_run_through_the_descent_and_stop_at_touchdown():
    link = PretendLink(height_metres=0.8)
    watch = WatchThatFindsPx4sHeightWrong(link, wrong_from_check_number=1000)
    flight_steps.land(link, PX4_DISTANCE_TO_GROUND, watch)

    assert watch.heights_when_checked[0] == pytest.approx(0.8)
    # They ran right down to the floor, but not while pushing down on it, where PX4's height
    # reads low on purpose.
    assert min(watch.heights_when_checked) < 0.05
    assert watch.checks_while_pushing_down == 0


# ---- Holding still, and giving up --------------------------------------------------------------

class WatchThatHoldsTwice:
    def __init__(self):
        self.checks = 0

    def check(self):
        self.checks += 1
        return self.checks <= 2


def test_fly_until_resends_the_last_target_while_the_watch_says_hold():
    link = PretendLink(height_metres=0.8)
    sent_onward = []
    flight_steps.fly_until(link, lambda: len(sent_onward) >= 3, lambda: sent_onward.append(link.seconds), 10.0,
                           "three onward targets", WatchThatHoldsTwice())

    assert [kind for kind, _ in link.targets.sent] == ["resend", "resend"]
    assert len(sent_onward) == 3


def test_fly_until_stops_the_run_when_it_takes_too_long():
    link = PretendLink(height_metres=0.8)
    with pytest.raises(RunMustStop, match="the impossible"):
        flight_steps.fly_until(link, lambda: False, lambda: None, 1.0, "the impossible")
