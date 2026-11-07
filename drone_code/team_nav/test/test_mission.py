"""mission.py against the pretend drone in fake_drone.py: the start, the takeoff, the rules checked
every step, the landing hand-over to velocity commands, and whole flights with the inputs that
exist today (none of the detectors)."""
import math

import pytest

from fake_drone import FakeDrone, fly
from team_nav.mission import Mission
from team_nav.mission_inputs import Clearances
from team_nav.mission_settings import MissionSettings

SWITCH_AT_SECONDS = 1.0
STEP_SECONDS = 0.1


def reasons(mission):
    return [reason for _, _, reason in mission.history]


def phases(mission):
    return [phase for _, phase, _ in mission.history]


def first_time(mission, text):
    return next(seconds for seconds, _, reason in mission.history if text in reason)


def commands_after(record, seconds):
    return [step.command for step in record if step.seconds >= seconds]


def drone_height_at(record, seconds):
    return next(step.drone_position[2] for step in record if step.seconds >= seconds)


def every_step_do(*actions):
    def every_step(now_seconds, mission, drone):
        for action in actions:
            action(now_seconds, mission, drone)
    return every_step


def feed_course_inputs(line_angle_left_radians=0.0, line_offset_left_metres=0.0):
    """Every detector input valid: vision healthy, the way ahead clear, a line to follow."""
    def every_step(now_seconds, mission, drone):
        mission.inputs.set_vision(now_seconds, True)
        mission.inputs.set_clearances(now_seconds, Clearances(math.inf, math.inf, math.inf, math.inf))
        mission.inputs.set_line(now_seconds, True, line_offset_left_metres, line_angle_left_radians)
    return every_step


ENDLESS_COURSE = MissionSettings(follow_line_time_limit_seconds=1e9)


# ---------------------------------------------------------------- before the start
def test_sends_a_ready_target_but_never_flies_before_offboard():
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 5.0, switch_at_seconds=1e9)
    assert {step.command.kind for step in record} == {"position"}
    assert all(step.command.position[2] == pytest.approx(0.3) for step in record)   # just above the drone
    assert mission.phase == "waiting_for_start" and drone.position == [0.0, 0.0, 0.0]


def test_without_a_ready_target_the_hand_over_would_be_refused():
    drone = FakeDrone()
    drone.arm()
    assert drone.switch_to_position_mode(1.0) is False        # what the organizers' code does without targets


def test_no_start_if_offboard_was_already_active_when_first_heard():
    mission, drone = Mission(), FakeDrone()
    drone.armed = drone.offboard = True
    record = fly(mission, drone, 3.0, switch_at_seconds=1e9)
    assert mission.phase == "stopped" and "already active" in mission.reason
    assert all(step.command.kind == "nothing" for step in record)


def test_no_start_if_px4_does_not_report_the_drone_on_the_ground():
    def offboard_while_flying(now_seconds, mission, drone):        # forced: the organizers' code would refuse
        if now_seconds >= SWITCH_AT_SECONDS:
            drone.offboard, drone.landed = True, False
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 3.0, every_step=offboard_while_flying, switch_at_seconds=1e9)
    assert mission.phase == "stopped" and "not report the drone on the ground" in mission.reason
    assert all(command.kind == "nothing" for command in commands_after(record, SWITCH_AT_SECONDS))


# ---------------------------------------------------------------- the takeoff
def test_takeoff_starts_on_the_first_step_and_reaches_cruise_height():
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 6.0)
    first = commands_after(record, SWITCH_AT_SECONDS)[0]
    assert first.kind == "position" and first.position[2] >= 0.3
    assert first_time(mission, "climbing") == SWITCH_AT_SECONDS
    assert first_time(mission, "at cruise height") - SWITCH_AT_SECONDS < 10.0
    assert drone.position[2] == pytest.approx(0.8, abs=0.1)


def test_cruise_height_is_a_setting():
    mission, drone = Mission(MissionSettings(cruise_height_metres=0.6)), FakeDrone()
    fly(mission, drone, 6.0)
    assert MissionSettings().cruise_height_metres == 0.8
    assert drone.position[2] == pytest.approx(0.6, abs=0.1)


def test_a_drone_that_never_leaves_the_ground_stops_publishing_after_10_s():
    mission, drone = Mission(), FakeDrone(motors_can_lift=False)
    record = fly(mission, drone, 15.0)
    stopped_at = first_time(mission, "did not leave the ground")
    assert stopped_at - SWITCH_AT_SECONDS == pytest.approx(10.0, abs=0.15)
    assert all(command.kind == "nothing" for command in commands_after(record, stopped_at))


# ---------------------------------------------------------------- a whole flight with today's inputs
def test_with_no_detectors_it_takes_off_holds_and_lands_by_section_7():
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 40.0)
    assert "finished" in phases(mission)
    said = reasons(mission)
    for expected in ("climbing to 0.8 m", "follow_line: camera tracking missing, holding",
                     "follow_line: camera tracking missing for 5 s, landing", "settling over the landing spot",
                     "descending at the medium speed", "descending gently", "touchdown confirmed: pushing down"):
        assert expected in said, expected
    assert any(reason.startswith("landed: PX4 switched the motors off (peak horizontal speed") for reason in said)
    downward_speeds = {round(-step.command.velocity[2], 2) for step in record if step.command.kind == "velocity"}
    assert downward_speeds == {0.5, 0.35, 1.0}                       # medium, gentle, push down
    gentle_starts = first_time(mission, "descending gently")
    assert drone_height_at(record, gentle_starts) == pytest.approx(0.3, abs=0.06)   # 0.5 m/s down to 0.3 m
    assert not drone.armed and drone.landed
    motors_off_at = first_time(mission, "landed: PX4")
    assert motors_off_at - first_time(mission, "settling") < 12.0


def test_inputs_from_detectors_reach_the_course():
    mission, drone = Mission(ENDLESS_COURSE), FakeDrone()
    fly(mission, drone, 10.0, every_step=feed_course_inputs())
    assert mission.phase == "flying_course" and reasons(mission)[-1] == "follow_line: following the line"
    assert drone.position[0] > 2.0                                     # flew along the line (East)


# ---------------------------------------------------------------- the hand-over to velocity commands
def first_velocity_step(record):
    return next(step for step in record if step.command.kind == "velocity")


def test_a_position_target_at_the_drone_is_sent_just_before_the_first_velocity_command():
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 40.0)
    first = first_velocity_step(record)
    hold_point = first.command.position_before_velocity
    assert hold_point is not None
    assert math.dist(hold_point[:3], first.drone_position) < 1e-9          # exactly where the drone is
    later = [step.command for step in record if step.command.kind == "velocity" and step is not first]
    assert all(command.position_before_velocity is None for command in later)


def test_velocity_commands_never_stop_from_the_first_until_the_motors_are_off():
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 40.0)
    start = first_velocity_step(record).seconds
    motors_off_at = first_time(mission, "landed: PX4")
    during = [step.command.kind for step in record if start <= step.seconds < motors_off_at]
    assert during and set(during) == {"velocity"}                           # through touchdown, every step


def test_if_the_velocity_commands_stopped_the_drone_would_hold_where_the_landing_began():
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 40.0, until=lambda step: step.command.kind == "velocity")
    hold_point = record[-1].command.position_before_velocity
    now = record[-1].seconds
    for _ in range(40):                                                     # the program dies: 4 s, no commands
        now += STEP_SECONDS
        drone.advance(now, STEP_SECONDS)
    assert math.hypot(drone.position[0] - hold_point[0], drone.position[1] - hold_point[1]) < 0.05
    assert drone.position[2] == pytest.approx(hold_point[2], abs=0.1)       # back at the hold point, not elsewhere


# ---------------------------------------------------------------- speed
def test_position_targets_stay_within_half_a_metre_of_the_drone():
    def drift_east_during_takeoff(now_seconds, mission, drone):
        if mission.phase == "taking_off":
            drone.position[0] += 0.04                                       # 0.4 m/s of drift, steady
    mission, drone = Mission(ENDLESS_COURSE), FakeDrone()
    record = fly(mission, drone, 20.0, every_step=every_step_do(
        feed_course_inputs(line_offset_left_metres=0.6), drift_east_during_takeoff))
    flying = [step for step in record if step.seconds >= SWITCH_AT_SECONDS and step.command.kind == "position"]
    furthest = max(math.hypot(step.command.position[0] - step.drone_position[0],
                              step.command.position[1] - step.drone_position[1]) for step in flying)
    assert furthest <= MissionSettings().carrot_lead_metres + 1e-9
    assert furthest > 0.45                                                  # the limit was really reached


def test_a_position_target_far_from_the_drone_is_pulled_in_to_half_a_metre():
    mission = Mission()
    mission.observe_position(10.0, (0.0, 0.0, 0.8), 0.0)
    command = mission.position_command(10.0, (2.0, 1.0, 0.8, 0.0), "test")
    assert math.hypot(command.position[0], command.position[1]) == pytest.approx(0.5)
    assert command.position[0] / command.position[1] == pytest.approx(2.0)   # same direction, just closer


def test_the_mission_steps_at_20_hz():
    assert MissionSettings().steps_per_second == 20.0


def test_the_peak_horizontal_speed_is_logged():
    mission, drone = Mission(ENDLESS_COURSE), FakeDrone()
    lines = []
    for step in fly(mission, drone, 15.0, every_step=feed_course_inputs()):
        lines += step.command.log_lines
    peaks = [line for line in lines if "peak horizontal speed so far" in line]
    assert peaks
    peak = mission.position_watch.peak_horizontal_speed_metres_per_second
    assert 0.4 < peak < 0.7                                                 # about the cruise speed


# ---------------------------------------------------------------- fresh reports before advancing
@pytest.mark.parametrize("what_stops, expected", [
    ("sends_position_reports", "camera tracking position reports stopped"),
    ("sends_px4_reports", "PX4 position reports stopped"),
    ("sends_flight_state", "flight state reports stopped"),
    ("px4_link", "the flight state says the link to PX4 is down"),
])
def test_each_report_must_be_fresh_on_its_own_before_anything_advances(what_stops, expected):
    def stop_it(now_seconds, mission, drone):
        setattr(drone, what_stops, now_seconds < 6.0)
    mission, drone = Mission(ENDLESS_COURSE), FakeDrone()
    record = fly(mission, drone, 13.0, every_step=every_step_do(feed_course_inputs(), stop_it))
    held_at = first_time(mission, f"{expected}, holding")
    assert 6.0 <= held_at <= 7.1
    held = [step.command for step in record if held_at + 0.1 <= step.seconds < held_at + 4.0]
    assert len({command.position for command in held}) == 1                 # the target stopped advancing
    assert first_time(mission, f"{expected} for 5 s, landing") - held_at == pytest.approx(5.0, abs=0.11)


def test_position_reports_stopping_holds_then_lands_and_px4_finishes_the_landing():
    def tracking_stops(now_seconds, mission, drone):
        drone.sends_position_reports = now_seconds < 6.0
    mission, drone = Mission(ENDLESS_COURSE), FakeDrone()
    fly(mission, drone, 45.0, every_step=every_step_do(feed_course_inputs(), tracking_stops))
    assert "camera tracking position reports stopped, holding" in reasons(mission)
    assert "touchdown confirmed: pushing down" not in reasons(mission)    # no stillness measure: no push
    assert "finished" in phases(mission) and not drone.armed               # PX4 landed it by itself


# ---------------------------------------------------------------- control that stops being ours
def test_offboard_lost_stops_publishing_for_good():
    def pilot_takes_over(now_seconds, mission, drone):
        if 5.0 <= now_seconds < 6.0:
            drone.offboard = False
        elif now_seconds >= 6.0:
            drone.offboard = True                                      # even if it comes back
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 10.0, every_step=pilot_takes_over)
    assert mission.phase == "stopped" and "PX4 left Offboard" in mission.reason
    assert all(command.kind == "nothing" for command in commands_after(record, 5.0))


def test_the_kill_switch_stops_publishing():
    def kill(now_seconds, mission, drone):
        if now_seconds >= 5.0:
            drone.armed = False
            drone.landed = False                                       # still falling: not on the ground yet
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 8.0, every_step=kill)
    assert mission.phase == "stopped" and "motors were switched off" in mission.reason
    assert all(command.kind == "nothing" for command in commands_after(record, 5.0))


# ---------------------------------------------------------------- position jumps
class JumpOnce:
    """Moves the drone's reported position suddenly, once, the first time ready(now, mission) is true."""

    def __init__(self, ready, east_metres=0.0, north_metres=0.0):
        self.ready = ready
        self.east_metres, self.north_metres = east_metres, north_metres
        self.done = False

    def __call__(self, now_seconds, mission, drone):
        if not self.done and self.ready(now_seconds, mission):
            drone.position[0] += self.east_metres
            drone.position[1] += self.north_metres
            self.done = True


def at_six_seconds(now_seconds, mission):
    return now_seconds == pytest.approx(6.0)


def in_the_medium_descent(now_seconds, mission):
    return mission.phase == "landing" and mission.landing.step_name == "descending_medium"


def pushing_down(now_seconds, mission):
    return mission.phase == "landing" and mission.landing.step_name == "pushing_down"


def test_a_position_jump_latches_a_fault_and_sends_only_a_zero_velocity():
    mission, drone = Mission(ENDLESS_COURSE), FakeDrone()
    record = fly(mission, drone, 20.0, every_step=every_step_do(feed_course_inputs(), JumpOnce(at_six_seconds, east_metres=0.6)))
    fault_at = first_time(mission, "position jumped")
    assert fault_at == pytest.approx(6.0)
    log_line = next(reason for reason in reasons(mission) if reason.startswith("position jumped"))
    assert log_line.endswith("fault latched; no more targets, recovery is up to the pilot")
    after = commands_after(record, fault_at)
    assert all(command.kind == "velocity" and command.velocity == (0.0, 0.0, 0.0) for command in after)
    assert all(command.position_before_velocity is None for command in after)   # no position hold
    assert "landing" not in phases(mission) and mission.phase == "faulted"
    assert mission.course.pad_position is None and mission.course.free_space_seen_until is None


def test_after_a_fault_the_pilot_taking_over_ends_publishing():
    def pilot_takes_over(now_seconds, mission, drone):
        if now_seconds >= 9.0:
            drone.offboard = False
    mission, drone = Mission(ENDLESS_COURSE), FakeDrone()
    record = fly(mission, drone, 12.0, every_step=every_step_do(feed_course_inputs(), JumpOnce(at_six_seconds, east_metres=0.6),
                                                                pilot_takes_over))
    assert mission.phase == "stopped"
    assert all(command.kind == "nothing" for command in commands_after(record, 9.0))


def test_a_jump_during_the_descent_also_faults():
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 30.0, every_step=JumpOnce(in_the_medium_descent, north_metres=0.7))
    fault_at = first_time(mission, "position jumped")
    assert mission.phase == "faulted"
    assert all(command.velocity == (0.0, 0.0, 0.0) for command in commands_after(record, fault_at))


def test_a_jump_after_touchdown_is_ignored_because_position_is_no_longer_used():
    mission, drone = Mission(), FakeDrone()
    fly(mission, drone, 40.0, every_step=JumpOnce(pushing_down, north_metres=0.7))
    assert mission.fault is None and "finished" in phases(mission)


def test_the_jump_limit_allows_for_a_gap_in_position_reports():
    def gap(now_seconds, mission, drone):
        drone.sends_position_reports = not (6.0 <= now_seconds < 6.15)
    mission, drone = Mission(ENDLESS_COURSE), FakeDrone()
    fly(mission, drone, 9.0, every_step=every_step_do(feed_course_inputs(), gap))
    assert mission.fault is None and mission.phase == "flying_course"


# ---------------------------------------------------------------- the flight time limit
def test_the_flight_time_limit_lands_at_170_s_whatever_it_is_doing():
    mission, drone = Mission(ENDLESS_COURSE), FakeDrone()
    fly(mission, drone, 200.0, every_step=feed_course_inputs(line_angle_left_radians=1.0))   # turning for ever
    assert MissionSettings().flight_time_limit_seconds == 170.0
    limit_at = first_time(mission, "flight time limit of 170 s reached, landing")
    assert limit_at - SWITCH_AT_SECONDS == pytest.approx(170.0, abs=0.11)
    course_reasons = [reason for seconds, phase, reason in mission.history if phase == "flying_course"
                      and "at cruise height" not in reason and seconds < limit_at]
    assert set(course_reasons) == {"follow_line: turning onto the line"}
    assert "finished" in phases(mission)


def test_the_flight_time_limit_never_interrupts_a_landing():
    settings = MissionSettings(flight_time_limit_seconds=10.0)         # runs out during the landing
    mission, drone = Mission(settings), FakeDrone()
    record = fly(mission, drone, 40.0)
    landing_began = first_time(mission, "camera tracking missing for 5 s, landing")
    assert landing_began - SWITCH_AT_SECONDS < settings.flight_time_limit_seconds
    assert not any("flight time limit" in reason for reason in reasons(mission))
    start = first_velocity_step(record).seconds
    motors_off_at = first_time(mission, "landed: PX4")
    assert {step.command.kind for step in record if start <= step.seconds < motors_off_at} == {"velocity"}


# ---------------------------------------------------------------- another run
def test_a_second_run_starts_with_every_run_derived_memory_cleared():
    mission, drone = Mission(), FakeDrone()
    first_run = fly(mission, drone, 30.0)
    drone.offboard = False                                             # the pilot moves the mode switch
    first_run += fly(mission, drone, 1.0, start_seconds=first_run[-1].seconds + STEP_SECONDS,
                     switch_at_seconds=1e9)
    assert "finished" in phases(mission) and mission.phase == "waiting_for_start"
    assert "disarmed on the ground: ready for another run" in reasons(mission)
    old_course = mission.course
    mission.course.pad_position = (9.0, 9.0)                          # pretend memories of the first run
    mission.inputs.set_line(29.9, True, 0.0, 0.0, end_seen=True)
    drone.has_flown = False                                            # the pilot arms again
    second_start = first_run[-1].seconds + STEP_SECONDS
    snapshot = SnapshotAtTakeoff()
    fly(mission, drone, 25.0, start_seconds=second_start, arm_at_seconds=second_start + 0.5,
        switch_at_seconds=second_start + 1.0, every_step=snapshot)
    assert snapshot.course is not old_course and snapshot.pad_position is None
    assert snapshot.line_stamp is None and snapshot.fault is None and snapshot.peak_speed < 0.1
    assert phases(mission).count("finished") == 2                      # the second run landed too


def test_offboard_still_reported_after_disarming_is_not_a_start_and_gets_no_target():
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 30.0)
    motors_off_at = first_time(mission, "landed: PX4")
    assert drone.offboard and not drone.armed                          # PX4 left in Offboard mode
    after = commands_after(record, motors_off_at)
    assert all(command.kind == "nothing" for command in after)
    assert reasons(mission)[-1] == "PX4 still reports Offboard while disarmed: waiting for it to end before the next run"
    assert "stopped" not in phases(mission)


class SnapshotAtTakeoff:
    """Records the mission's run memory at the first step of the takeoff."""

    def __init__(self):
        self.taken = False

    def __call__(self, now_seconds, mission, drone):
        if self.taken or mission.phase != "taking_off":
            return
        self.taken = True
        self.course = mission.course
        self.pad_position = mission.course.pad_position
        self.line_stamp = mission.inputs.line_stamp_seconds
        self.fault = mission.fault
        self.peak_speed = mission.position_watch.peak_horizontal_speed_metres_per_second


def test_every_change_of_reason_is_logged_once():
    mission, drone = Mission(), FakeDrone()
    lines = []
    for step in fly(mission, drone, 25.0):
        lines += step.command.log_lines
    reason_lines = [line for line in lines if "peak horizontal speed so far" not in line]
    assert reason_lines == [f"{phase}: {reason}" for _, phase, reason in mission.history]
