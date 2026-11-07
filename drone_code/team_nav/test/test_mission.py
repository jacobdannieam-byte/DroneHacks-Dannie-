"""mission.py against the pretend drone in fake_drone.py: the start, the takeoff, the rules checked
every step, and a whole flight with the inputs that exist today (none of the detectors)."""
import math

import pytest

from fake_drone import FakeDrone, fly
from team_nav.mission import Mission
from team_nav.mission_inputs import Clearances
from team_nav.mission_settings import MissionSettings

SWITCH_AT_SECONDS = 1.0


def reasons(mission):
    return [reason for _, _, reason in mission.history]


def first_time(mission, text):
    return next(seconds for seconds, _, reason in mission.history if text in reason)


def commands_after(record, seconds):
    return [step.command for step in record if step.seconds >= seconds]


def drone_height_at(record, seconds):
    return next(step.drone_position[2] for step in record if step.seconds >= seconds)


def feed_course_inputs(line_angle_left_radians=0.0):
    """Every detector input valid: vision healthy, the way ahead clear, a line straight ahead."""
    def every_step(now_seconds, mission, drone):
        mission.inputs.set_vision(now_seconds, True)
        mission.inputs.set_clearances(now_seconds, Clearances(math.inf, math.inf, math.inf, math.inf))
        mission.inputs.set_line(now_seconds, True, 0.0, line_angle_left_radians)
    return every_step


# ---------------------------------------------------------------- before the start
def test_sends_a_ready_target_but_never_flies_before_offboard():
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 5.0, switch_at_seconds=1e9)
    assert {step.command.kind for step in record} == {"position"}
    assert all(step.command.position[2] == pytest.approx(0.3) for step in record)   # just above the drone
    assert mission.phase == "waiting_for_start" and drone.position == [0.0, 0.0, 0.0]
    assert reasons(mission) == ["ready: waiting for Offboard (the pilot's switch to Position after arming)"]


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
    assert [phase for _, phase, _ in mission.history][-1] == "finished"
    said = reasons(mission)
    for expected in ("climbing to 0.8 m", "follow_line: camera tracking missing, holding",
                     "follow_line: camera tracking missing for 5 s, landing", "settling over the landing spot",
                     "descending at the medium speed", "descending gently", "touchdown confirmed: pushing down",
                     "landed: PX4 switched the motors off"):
        assert expected in said, expected
    downward_speeds = {round(-step.command.velocity[2], 2) for step in record if step.command.kind == "velocity"}
    assert downward_speeds == {0.5, 0.35, 1.0}                       # medium, gentle, push down
    gentle_starts = first_time(mission, "descending gently")
    assert drone_height_at(record, gentle_starts) == pytest.approx(0.3, abs=0.06)   # 0.5 m/s down to 0.3 m
    assert not drone.armed and drone.landed
    assert first_time(mission, "landed: PX4") - first_time(mission, "settling") < 12.0
    assert all(command.kind == "nothing" for command in commands_after(record, first_time(mission, "landed: PX4")))


def test_inputs_from_detectors_reach_the_course():
    mission, drone = Mission(MissionSettings(follow_line_time_limit_seconds=1e9)), FakeDrone()
    fly(mission, drone, 10.0, every_step=feed_course_inputs())
    assert mission.phase == "flying_course" and reasons(mission)[-1] == "follow_line: following the line"
    assert drone.position[0] > 2.0                                     # flew along the line (East)


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
    mission, drone = Mission(), FakeDrone()
    record = fly(mission, drone, 8.0, every_step=kill)
    assert mission.phase == "stopped" and "motors were switched off" in mission.reason
    assert all(command.kind == "nothing" for command in commands_after(record, 5.0))


# ---------------------------------------------------------------- safety rules while flying
def test_a_position_jump_lands_without_sideways_corrections_and_never_resumes():
    def jump(now_seconds, mission, drone):
        if now_seconds == pytest.approx(6.0):
            drone.position[0] += 0.6
    mission, drone = Mission(MissionSettings(follow_line_time_limit_seconds=1e9)), FakeDrone()
    record = fly(mission, drone, 40.0, every_step=lambda t, m, d: (feed_course_inputs()(t, m, d), jump(t, m, d)))
    jumped_at = first_time(mission, "position jumped")
    assert jumped_at == pytest.approx(6.0)
    landing_reason = next(reason for reason in reasons(mission) if reason.startswith("position jumped"))
    assert landing_reason.startswith("position jumped 0.6")              # 0.6 m, plus one step of flight
    assert landing_reason.endswith("every position goal dropped, landing where it is")
    after = commands_after(record, jumped_at)
    assert all(command.velocity[:2] == (0.0, 0.0) for command in after if command.kind == "velocity")
    assert all(step.phase in ("landing", "finished") for step in record if step.seconds >= jumped_at)
    assert mission.phase == "finished" and mission.course.pad_position is None


def test_a_jump_during_the_landing_drops_its_sideways_corrections():
    def jump(now_seconds, mission, drone):
        if mission.phase == "landing" and mission.landing.step_name == "descending_medium" and mission.fault is None:
            drone.position[1] += 0.7
    mission, drone = Mission(), FakeDrone()
    fly(mission, drone, 40.0, every_step=jump)
    assert mission.fault is not None and mission.landing.sideways_goal_allowed is False
    assert mission.phase == "finished"


def test_the_jump_limit_allows_for_a_gap_in_position_reports():
    def gap(now_seconds, mission, drone):
        drone.sends_position_reports = not (6.0 <= now_seconds < 6.15)
    mission, drone = Mission(MissionSettings(follow_line_time_limit_seconds=1e9)), FakeDrone()
    fly(mission, drone, 9.0, every_step=lambda t, m, d: (feed_course_inputs()(t, m, d), gap(t, m, d)))
    assert mission.fault is None and mission.phase == "flying_course"


def test_position_reports_stopping_holds_then_lands_on_the_laser():
    def tracking_stops(now_seconds, mission, drone):
        drone.sends_position_reports = now_seconds < 6.0
    mission, drone = Mission(MissionSettings(follow_line_time_limit_seconds=1e9)), FakeDrone()
    fly(mission, drone, 40.0, every_step=lambda t, m, d: (feed_course_inputs()(t, m, d), tracking_stops(t, m, d)))
    held_at = first_time(mission, "position reports stopped, holding")
    assert held_at == pytest.approx(6.0 + 0.2, abs=0.11)
    assert first_time(mission, "position reports stopped for 5 s, landing") - held_at == pytest.approx(5.0, abs=0.11)
    assert mission.phase == "finished" and not drone.armed


def test_the_flight_time_limit_lands_at_170_s_whatever_it_is_doing():
    settings = MissionSettings(follow_line_time_limit_seconds=1e9)
    mission, drone = Mission(settings), FakeDrone()
    fly(mission, drone, 200.0, every_step=feed_course_inputs(line_angle_left_radians=1.0))   # turning for ever
    assert settings.flight_time_limit_seconds == 170.0
    limit_at = first_time(mission, "flight time limit of 170 s reached, landing")
    assert limit_at - SWITCH_AT_SECONDS == pytest.approx(170.0, abs=0.11)
    assert set(reasons(mission)[3:reasons(mission).index("flight time limit of 170 s reached, landing")]) == {
        "follow_line: turning onto the line"}
    assert mission.phase == "finished"


def test_every_change_of_reason_is_logged_once():
    mission, drone = Mission(), FakeDrone()
    lines = []
    record = fly(mission, drone, 40.0)
    for step in record:
        lines += step.command.log_lines
    assert lines == [f"{phase}: {reason}" for _, phase, reason in mission.history]
    assert len(lines) == len(set(lines))
