"""Tests how the hand tests are chosen, run and saved (team_nav/hand_tests/run_hand_test.py),
using a pretend drone and a pretend person, so no drone or ROS is needed.

How to run these tests is in the package's README.
"""
import json
import os
from types import SimpleNamespace

from team_nav.hand_tests import laser_heights, run_hand_test
from team_nav.run_mode import RUN_MODE_SETTING


class PretendReports:
    """Stands in for DroneReports: PX4 is talking and the drone is level."""

    def __init__(self, px4_silent=False):
        self.px4_silent = px4_silent
        self.latest = {}
        self.topic_names = {}

    def seconds_since(self, report_name):
        return float("inf") if self.px4_silent else 0.1

    def has(self, report_name):
        return True

    def tilt_degrees(self):
        return 0.0

    def px4_distance_to_ground_metres(self):
        return 0.10


class PretendDrone:
    """Stands in for DroneListener, like drone 16 on 2026-11-07: no raw laser reading, but PX4
    reports its distance to the ground 20 times a second (fewer than the real 100, to keep the
    test fast) and says it uses the laser. The distance follows whatever height the pretend
    person holds it at, on a pretend clock that only moves when a test waits."""

    def __init__(self, px4_silent=False):
        self.reports = PretendReports(px4_silent)
        self.px4_topic_names = {}
        self.clock_seconds = 0.0
        self.laser_height_metres = 0.06
        self.px4_height_history = []

    def now(self):
        return self.clock_seconds

    def wait(self, seconds):
        end = self.clock_seconds + seconds
        while self.clock_seconds < end:
            self.clock_seconds += 0.05
            self.px4_height_history.append(SimpleNamespace(
                arrival_seconds=self.clock_seconds,
                distance_to_ground_metres=self.laser_height_metres + laser_heights.LASER_BELOW_CENTRE_METRES,
                z_metres=-self.laser_height_metres, uses_laser_for_height=True, uses_laser_for_ground=False,
                laser_declared_faulty=False))

    def px4_heights_since(self, start_seconds):
        # Strictly after: a report stamped at the start moment belongs to the wait before.
        return [sample for sample in self.px4_height_history if sample.arrival_seconds > start_seconds]

    def laser_since(self, start_seconds):
        return []

    def tilt_degrees_now(self):
        return 0.0

    def node_names(self):
        return ["flight_interface"]

    def read_parameters(self, node_name, names):
        return {"auto_offboard_on_arm": False, "allow_mode_request": False}


class PretendPerson:
    """Holds the drone at each height the laser test asks for, by reading the instruction."""

    def __init__(self, drone, error_metres=0.0):
        self.drone = drone
        self.error_metres = error_metres
        self.instructions = []

    def do_this_then_press_enter(self, instruction):
        self.instructions.append(instruction)
        for height in laser_heights.HELD_HEIGHTS_METRES:
            if f"is {height:.2f} m above the floor" in instruction:
                self.drone.laser_height_metres = height + self.error_metres

    def tell(self, text):
        pass


def test_no_name_lists_the_tests(capsys):
    assert run_hand_test.main([]) == 0
    listing = capsys.readouterr().out
    for name in ("laser_heights", "directions", "tunnel_carry", "touch_tilt", "laser_over_step", "all_table_checks"):
        assert name in listing


def test_an_unknown_name_is_refused_with_the_list(capsys):
    assert run_hand_test.main(["laser_height"]) == 2
    assert "There is no test called 'laser_height'" in capsys.readouterr().out


def test_all_table_checks_means_the_four_table_checks_in_order():
    assert run_hand_test.names_to_run("all_table_checks") == [
        "report_rates", "estimator_sources", "interface_settings", "tracking_still_drift"]
    assert run_hand_test.names_to_run("directions") == ["directions"]


def test_ros_arguments_are_not_taken_for_a_test_name(capsys):
    assert run_hand_test.main(["--ros-args", "-r", "__node:=x"]) == 0


def run_with_records_in(tmp_path, monkeypatch, names, drone, person):
    monkeypatch.setenv("TEAM_NAV_RECORDS_FOLDER", str(tmp_path))
    monkeypatch.delenv(RUN_MODE_SETTING, raising=False)
    return run_hand_test.run_tests(names, drone, person)


def saved_result(tmp_path, test_name):
    folders = [name for name in os.listdir(tmp_path) if name.endswith(f"hand_test_{test_name}")]
    assert len(folders) == 1
    with open(os.path.join(tmp_path, folders[0], "result.json"), encoding="utf-8") as file:
        return json.load(file), os.path.join(tmp_path, folders[0])


def test_the_laser_test_passes_on_a_good_laser_and_saves_its_readings(tmp_path, monkeypatch):
    drone = PretendDrone()
    person = PretendPerson(drone, error_metres=0.01)
    verdicts = run_with_records_in(tmp_path, monkeypatch, ["laser_heights"], drone, person)

    assert verdicts[0][1].passed, verdicts[0][1].reason
    assert "Hold the drone so the bottom laser is 0.50 m above the floor (measure with a tape)" in " ".join(person.instructions)
    result, folder = saved_result(tmp_path, "laser_heights")
    assert result["result"] == "PASS"
    assert os.path.exists(os.path.join(folder, "px4_height.csv"))


def test_the_laser_test_fails_on_a_laser_reading_10_cm_high(tmp_path, monkeypatch):
    drone = PretendDrone()
    verdicts = run_with_records_in(tmp_path, monkeypatch, ["laser_heights"], drone, PretendPerson(drone, error_metres=0.10))

    assert not verdicts[0][1].passed
    assert saved_result(tmp_path, "laser_heights")[0]["result"] == "FAIL"


def test_a_silent_flight_controller_stops_a_hand_test_at_once_with_a_clear_reason(tmp_path, monkeypatch):
    drone = PretendDrone(px4_silent=True)
    person = PretendPerson(drone)
    verdicts = run_with_records_in(tmp_path, monkeypatch, ["laser_heights"], drone, person)

    assert "is the flight controller powered (battery connected)?" in verdicts[0][1].reason
    assert person.instructions == []


def test_the_interface_check_reads_the_settings(tmp_path, monkeypatch):
    drone = PretendDrone()
    verdicts = run_with_records_in(tmp_path, monkeypatch, ["interface_settings"], drone, PretendPerson(drone))
    assert verdicts[0][1].passed, verdicts[0][1].reason
