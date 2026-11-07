"""Runs our hand tests and table checks (drone_code/team_nav/team_nav/hand_tests/) against the
simulator, to show that they run, read the real ROS topics, and reach sensible verdicts.

SIMULATOR TEST. Do not run this on the real drone: it flies the drone, and it first asks for the
simulator's start service, so on the real drone it stops before sending anything. The hand tests
themselves never send anything; only this file's pretend person flies.

It needs an open floor, because the hand tests move the drone 1 m to the left of where it
starts, and on the real course that would be outside the net. So first make a world with the
organizers' course edges but no walls or boulders, with the launch mark in the middle (this
runs on the Mac and needs no ROS), then fly in it:

    python3 simulator/test_flights/hand_tests_in_simulator.py --make-open-floor-world
    simulator/run_test_flight.sh --world simulator/generated_files/hand_test_worlds/open_floor.json hand_tests_in_simulator.py

How it works
------------
On the real drone a person moves the drone and presses Enter. Here a pretend person does it:
  1. On the ground, it runs the four table checks, then two hand tests with a person who
     doesn't move the drone (the simulated drone can't be picked up).
  2. It takes off to 0.8 m and runs three hand tests with a person who "moves" the drone by
     flying it: 1 m forward, 1 m left and a 90 degree left turn; a 2 m backwards-and-back
     "carry"; and holding still. Then it lands.

Some tests rightly FAIL here, and EXPECTED_RESULTS says which and why. This file fails only if
a test's result differs from what we expect, or a test crashes.

What the simulator can't show: camera pictures and lens settings (the simulator has none, so
those topics are reported missing), a ROS network number (the box has no network), a real
laser's beam width, and a real person's tilt.
"""
import json
import math
import os
import sys

# Making the open-floor world runs on the Mac, where ROS isn't installed. So the ROS parts are
# only imported below when the simulator test itself runs.
if "--make-open-floor-world" not in sys.argv:
    from geometry_msgs.msg import PoseStamped

    from simulator_test_flight import CRUISE_HEIGHT_METRES, LONGEST_FLIGHT_STAGE_SECONDS, SimulatorTestFlight
    from team_nav.hand_tests.drone_listener import DroneListener
    from team_nav.hand_tests.run_hand_test import run_tests
else:
    SimulatorTestFlight = object   # Only so the class below can be defined; it is never used then.

REPOSITORY = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
COURSE_DRAWING_MAP = os.path.join(REPOSITORY, "drone_code/team_nav/course_maps/organizers_drawing.json")
# Inside simulator/generated_files, which Git ignores.
OPEN_FLOOR_WORLD = os.path.join(REPOSITORY, "simulator/generated_files/hand_test_worlds/open_floor.json")

# Inside the simulator's box this folder is in the repository, and Git ignores it.
RECORDS_FOLDER_IN_THE_BOX = "/repository/simulator/generated_files/team_runs"

# test name -> (expected to pass?, why)
EXPECTED_RESULTS = {
    "report_rates": (True, "every needed report exists in the simulator; camera pictures are not needed"),
    "estimator_sources": (True, "the simulator's PX4 has drone 16's estimator settings"),
    "interface_settings": (False, "the simulator, like the organizers' own launch file, sets auto_offboard_on_arm "
                                  "to true so it can fly; on the drone our launch file sets it false"),
    "tracking_still_drift": (True, "the pretend tracking only drifts while moving"),
    "laser_heights": (False, "nobody can lift the simulated drone, so only the floor reading can pass"),
    "laser_over_step": (False, "there is no box to carry the simulated drone over"),
    "directions": (True, "the pretend person flies the moves"),
    "tunnel_carry": (True, "the pretend person flies 2 m backwards and back"),
    "touch_tilt": (False, "a hovering drone can't be tilted 25 degrees, so only the still part can pass"),
}
TESTS_ON_THE_GROUND = ["report_rates", "estimator_sources", "interface_settings", "tracking_still_drift",
                       "laser_heights", "laser_over_step"]
TESTS_IN_FLIGHT = ["directions", "tunnel_carry", "touch_tilt"]

# Our own choices, for the pretend person's flying.
POSITION_TOLERANCE_METRES = 0.08
SECONDS_TO_SETTLE_AFTER_A_MOVE = 4.0
TIMES_TO_SEND_EACH_TARGET = 5
SECONDS_BETWEEN_SENDS = 0.1
# Our own choice: the referee rewrites its summary every second (referee.py), so wait for two.
SECONDS_FOR_THE_REFEREE_SUMMARY = 2.0
# Negative: backwards, which keeps the drone in the middle of the open floor after the left move.
OUT_AND_BACK_METRES = -2.0
BOX_HEIGHT_THE_PRETEND_PERSON_TYPES_METRES = 0.30
SECONDS_THE_STILL_PERSON_TAKES = 3.0


class HandTestPilot(SimulatorTestFlight):
    """Flies the simulated drone where the pretend person is told to carry it.

    While the hand tests run, it does NOT keep processing its own messages, as the other test
    flights do. In the first runs on 2026-11-07 that busy loop starved the hand tests' listener,
    which then missed camera tracking reports for over a second, a fault the real drone would not
    have. A position target stays in force in the organizers' interface until it is replaced, so
    the pilot sends each target a few times and then just watches the listener's tracking."""

    def __init__(self):
        super().__init__("hand_tests_in_simulator")
        self.listener = None   # Set once the hand tests' listener exists.

    def tracked_forward_and_left_metres(self):
        latest = self.listener.tracking_history[-1]
        return latest.forward_metres, latest.left_metres

    def fly_to(self, forward_metres, left_metres, heading_degrees=None):
        """Flies to this tracked position at cruise height, optionally turning to a heading
        (tracking's heading, growing to the left), then waits for the drone to settle."""
        for _ in range(TIMES_TO_SEND_EACH_TARGET):
            self.send_target_with_heading(forward_metres, left_metres, CRUISE_HEIGHT_METRES, heading_degrees)
            self.listener.wait(SECONDS_BETWEEN_SENDS)
        give_up_at = self.listener.now() + LONGEST_FLIGHT_STAGE_SECONDS
        while True:
            forward, left = self.tracked_forward_and_left_metres()
            if math.hypot(forward - forward_metres, left - left_metres) < POSITION_TOLERANCE_METRES:
                break
            if self.listener.now() > give_up_at:
                raise TimeoutError(f"The pretend person's flight to ({forward_metres:.2f}, {left_metres:.2f}) took "
                                   f"over {LONGEST_FLIGHT_STAGE_SECONDS:.0f} s; the drone is at ({forward:.2f}, {left:.2f}).")
            self.listener.wait(SECONDS_BETWEEN_SENDS)
        self.listener.wait(SECONDS_TO_SETTLE_AFTER_A_MOVE)

    def send_target_with_heading(self, forward_metres, left_metres, up_metres, heading_degrees):
        if heading_degrees is None:
            self.send_target_position(forward_metres, left_metres, up_metres)
            return
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "odom"
        target.pose.position.x = float(forward_metres)
        target.pose.position.y = float(left_metres)
        target.pose.position.z = float(up_metres)
        # A turn about the vertical axis by heading_degrees, as a quaternion.
        target.pose.orientation.z = math.sin(math.radians(heading_degrees) / 2.0)
        target.pose.orientation.w = math.cos(math.radians(heading_degrees) / 2.0)
        self.pose_publisher.publish(target)


class PersonWhoCannotLiftTheDrone:
    """Stands in for the person on the ground: reads each instruction, waits, presses Enter."""

    def __init__(self, listener):
        self.listener = listener

    def do_this_then_press_enter(self, instruction):
        print(f"\n>>> {instruction}\n    (pretend person: the simulated drone can't be lifted; waiting, then Enter)", flush=True)
        self.listener.wait(SECONDS_THE_STILL_PERSON_TAKES)

    def type_a_number(self, question):
        print(f"\n>>> {question} {BOX_HEIGHT_THE_PRETEND_PERSON_TYPES_METRES}", flush=True)
        return BOX_HEIGHT_THE_PRETEND_PERSON_TYPES_METRES

    def tell(self, text):
        print(f"    {text}", flush=True)


class PersonWhoFliesTheDrone(PersonWhoCannotLiftTheDrone):
    """Stands in for the person in the hand tests that move the drone, by flying it. It
    recognises each instruction by a phrase in it."""

    def __init__(self, listener, pilot):
        super().__init__(listener)
        self.pilot = pilot
        self.mark_forward_metres, self.mark_left_metres = pilot.tracked_forward_and_left_metres()

    def do_this_then_press_enter(self, instruction):
        print(f"\n>>> {instruction}", flush=True)
        forward, left = self.mark_forward_metres, self.mark_left_metres
        if "on mark A" in instruction or "on mark S" in instruction:
            self.mark_forward_metres, self.mark_left_metres = self.pilot.tracked_forward_and_left_metres()
            print("    (pretend person: this spot is the mark)", flush=True)
            self.pilot.fly_to(self.mark_forward_metres, self.mark_left_metres)
        elif "on mark B" in instruction:
            self.pilot.fly_to(forward + 1.0, left)
        elif "on mark C (1 m to its left)" in instruction:
            self.pilot.fly_to(forward + 1.0, left + 1.0)
        elif "90 degrees to the LEFT" in instruction:
            heading_now = self.listener.tracking_history[-1].heading_degrees
            self.pilot.fly_to(forward + 1.0, left + 1.0, heading_now + 90.0)
        elif "carry it through the whole tunnel" in instruction:
            self.pilot.fly_to(forward + OUT_AND_BACK_METRES, left)
            self.pilot.fly_to(forward, left)
        else:
            print("    (pretend person: nothing to fly for this one; waiting, then Enter)", flush=True)
            self.listener.wait(SECONDS_THE_STILL_PERSON_TAKES)


def make_open_floor_world():
    """Writes a copy of the organizers' drawing with no walls, tubes or boulders, and the launch
    mark in the middle of the floor."""
    with open(COURSE_DRAWING_MAP, encoding="utf-8") as file:
        course = json.load(file)
    course["name"] = "Open floor for the hand tests (simulator only)"
    course["description"] = ("The organizers' drawing with every wall, tube and boulder removed and the launch mark "
                             "moved to the middle, made by simulator/test_flights/hand_tests_in_simulator.py.")
    course["walls"] = []
    course["tubes"] = []
    course["low_obstacles"] = []
    # A checkpoint line through an opening needs the wall with that opening, which is gone.
    lines_without_openings = []
    for line in course["checkpoint_lines"]:
        if "opening_name" not in line:
            lines_without_openings.append(line)
    course["checkpoint_lines"] = lines_without_openings
    course["launch_mark"] = {"centre_metres": [0.0, 0.0], "source": "Moved to the middle of the floor for the hand tests."}
    os.makedirs(os.path.dirname(OPEN_FLOOR_WORLD), exist_ok=True)
    with open(OPEN_FLOOR_WORLD, "w", encoding="utf-8") as file:
        json.dump(course, file, indent=2)
    print(f"Wrote {OPEN_FLOOR_WORLD}")


def main():
    if "--make-open-floor-world" in sys.argv:
        make_open_floor_world()
        return 0
    os.environ.setdefault("TEAM_NAV_RECORDS_FOLDER", RECORDS_FOLDER_IN_THE_BOX)
    import rclpy
    rclpy.init()
    pilot = HandTestPilot()
    pilot.refuse_to_run_outside_the_simulator()
    pilot.wait_for_simulator()
    listener = DroneListener()
    pilot.listener = listener
    verdicts = run_tests(TESTS_ON_THE_GROUND, listener, PersonWhoCannotLiftTheDrone(listener))

    pilot.remember_the_floor()
    pilot.press_start()
    pilot.climb()
    verdicts += run_tests(TESTS_IN_FLIGHT, listener, PersonWhoFliesTheDrone(listener, pilot))
    pilot.fly_to(pilot.start_position[0], pilot.start_position[1])
    pilot.wait_for_motors_off(pilot.descend_until_touchdown())
    # Let the referee write its summary of the flight before the simulator stops.
    listener.wait(SECONDS_FOR_THE_REFEREE_SUMMARY)
    listener.close()
    # Stop ROS cleanly; leaving it running at exit made the program abort on 2026-11-07.
    pilot.destroy_node()
    rclpy.shutdown()
    return compare_with_expectations(verdicts)


def compare_with_expectations(verdicts):
    print("\n===== Hand tests in the simulator: results against expectations =====")
    surprises = 0
    for name, verdict in verdicts:
        expected_to_pass, why = EXPECTED_RESULTS[name]
        as_expected = verdict.passed == expected_to_pass
        surprises += 0 if as_expected else 1
        print(f"{'as expected' if as_expected else 'SURPRISE   '}  {'PASS' if verdict.passed else 'FAIL'}  {name}: "
              f"{verdict.reason}\n             Expected {'PASS' if expected_to_pass else 'FAIL'} because {why}.")
    if surprises or len(verdicts) != len(EXPECTED_RESULTS):
        print(f"TEST FAILED: {surprises} result(s) differ from what we expect, or a test didn't run.")
        return 1
    print("Every hand test ran and gave the result we expect in the simulator.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
