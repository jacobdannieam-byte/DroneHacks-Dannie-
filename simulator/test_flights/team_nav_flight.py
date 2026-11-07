"""Presses start for our real mission program (drone_code/team_nav), then watches it fly.

SIMULATOR TEST. Do not run this on the real drone: it asks for the simulator's start service.
simulator/run_team_nav.sh runs it, after starting the simulator and our launch file.

What it does
------------
It sends the drone nothing. Everything the drone does comes from team_nav, through the same topics
it uses on drone 16. This program's only action is the call to /simulator_only/start_run, which
stands in for the person who arms the drone and flips the switch to Position.

It checks, in order, and prints each moment as it happens:
  1. The simulator is ready: PX4 is linked, says the drone is on the ground, and has a position
     and a laser reading.
  2. Our mission is sending its ready target. The organizers' code refuses to hand over control
     without one.
  3. After pressing start: the hand-over to automatic flight (Offboard), the climb to about the
     cruise height, the push-down at the end of the landing, and the motors switching off.
If a step takes too long, it stops with a sentence saying what did not happen. At the end it
prints a RESULT line and exits with 0 only if everything expected happened.

    --mission stages   (the default) Expected today: the hand-over, a climb to 0.8 m, a hold of
                       about 5 s (there is no line input yet, so the first stage waits for one),
                       the landing with the push-down, and the motors switching off.
    --mission route    The route in drone_code/team_nav/missions/route_waypoints.json. It also
                       checks, from the camera tracking reports, how close the drone came to each
                       waypoint, and whether it reached the last one before landing.
"""
import argparse
import math
import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

# From the mission's settings (drone_code/team_nav/team_nav/mission_settings.py).
CRUISE_HEIGHT_METRES = 0.8
PUSH_DOWN_METRES_PER_SECOND = 1.0

# Our own choices for this test.
CHECKS_PER_SECOND = 20.0
CRUISE_HEIGHT_TOLERANCE_METRES = 0.1
SECONDS_TO_SETTLE_BEFORE_START = 5.0
OUR_TARGETS_COUNT_AS_LIVE_SECONDS = 0.5
PUSH_DOWN_SEEN_AT_METRES_PER_SECOND = 0.9       # a command at least this fast downward

# The route mission's waypoints: the same files the mission reads (--route-file picks one).
MISSIONS_FOLDER = "/repository/drone_code/team_nav/missions"
# Our own choice: the drone counts as having reached a waypoint within this distance (the mission
# itself uses 0.15 m).
WAYPOINT_REACHED_METRES = 0.2

# How long each step may take before we call it a failure.
LONGEST_WAIT_FOR_SIMULATOR_SECONDS = 180.0
LONGEST_WAIT_FOR_OUR_MISSION_SECONDS = 60.0
LONGEST_WAIT_FOR_HAND_OVER_SECONDS = 10.0
LONGEST_CLIMB_SECONDS = 20.0
LONGEST_WAIT_FOR_MOTORS_OFF_SECONDS = {"stages": 90.0, "route": 300.0}

PX4_MESSAGE_SETTINGS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class TeamNavFlightWatcher(Node):

    def __init__(self, mission_kind, route_file):
        super().__init__("team_nav_flight_watcher")
        self.mission_kind = mission_kind
        self.route_file = route_file
        self.tracked_position = None
        self.tracked_heading_radians = None
        self.waypoints = []                      # (x, y) in the camera tracking frame, set at the start
        self.closest_approach_metres = []
        self.drone_state = None
        self.px4_position = None
        self.last_pose_target_seconds = None
        self.fastest_downward_command_metres_per_second = 0.0
        self.create_subscription(String, "/skybeta/state", self.on_drone_state, 10)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                                 self.on_px4_position, PX4_MESSAGE_SETTINGS)
        self.create_subscription(PoseStamped, "/skybeta/setpoint/pose", self.on_pose_target, 10)
        self.create_subscription(TwistStamped, "/skybeta/setpoint/twist", self.on_velocity_command, 10)
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self.on_tracking_report, 10)
        self.start_service = self.create_client(Trigger, "/simulator_only/start_run")

    # ---- What we hear ------------------------------------------------------------------------

    def on_drone_state(self, message):
        self.drone_state = json.loads(message.data)

    def on_px4_position(self, message):
        self.px4_position = message

    def on_pose_target(self, message):
        self.last_pose_target_seconds = time.monotonic()

    def on_velocity_command(self, message):
        downward = -float(message.twist.linear.z)
        self.fastest_downward_command_metres_per_second = max(self.fastest_downward_command_metres_per_second, downward)

    def on_tracking_report(self, message):
        point, quaternion = message.pose.pose.position, message.pose.pose.orientation
        self.tracked_position = (point.x, point.y)
        self.tracked_heading_radians = math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                                                  1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))
        for index, (east, north) in enumerate(self.waypoints):
            distance = math.hypot(point.x - east, point.y - north)
            self.closest_approach_metres[index] = min(self.closest_approach_metres[index], distance)

    def say(self, text):
        print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)

    def tracked_height_metres(self):
        return self.drone_state["odom_enu"][2]

    def simulator_is_ready(self):
        state = self.drone_state
        laser_ready = self.px4_position is not None and self.px4_position.z_valid
        return (state is not None and state["px4_link"] and state["landed"] is True
                and state["odom_enu"] is not None and state["local_enu"] is not None and laser_ready)

    def our_mission_is_sending_targets(self):
        last = self.last_pose_target_seconds
        return last is not None and time.monotonic() - last < OUR_TARGETS_COUNT_AS_LIVE_SECONDS

    # ---- Waiting -----------------------------------------------------------------------------

    def wait_until(self, is_finished, longest_wait_seconds, what_we_are_waiting_for):
        start_time = time.monotonic()
        while not is_finished():
            if time.monotonic() - start_time > longest_wait_seconds:
                raise TimeoutError(
                    f"Waited {longest_wait_seconds:.0f} seconds for {what_we_are_waiting_for}, and it did "
                    f"not happen. The last drone state was: {self.drone_state}. Our mission's own log is in "
                    "simulator/generated_files/last_team_nav_log.txt.")
            rclpy.spin_once(self, timeout_sec=1.0 / CHECKS_PER_SECOND)

    def wait_for_seconds(self, seconds):
        end_time = time.monotonic() + seconds
        self.wait_until(lambda: time.monotonic() > end_time, seconds + 1.0, "a timed wait")

    # ---- The flight, step by step ------------------------------------------------------------

    def run(self):
        self.refuse_to_run_outside_the_simulator()
        self.wait_for_the_simulator()
        self.wait_for_our_mission()
        self.press_start()
        self.watch_the_climb()
        self.watch_until_the_motors_are_off()
        return self.report()

    def refuse_to_run_outside_the_simulator(self):
        if not self.start_service.wait_for_service(timeout_sec=LONGEST_WAIT_FOR_SIMULATOR_SECONDS):
            raise TimeoutError(
                "The service /simulator_only/start_run did not appear, so this does not look like the "
                "simulator. Nothing was done. Use simulator/run_team_nav.sh to run this.")

    def wait_for_the_simulator(self):
        self.say("Waiting for PX4, the camera tracking and the laser to be ready.")
        self.wait_until(self.simulator_is_ready, LONGEST_WAIT_FOR_SIMULATOR_SECONDS, "the simulator to become ready")
        self.say("The simulator is ready. Letting the height estimate settle.")
        self.wait_for_seconds(SECONDS_TO_SETTLE_BEFORE_START)

    def wait_for_our_mission(self):
        self.wait_until(self.our_mission_is_sending_targets, LONGEST_WAIT_FOR_OUR_MISSION_SECONDS,
                        "our mission (team_nav) to send its ready target")
        self.say("Our mission is sending its ready target.")

    def remember_the_route(self):
        """The route's waypoints in the camera tracking frame, from where the drone sits and the way
        its nose points now, just as the mission works them out at takeoff."""
        with open(f"{MISSIONS_FOLDER}/{self.route_file}", encoding="utf-8") as opened:
            route = json.load(opened)["waypoints_forward_left_metres"]
        start_east, start_north = self.tracked_position
        cos_heading, sin_heading = math.cos(self.tracked_heading_radians), math.sin(self.tracked_heading_radians)
        self.waypoints = [(start_east + cos_heading * forward - sin_heading * left,
                           start_north + sin_heading * forward + cos_heading * left) for forward, left in route]
        self.closest_approach_metres = [math.inf] * len(self.waypoints)
        self.say(f"The route in {self.route_file} has {len(self.waypoints)} waypoints.")

    def press_start(self):
        if self.mission_kind == "route":
            self.remember_the_route()
        self.start_height_metres = self.tracked_height_metres()
        reply = self.start_service.call_async(Trigger.Request())
        self.wait_until(reply.done, 5.0, "the start service to reply")
        self.say("Start pressed: the pretend person arms the drone, then flips the switch to Position.")
        self.wait_until(lambda: self.drone_state["offboard_active"], LONGEST_WAIT_FOR_HAND_OVER_SECONDS,
                        "the hand-over to automatic flight (Offboard)")
        self.hand_over_seconds = time.monotonic()
        self.say("Hand-over: the drone is in automatic flight, flown by team_nav.")

    def height_climbed_metres(self):
        return self.tracked_height_metres() - self.start_height_metres

    def watch_the_climb(self):
        self.peak_height_metres = 0.0

        def at_cruise_height():
            self.peak_height_metres = max(self.peak_height_metres, self.height_climbed_metres())
            return self.height_climbed_metres() >= CRUISE_HEIGHT_METRES - CRUISE_HEIGHT_TOLERANCE_METRES

        self.wait_until(at_cruise_height, LONGEST_CLIMB_SECONDS, "the climb to cruise height")
        self.say(f"Climbed to {self.height_climbed_metres():.2f} m, "
                 f"{time.monotonic() - self.hand_over_seconds:.1f} s after the hand-over.")

    def watch_until_the_motors_are_off(self):
        self.landed_seconds = None

        def motors_are_off():
            self.peak_height_metres = max(self.peak_height_metres, self.height_climbed_metres())
            if self.landed_seconds is None and self.drone_state["landed"]:
                self.landed_seconds = time.monotonic()
                self.say(f"PX4 says landed, {self.landed_seconds - self.hand_over_seconds:.1f} s after the hand-over.")
            return self.drone_state["armed"] is False

        self.wait_until(motors_are_off, LONGEST_WAIT_FOR_MOTORS_OFF_SECONDS[self.mission_kind], "the motors to switch off")
        self.motors_off_seconds = time.monotonic()
        self.say(f"Motors off, {self.motors_off_seconds - self.hand_over_seconds:.1f} s after the hand-over.")

    def report(self):
        peak_ok = abs(self.peak_height_metres - CRUISE_HEIGHT_METRES) <= CRUISE_HEIGHT_TOLERANCE_METRES
        push_down_seen = self.fastest_downward_command_metres_per_second >= PUSH_DOWN_SEEN_AT_METRES_PER_SECOND
        if self.mission_kind == "route":
            return self.report_the_route(push_down_seen)
        self.say(f"RESULT: hand-over yes; peak height {self.peak_height_metres:.2f} m "
                 f"({'as expected' if peak_ok else 'NOT about ' + str(CRUISE_HEIGHT_METRES) + ' m'}); "
                 f"push-down {'seen' if push_down_seen else 'NOT seen'} (fastest downward command "
                 f"{self.fastest_downward_command_metres_per_second:.2f} m/s); motors off yes.")
        return peak_ok and push_down_seen

    def report_the_route(self, push_down_seen):
        for index, closest in enumerate(self.closest_approach_metres):
            self.say(f"waypoint {index + 1}: closest approach {closest:.2f} m")
        reached = [closest <= WAYPOINT_REACHED_METRES for closest in self.closest_approach_metres]
        last_reached = reached[-1]
        self.say(f"RESULT: reached {sum(reached)} of {len(reached)} waypoints within {WAYPOINT_REACHED_METRES} m; "
                 f"last waypoint {'REACHED' if last_reached else 'NOT reached'} (closest "
                 f"{self.closest_approach_metres[-1]:.2f} m); landed and motors off: yes; push-down "
                 f"{'seen' if push_down_seen else 'not seen'}.")
        return last_reached


def main():
    parser = argparse.ArgumentParser(description="Press start for team_nav in the simulator and watch it fly.")
    parser.add_argument("--mission", choices=["stages", "route"], default="stages")
    parser.add_argument("--route-file", default="route_waypoints.json",
                        help="the route file in drone_code/team_nav/missions that the mission flies")
    arguments = parser.parse_args()
    rclpy.init()
    watcher = TeamNavFlightWatcher(arguments.mission, arguments.route_file)
    try:
        everything_expected = watcher.run()
    except TimeoutError as problem:
        print(f"TEST FAILED: {problem}", flush=True)
        sys.exit(1)
    sys.exit(0 if everything_expected else 1)


if __name__ == "__main__":
    main()
