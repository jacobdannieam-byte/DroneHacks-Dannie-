"""Watches each simulated run and records what happened. SIMULATOR ONLY.

What it records
---------------
  - Each time a propeller touches an obstacle. The simulated drone does NOT bounce or crash,
    because PX4's physics does not know about walls. It flies straight through, and the
    touch is only written down. The team decided that any touch must stop the run, so the
    referee also announces each touch on /simulator_only/touches. The test flight lands as
    soon as it hears one, and run_test_flight.sh fails the test if the summary shows any touch.
  - Whether the drone went above the wall tops or left the course.
  - When each checkpoint was reached, and the finish time.
  - Each time the drone was above a boulder, and how far PX4's height estimate strayed from
    the true height (boulder_watch.py). run_test_flight.sh fails the test if the drone stayed
    above a boulder too long or the estimate went too far wrong. The estimate is judged in
    three parts: the takeoff climb straight above the launch mark has a looser limit (20 cm),
    because PX4 harmlessly trails the true height there; the rest of the flight keeps the 10 cm
    limit; and the time on the floor after touchdown, while the drone presses down to make the
    motors stop, is not counted. boulder_watch.py explains why.

Every sample also goes to a file, which plot_flight.py can turn into a picture of the run.

Where the checkpoints come from
-------------------------------
The order of the checkpoints comes from the mission file (drone_code/team_nav/missions/), and
the lines themselves from the simulator's world map (see world.py). A checkpoint is cleared at
the first moment when every propeller is past one of its lines, while the drone's centre is
level with that line rather than beyond one of its ends. The last checkpoint is landing with
every propeller inside the pad's outer ring.

THE CHECKPOINT LINES ARE A DRAFT. The rulebook describes them in words, and
course_map_tools/checkpoint_line_rules.py is our reading of those words. Check them against
the organizers' rules before relying on them.
"""
import csv
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from team_nav.mission import check_mission_fits_map, load_mission

from course_simulator.boulder_watch import BoulderWatch
from course_simulator.course_model import CourseModel
from course_simulator.paths import FOLDER_FOR_FLIGHT_RECORDS, RULEBOOK_MISSION_FILE
from course_simulator.true_drone_state import DEFAULT_START_HEADING_DEGREES, TrueDroneState
from course_simulator.world import course_model_file_and_tables_folder, load_world_map, world_map_file

CHECKS_PER_SECOND = 30.0

# Our own choice. Often enough that a flight which fails early still leaves a summary.
SECONDS_BETWEEN_SUMMARY_WRITES = 1.0

# Motor positions on the frame as (forward, left), in metres. From the organizers' settings
# file 07_airframe_geometry.params, measured on drone 20. All the drones use the same frame.
MOTOR_POSITIONS_METRES = (
    (0.08, 0.16), (0.08, -0.16), (-0.15, 0.14), (-0.15, -0.14),
)
# Assuming 7-inch propellers, which nobody has confirmed. Half of 7 inches is 8.89 cm.
PROPELLER_RADIUS_METRES = 0.0889
# Estimated height of the propellers above the floor when the drone is standing on the floor.
PROPELLER_HEIGHT_METRES = 0.10
POINTS_AROUND_EACH_PROPELLER = 8

# A propeller closer than this to an obstacle counts as touching it. It is about the accuracy
# of the course tables. It is not the width of the propeller's blade.
TOUCHING_DISTANCE_METRES = 0.01
# Contact that continues for less than this gap counts as one touch, not several.
SECONDS_OF_CLEARANCE_BEFORE_NEXT_TOUCH_COUNTS = 1.0

# From the rulebook: each touch adds this much to the finish time.
PENALTY_SECONDS_PER_TOUCH = 5.0

# We treat the drone as having left the launch mark once it is this high. Rule 7.5 starts the
# clock when it leaves. The exact height is our own choice.
HEIGHT_THAT_COUNTS_AS_TAKEN_OFF_METRES = 0.05


def whole_drone_is_past_line(line, propeller_points, drone_centre):
    """True if every propeller is on the line's far side, and the drone's centre is level with
    the line rather than beyond one of its ends."""
    along = (line.end_metres - line.start_metres) / np.linalg.norm(line.end_metres - line.start_metres)
    towards_far_side = np.array([-along[1], along[0]])
    if np.dot(line.far_side_point_metres - line.start_metres, towards_far_side) < 0:
        towards_far_side = -towards_far_side
    every_propeller_past = np.all((propeller_points[:, :2] - line.start_metres) @ towards_far_side > 0)
    centre_along_metres = np.dot(drone_centre[:2] - line.start_metres, along)
    centre_is_level_with_line = 0.0 <= centre_along_metres <= np.linalg.norm(line.end_metres - line.start_metres)
    return bool(every_propeller_past and centre_is_level_with_line)


def inside_landing_pad(world_map, propeller_points):
    distances = np.linalg.norm(propeller_points[:, :2] - world_map.landing_pad_centre_metres, axis=1)
    return bool(np.all(distances < world_map.landing_pad_outer_radius_metres))


def build_propeller_points_on_body():
    """Returns points around the edge of each propeller, in forward, left, up directions."""
    points = []
    for motor_forward, motor_left in MOTOR_POSITIONS_METRES:
        for point_number in range(POINTS_AROUND_EACH_PROPELLER):
            angle = 2.0 * np.pi * point_number / POINTS_AROUND_EACH_PROPELLER
            points.append((
                motor_forward + PROPELLER_RADIUS_METRES * np.cos(angle),
                motor_left + PROPELLER_RADIUS_METRES * np.sin(angle),
                PROPELLER_HEIGHT_METRES,
            ))
    return np.array(points)


class Referee(Node):

    def __init__(self):
        super().__init__("referee")
        self.declare_parameter("start_heading_degrees", DEFAULT_START_HEADING_DEGREES)
        self.true_state = TrueDroneState(self, self.get_parameter("start_heading_degrees").value)
        self.world_map = load_world_map()
        self.course_model_file, tables_folder = course_model_file_and_tables_folder()
        self.course = CourseModel(self.course_model_file, tables_folder)
        mission = load_mission(RULEBOOK_MISSION_FILE)
        check_mission_fits_map(mission, self.world_map)
        # For each checkpoint: its name, and the lines that clear it.
        self.checkpoints_in_order = []
        for checkpoint in mission.checkpoints:
            lines = [self.world_map.checkpoint_line_named(name) for name in checkpoint.line_names_in_order_of_preference]
            self.checkpoints_in_order.append((checkpoint.name, lines))
        self.propeller_points_on_body = build_propeller_points_on_body()
        self.boulder_watch = BoulderWatch(self, self.world_map)

        self.drone_state = {}
        self.takeoff_time_seconds = None
        self.finish_time_seconds = None
        self.landed_on_pad = False
        self.next_checkpoint_number = 0
        self.checkpoint_times = {}
        self.checkpoint_lines_used = {}
        self.touch_count = 0
        self.last_contact_time_seconds = None
        self.went_above_walls = False
        self.left_the_course = False
        self.first_px4_clock_seconds = None
        self.first_real_clock_seconds = None

        os.makedirs(FOLDER_FOR_FLIGHT_RECORDS, exist_ok=True)
        record_name = time.strftime("flight_%Y%m%d_%H%M%S")
        self.record_path = os.path.join(FOLDER_FOR_FLIGHT_RECORDS, f"{record_name}.csv")
        self.summary_path = os.path.join(FOLDER_FOR_FLIGHT_RECORDS, f"{record_name}_summary.json")
        self.record_file = open(self.record_path, "w", newline="", encoding="utf-8")
        self.record_writer = csv.writer(self.record_file)
        self.record_writer.writerow((
            "seconds", "course_x_metres", "course_y_metres", "height_metres", "heading_degrees",
            "nearest_obstacle_metres", "armed", "landed", "automatic_flight", "px4_estimated_height_metres",
        ))

        self.create_subscription(String, "/skybeta/state", self.on_drone_state, 10)
        # SIMULATOR ONLY. The real drone has no referee, so real flight code can't use this.
        self.touch_announcer = self.create_publisher(String, "/simulator_only/touches", 10)
        self.create_timer(1.0 / CHECKS_PER_SECOND, self.check_the_drone)
        # run_test_flight.sh prints the newest summary when a flight ends. So every flight must
        # write its own summary early, even one that fails straight away. Otherwise a failed
        # flight would show the summary of the flight before it.
        self.create_timer(SECONDS_BETWEEN_SUMMARY_WRITES, self.write_summary)

    def on_drone_state(self, message):
        self.drone_state = json.loads(message.data)

    def now_in_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def px4_clock_speed(self):
        """Returns how fast PX4's simulated clock runs compared with real time. 1.0 is perfect.

        If this is far from 1.0, the drone is flying in slow or fast motion compared with the
        pretend sensors, so the flight's results are less trustworthy. See start_px4.sh.

        Returns None until PX4 has sent enough data to measure this.
        """
        if self.first_real_clock_seconds is None:
            return None
        real_seconds_passed = self.now_in_seconds() - self.first_real_clock_seconds
        if real_seconds_passed <= 0.0:
            return None
        px4_seconds_passed = self.true_state.px4_clock_seconds - self.first_px4_clock_seconds
        return round(px4_seconds_passed / real_seconds_passed, 3)

    def check_the_drone(self):
        if not self.true_state.has_data:
            return
        if self.first_px4_clock_seconds is None:
            self.first_px4_clock_seconds = self.true_state.px4_clock_seconds
            self.first_real_clock_seconds = self.now_in_seconds()
        position = self.true_state.position_on_course_metres()
        propeller_points = self.true_state.body_points_on_course_metres(self.propeller_points_on_body)
        nearest_obstacle_metres = float(
            np.min(self.course.distance_to_nearest_obstacle_metres(propeller_points))
        )

        self.check_for_takeoff(position)
        self.check_for_touch(position, nearest_obstacle_metres)
        self.check_limits(position)
        self.check_checkpoints(propeller_points, position)
        self.check_for_finish(propeller_points)
        self.boulder_watch.check(position, self.now_in_seconds(), self.seconds_flying())
        self.write_sample(position, nearest_obstacle_metres)

    def check_for_takeoff(self, position):
        if self.takeoff_time_seconds is None and position[2] > HEIGHT_THAT_COUNTS_AS_TAKEN_OFF_METRES:
            self.takeoff_time_seconds = self.now_in_seconds()
            self.get_logger().info("The drone has left the launch mark. The clock is running.")

    def seconds_since_takeoff(self):
        return self.now_in_seconds() - self.takeoff_time_seconds

    def seconds_flying(self):
        """Seconds since takeoff, or None before takeoff and after the motors have stopped."""
        if self.takeoff_time_seconds is None or self.finish_time_seconds is not None:
            return None
        return self.seconds_since_takeoff()

    def check_for_touch(self, position, nearest_obstacle_metres):
        if nearest_obstacle_metres >= TOUCHING_DISTANCE_METRES:
            return
        now = self.now_in_seconds()
        contact_is_new = (
            self.last_contact_time_seconds is None
            or now - self.last_contact_time_seconds > SECONDS_OF_CLEARANCE_BEFORE_NEXT_TOUCH_COUNTS
        )
        self.last_contact_time_seconds = now
        if contact_is_new:
            self.touch_count += 1
            description = (
                f"Touch number {self.touch_count}, with the drone's centre at ({position[0]:.2f}, "
                f"{position[1]:.2f}) and {position[2]:.2f} m up. In a real run this adds "
                f"{PENALTY_SECONDS_PER_TOUCH:.0f} seconds, and it could also end the run."
            )
            self.get_logger().warning(description)
            self.touch_announcer.publish(String(data=description))
            self.write_summary()

    def check_limits(self, position):
        if position[2] > self.world_map.wall_top_height_metres and not self.went_above_walls:
            self.went_above_walls = True
            self.get_logger().warning("The drone went above the wall tops.")
        outside_the_net = (
            np.any(position[:2] < self.world_map.lowest_corner_metres)
            or np.any(position[:2] > self.world_map.highest_corner_metres)
        )
        if outside_the_net and not self.left_the_course:
            self.left_the_course = True
            self.get_logger().warning("The drone left the course, which would hit the net.")

    def check_checkpoints(self, propeller_points, position):
        if self.takeoff_time_seconds is None:
            return
        if self.next_checkpoint_number >= len(self.checkpoints_in_order):
            return
        name, lines = self.checkpoints_in_order[self.next_checkpoint_number]
        for line in lines:
            if whole_drone_is_past_line(line, propeller_points, position):
                self.checkpoint_times[name] = round(self.seconds_since_takeoff(), 2)
                self.checkpoint_lines_used[name] = line.name
                self.next_checkpoint_number += 1
                self.get_logger().info(f"Checkpoint reached: {name}, through '{line.name}', at {self.checkpoint_times[name]} seconds.")
                self.write_summary()
                return

    def check_for_finish(self, propeller_points):
        if self.finish_time_seconds is not None or self.takeoff_time_seconds is None:
            return
        # The state message starts empty, so a missing "armed" must not read as motors off.
        motors_are_off = self.drone_state.get("armed") is False
        if not motors_are_off:
            return
        self.finish_time_seconds = round(self.seconds_since_takeoff(), 2)
        self.landed_on_pad = inside_landing_pad(self.world_map, propeller_points)
        self.get_logger().info(
            f"The motors switched off at {self.finish_time_seconds} seconds. "
            f"{'The whole drone is inside the pad.' if self.landed_on_pad else 'The drone is NOT inside the pad.'}"
        )
        self.write_summary()

    def write_sample(self, position, nearest_obstacle_metres):
        self.record_writer.writerow((
            round(self.now_in_seconds(), 3),
            round(float(position[0]), 4), round(float(position[1]), 4), round(float(position[2]), 4),
            round(self.true_state.heading_on_course_degrees(), 1),
            round(nearest_obstacle_metres, 4),
            self.drone_state.get("armed"), self.drone_state.get("landed"),
            self.drone_state.get("offboard_active"),
            self.boulder_watch.px4_estimated_height_metres,
        ))
        self.record_file.flush()

    def write_summary(self):
        summary = {
            "flight_record": os.path.basename(self.record_path),
            "world_map_file": world_map_file(),
            "course_model_file": self.course_model_file,
            "checkpoint_seconds": self.checkpoint_times,
            "checkpoint_lines_used": self.checkpoint_lines_used,
            "motors_off_seconds": self.finish_time_seconds,
            "landed_on_pad": self.landed_on_pad,
            "touches": self.touch_count,
            "penalty_seconds": self.touch_count * PENALTY_SECONDS_PER_TOUCH,
            "went_above_walls": self.went_above_walls,
            "left_the_course": self.left_the_course,
            "px4_clock_speed_compared_with_real_time": self.px4_clock_speed(),
        }
        summary.update(self.boulder_watch.summary())
        with open(self.summary_path, "w", encoding="utf-8") as summary_file:
            json.dump(summary, summary_file, indent=2)


def main():
    rclpy.init()
    rclpy.spin(Referee())
