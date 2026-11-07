"""Pretends to be the laser sensor under the drone, which measures distance downward.

SIMULATOR ONLY.

What it does
------------
It takes the drone's true position, shoots one ray downward in the course model, adds a little
noise, and gives the reading to PX4, the same way the real sensor does. PX4 uses it as the
drone's height, so it matters a great deal.

Because the ray looks at the real course model, the height reading suddenly drops when the
drone passes over a boulder or the bottom edge of the window. This is what we want to test.

The physics inside PX4 does not know about any of this. It has a flat floor and nothing else.

How fast and how late the readings come
---------------------------------------
This copies what PX4's driver for the real sensor (a VL53L1X) does. The sensor collects light
for 0.2 seconds to make each reading, so it gives only 5 readings per second. A reading
describes where the drone was in the middle of that time, about 0.1 seconds before PX4
receives it. PX4 is told the lag is only 5 milliseconds (the setting EKF2_RNG_DELAY), so on
the real drone PX4 slightly misjudges when each height was measured. We copy that on purpose.

What it does not copy
---------------------
  - The real sensor sees a cone, not a thin ray. PX4's driver says the cone is 27 degrees wide
    and steps each reading around 15 small patches inside it. The organizers say the beam is
    only about 1 degree wide. Until that is settled, we use a thin ray.
  - How dark, shiny or brightly lit surfaces change the reading.
"""
from collections import deque

import numpy as np
import rclpy
from px4_msgs.msg import DistanceSensor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from course_simulator.course_model import CourseModel
from course_simulator.true_drone_state import DEFAULT_START_HEADING_DEGREES, TrueDroneState
from course_simulator.world import course_model_file_and_tables_folder

# From PX4 source code (src/drivers/distance_sensor/vl53l1x/vl53l1x.cpp). The driver asks the
# sensor for a new reading every 50 ms, and the sensor takes 200 ms to make one.
SECONDS_BETWEEN_DRIVER_CHECKS = 0.05
DRIVER_CHECKS_PER_READING = 4

# Follows from the two numbers above. We keep the last three positions, 50 ms apart, so the
# oldest is 100 ms old: the middle of the sensor's 200 ms of collecting light.
POSITIONS_KEPT = 3

# Estimated. The sensor is near the bottom of the frame, a few centimetres above the floor
# when the drone is on the ground. In the physics, the drone's centre sits exactly on the
# floor when landed, so we add this to the height. We need to measure the real value.
LASER_HEIGHT_ABOVE_FLOOR_WHEN_LANDED_METRES = 0.06

# From drone 16's setting EKF2_RNG_POS_Y, which says the laser is 2.7 cm left of centre.
LASER_DISTANCE_LEFT_OF_CENTRE_METRES = 0.027432

# Estimated. We use drone 16's setting EKF2_RNG_NOISE, which is the error PX4 is told to
# expect. Nobody has measured the real sensor's error.
READING_NOISE_METRES = 0.010058

# From PX4 source code: the driver runs the sensor in its long range mode and reports this
# range to PX4. Drone 16 reports the same range.
SHORTEST_READING_METRES = 0.0
LONGEST_READING_METRES = 4.0

# The value PX4 uses for a sensor that points straight down.
SENSOR_POINTS_DOWN = 25
SENSOR_TYPE_LASER = 0

# PX4 expects this delivery setting for the messages it receives.
PX4_INPUT_SETTINGS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class PretendBottomLaser(Node):

    def __init__(self):
        super().__init__("pretend_bottom_laser")
        self.declare_parameter("start_heading_degrees", DEFAULT_START_HEADING_DEGREES)
        self.true_state = TrueDroneState(self, self.get_parameter("start_heading_degrees").value)
        self.course = CourseModel(*course_model_file_and_tables_folder())
        self.random = np.random.default_rng(2)
        self.publisher = self.create_publisher(DistanceSensor, "/fmu/in/distance_sensor", PX4_INPUT_SETTINGS)

        # The newest distance is added on the right. The oldest drops off the left by itself.
        self.recent_distances_metres = deque(maxlen=POSITIONS_KEPT)
        self.checks_since_last_reading = 0
        self.create_timer(SECONDS_BETWEEN_DRIVER_CHECKS, self.on_driver_check)

    def on_driver_check(self):
        if not self.true_state.has_data:
            return
        self.recent_distances_metres.append(self.measure_distance_down_metres())

        self.checks_since_last_reading += 1
        if self.checks_since_last_reading < DRIVER_CHECKS_PER_READING:
            return
        self.checks_since_last_reading = 0

        # The oldest distance we keep is from the middle of the sensor's collecting time.
        self.publish_reading(self.recent_distances_metres[0])

    def measure_distance_down_metres(self):
        # The sensor is fixed to the frame, so it moves and tilts with the drone. The columns of
        # the body directions are the drone's forward, left and up directions.
        body_directions = self.true_state.body_directions_on_course()
        left_direction = body_directions[:, 1]
        sensor_position = (
            self.true_state.position_on_course_metres()
            + left_direction * LASER_DISTANCE_LEFT_OF_CENTRE_METRES
            + np.array([0.0, 0.0, LASER_HEIGHT_ABOVE_FLOOR_WHEN_LANDED_METRES])
        )
        beam_direction = -body_directions[:, 2]
        distances = self.course.cast_rays(
            sensor_position[np.newaxis], beam_direction[np.newaxis], LONGEST_READING_METRES
        )
        return float(distances[0])

    def publish_reading(self, distance_metres):
        if np.isfinite(distance_metres):
            distance_metres += self.random.normal(0.0, READING_NOISE_METRES)

        reading = DistanceSensor()
        reading.timestamp = self.get_clock().now().nanoseconds // 1000
        reading.min_distance = SHORTEST_READING_METRES
        reading.max_distance = LONGEST_READING_METRES
        # PX4 treats a value past the longest distance as "no valid reading".
        reading.current_distance = float(min(distance_metres, LONGEST_READING_METRES + 1.0))
        reading.variance = READING_NOISE_METRES ** 2
        reading.signal_quality = -1  # -1 means "this sensor does not report its quality".
        reading.type = SENSOR_TYPE_LASER
        reading.orientation = SENSOR_POINTS_DOWN
        self.publisher.publish(reading)


def main():
    rclpy.init()
    rclpy.spin(PretendBottomLaser())
