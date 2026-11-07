"""Pretends to be the laser sensor under the drone, which measures distance downward.

SIMULATOR ONLY.

What it does
------------
Many times per second it takes the drone's true position, shoots one ray downward in the
course model, adds a little noise, and gives the reading to PX4, the same way the real
sensor does. PX4 uses it as the drone's height, so it matters a great deal.

Because the ray looks at the real course model, the height reading suddenly drops when the
drone passes over a boulder or the bottom edge of the window. This is what we want to test.

The physics inside PX4 does not know about any of this. It has a flat floor and nothing else.
"""
import numpy as np
import rclpy
from px4_msgs.msg import DistanceSensor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from course_simulator.course_model import CourseModel
from course_simulator.paths import COURSE_MODEL_FILE, FOLDER_FOR_SAVED_TABLES
from course_simulator.true_drone_state import DEFAULT_START_HEADING_DEGREES, TrueDroneState

# Estimated. We have not measured how often the real laser reports.
READINGS_PER_SECOND = 50.0

# Estimated. The sensor is near the bottom of the frame, a few centimetres above the floor
# when the drone is on the ground. In the physics, the drone's centre sits exactly on the
# floor when landed, so we add this to the height. We need to measure the real value.
LASER_HEIGHT_ABOVE_FLOOR_WHEN_LANDED_METRES = 0.06

# Read from the drone's setting EKF2_RNG_NOISE.
READING_NOISE_METRES = 0.01

# Estimated from the sensor's data sheet. Readings outside this range are not valid.
SHORTEST_READING_METRES = 0.04
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
        self.course = CourseModel(COURSE_MODEL_FILE, FOLDER_FOR_SAVED_TABLES)
        self.random = np.random.default_rng(2)
        self.publisher = self.create_publisher(DistanceSensor, "/fmu/in/distance_sensor", PX4_INPUT_SETTINGS)
        self.create_timer(1.0 / READINGS_PER_SECOND, self.publish_reading)

    def measure_distance_down_metres(self):
        sensor_position = self.true_state.position_on_course_metres() + np.array(
            [0.0, 0.0, LASER_HEIGHT_ABOVE_FLOOR_WHEN_LANDED_METRES]
        )
        # The sensor is fixed to the frame, so it tilts with the drone. The third column of
        # the body directions is the drone's "up" direction, so "down" is its opposite.
        beam_direction = -self.true_state.body_directions_on_course()[:, 2]
        distances = self.course.cast_rays(
            sensor_position[np.newaxis], beam_direction[np.newaxis], LONGEST_READING_METRES
        )
        return float(distances[0])

    def publish_reading(self):
        if not self.true_state.has_data:
            return
        distance_metres = self.measure_distance_down_metres()
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
