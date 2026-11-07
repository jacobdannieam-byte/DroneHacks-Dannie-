"""Tells the pretend sensors where the drone truly is on the course.

SIMULATOR ONLY. A real drone has no way of knowing its true position.

Where the true position comes from
----------------------------------
PX4's built-in physics works out the drone's true position and tilt, and shares them
on two messages that we added (see px4_changes/let_simulator_read_true_position.py).
PX4 does not know about the course, though. It reports the position relative to where
the drone started, using its own directions. So this file does the conversion.

The three sets of directions used here
--------------------------------------
Each set of directions (a "frame") is just an agreed meaning for x, y and z.

  - PX4 "NED": x points North, y points East, z points Down. PX4's physics always
    starts with the drone's nose pointing North.
  - "Start frame": x is the way the nose pointed at takeoff, y is to the drone's left
    at takeoff, z is up. The real camera tracking reports positions in this frame,
    because it only knows where it started.
  - "Course frame": the organizers' 3D model. x and y are along the floor, and z is
    height above the floor.

"Body" means directions fixed to the drone, which move and tilt with it: forward,
left, up.

We can place the simulated drone facing any direction on the launch mark, because
PX4's physics has a flat floor and no wind, so nothing in it depends on the compass
direction. The course is simply placed around it, rotated to match.
"""
import math

import numpy as np
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition

# The centre of the launch mark, measured from the "Launch mark" part in the course model.
LAUNCH_MARK_POSITION_METRES = np.array([-2.5145, 0.914, 0.0])

# Which way the drone faces at the start, as an angle measured from the course's x
# axis, turning towards the y axis. 90 degrees faces along y, which is straight down
# the first corridor. This is our assumption. The rules do not say which way it faces.
DEFAULT_START_HEADING_DEGREES = 90.0

# PX4 publishes these messages with this delivery setting, and a listener must match.
PX4_MESSAGE_SETTINGS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

TRUE_POSITION_TOPIC = "/fmu/out/vehicle_local_position_groundtruth"
TRUE_ATTITUDE_TOPIC = "/fmu/out/vehicle_attitude_groundtruth"


class TrueDroneState:
    """Listens to PX4's physics and always holds the latest true state of the drone."""

    def __init__(self, ros_node, start_heading_degrees=DEFAULT_START_HEADING_DEGREES):
        heading_radians = math.radians(start_heading_degrees)
        self._turn_start_frame_to_course_frame = np.array([
            [math.cos(heading_radians), -math.sin(heading_radians), 0.0],
            [math.sin(heading_radians), math.cos(heading_radians), 0.0],
            [0.0, 0.0, 1.0],
        ])

        self.has_data = False
        self.px4_clock_seconds = 0.0
        self.position_in_start_frame_metres = np.zeros(3)
        self.velocity_in_start_frame_metres_per_second = np.zeros(3)
        # The rotation, as a quaternion (w, x, y, z), that turns body directions into
        # start frame directions. A quaternion is a compact way to store a 3D rotation.
        self.body_rotation_in_start_frame = np.array([1.0, 0.0, 0.0, 0.0])

        ros_node.create_subscription(
            VehicleLocalPosition, TRUE_POSITION_TOPIC, self._on_true_position, PX4_MESSAGE_SETTINGS
        )
        ros_node.create_subscription(
            VehicleAttitude, TRUE_ATTITUDE_TOPIC, self._on_true_attitude, PX4_MESSAGE_SETTINGS
        )

    def _on_true_position(self, message):
        self.px4_clock_seconds = message.timestamp * 1e-6  # PX4 gives this in microseconds.
        # North is the nose direction at the start, East is to the right of it, and
        # Down is the opposite of up.
        self.position_in_start_frame_metres = np.array([message.x, -message.y, -message.z])
        self.velocity_in_start_frame_metres_per_second = np.array(
            [message.vx, -message.vy, -message.vz]
        )
        self.has_data = True

    def _on_true_attitude(self, message):
        # PX4 gives a rotation from forward-right-down directions to north-east-down.
        # Both of our frames differ from those by the same half turn about the x axis.
        # Doing that half turn on both sides of a rotation keeps its size and flips the
        # y and z parts of its axis, which is why only two signs change here.
        w, x, y, z = message.q
        self.body_rotation_in_start_frame = np.array([w, x, -y, -z])

    def position_on_course_metres(self):
        """Returns where the drone's centre truly is, in the course model's frame."""
        return (
            LAUNCH_MARK_POSITION_METRES
            + self._turn_start_frame_to_course_frame @ self.position_in_start_frame_metres
        )

    def body_directions_on_course(self):
        """Returns a 3x3 matrix whose columns are the drone's forward, left and up
        directions, measured in the course model's frame."""
        return self._turn_start_frame_to_course_frame @ rotation_matrix_from_quaternion(
            self.body_rotation_in_start_frame
        )

    def body_points_on_course_metres(self, points_on_the_body_metres):
        """Converts points fixed to the drone (forward, left, up) into course positions."""
        return (
            self.position_on_course_metres()
            + np.asarray(points_on_the_body_metres) @ self.body_directions_on_course().T
        )


def rotation_matrix_from_quaternion(quaternion):
    """The standard conversion from a quaternion (w, x, y, z) to a rotation matrix."""
    w, x, y, z = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
