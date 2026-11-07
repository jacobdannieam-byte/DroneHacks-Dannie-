"""Pretends to be the distance measurement that we will calculate from the front cameras.

SIMULATOR ONLY. The topic name starts with /simulator_only/ because this does not exist on
the drone. On the drone, our own "distance program" must produce these points from the two
camera pictures. This file lets us test the programs that use the points before that exists.

What it does
------------
Ten times per second, it shoots a grid of rays out of the front of the drone into the course
model, like the pixels of a low-resolution camera. Each ray gives one point. It then damages
the result in the two ways real stereo cameras are imperfect:

  - Errors grow with the square of the distance. This is the basic geometry of two cameras
    that sit close together.
  - Some points are simply missing, because a plain wall gives the two cameras nothing to match.

It publishes the points in the drone's own directions: forward, left, up, in metres.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from course_simulator.course_model import CourseModel
from course_simulator.paths import COURSE_MODEL_FILE, FOLDER_FOR_SAVED_TABLES
from course_simulator.true_drone_state import DEFAULT_START_HEADING_DEGREES, TrueDroneState

# Our own choice, to keep the simulator light on the Mac. We have not measured how often the real
# depth calculation will run on the drone.
MEASUREMENTS_PER_SECOND = 10.0

# How many rays we shoot. The real plan uses 512 x 320 pixels, but a grid this size gives
# about the 2000 points that the plan uses, and it runs quickly on the Mac.
RAYS_ACROSS = 64
RAYS_DOWN = 40

# The real camera, read from drone 16: 1024 x 640 pictures, with a focal length of 634.5
# pixels. Together these fix how wide the view is.
REAL_PICTURE_WIDTH_PIXELS = 1024.0
REAL_PICTURE_HEIGHT_PIXELS = 640.0
REAL_FOCAL_LENGTH_PIXELS = 634.5

# From the organizers' launch file: the camera is 10 cm in front of the drone's centre.
CAMERA_POSITION_ON_BODY_METRES = np.array([0.10, 0.0, 0.0])

# Estimated from the architecture plan: 4 cm of error at 1 m, growing as distance squared.
DEPTH_ERROR_METRES_AT_ONE_METRE = 0.04

# Estimated. We have not seen the real walls. Change this after seeing a photo of them.
FRACTION_OF_POINTS_MISSING = 0.3

# Our own choice. The course is about 6 m wide, and at this distance the error above has grown
# to about 1.4 m.
LONGEST_RANGE_METRES = 6.0

# Estimated. Two cameras can't match up pictures of something very close to them, so there is no
# measurement below this distance. This also skips rays that start inside the floor, which
# happens while the simulated drone is on the ground, because its centre is then at floor level.
SHORTEST_RANGE_METRES = 0.20

# Our own choice of layout for the message: each point is three 4-byte decimal numbers, for its
# forward, left and up distances.
BYTES_PER_NUMBER = 4
NUMBERS_PER_POINT = 3


def build_ray_directions_on_body():
    """Returns one unit-length direction per ray, in the drone's forward, left, up directions."""
    half_width = REAL_PICTURE_WIDTH_PIXELS / 2.0 / REAL_FOCAL_LENGTH_PIXELS
    half_height = REAL_PICTURE_HEIGHT_PIXELS / 2.0 / REAL_FOCAL_LENGTH_PIXELS
    leftward_slopes = np.linspace(half_width, -half_width, RAYS_ACROSS)
    upward_slopes = np.linspace(half_height, -half_height, RAYS_DOWN)
    leftward, upward = np.meshgrid(leftward_slopes, upward_slopes)
    directions = np.stack([np.ones_like(leftward), leftward, upward], axis=-1).reshape(-1, 3)
    return directions / np.linalg.norm(directions, axis=1, keepdims=True)


class PretendDepthCamera(Node):

    def __init__(self):
        super().__init__("pretend_depth_camera")
        self.declare_parameter("start_heading_degrees", DEFAULT_START_HEADING_DEGREES)
        self.true_state = TrueDroneState(self, self.get_parameter("start_heading_degrees").value)
        self.course = CourseModel(COURSE_MODEL_FILE, FOLDER_FOR_SAVED_TABLES)
        self.random = np.random.default_rng(3)
        self.ray_directions_on_body = build_ray_directions_on_body()
        self.publisher = self.create_publisher(PointCloud2, "/simulator_only/depth_points", 10)
        self.create_timer(1.0 / MEASUREMENTS_PER_SECOND, self.publish_points)

    def measure_points_on_body(self):
        camera_position = self.true_state.body_points_on_course_metres(CAMERA_POSITION_ON_BODY_METRES)
        body_directions = self.true_state.body_directions_on_course()
        ray_directions_on_course = self.ray_directions_on_body @ body_directions.T
        ray_starts = np.tile(camera_position, (len(ray_directions_on_course), 1))
        ray_lengths = self.course.cast_rays(ray_starts, ray_directions_on_course, LONGEST_RANGE_METRES)

        ray_gave_a_distance = np.isfinite(ray_lengths) & (ray_lengths >= SHORTEST_RANGE_METRES)
        ray_was_not_lost = self.random.random(len(ray_lengths)) > FRACTION_OF_POINTS_MISSING
        keep = ray_gave_a_distance & ray_was_not_lost

        points = self.ray_directions_on_body[keep] * ray_lengths[keep, np.newaxis]
        forward_distance = points[:, 0]
        depth_error = self.random.normal(0.0, 1.0, len(points)) * (
            DEPTH_ERROR_METRES_AT_ONE_METRE * forward_distance ** 2
        )
        # A stereo camera's error lies along the line of sight, so we scale the whole point.
        points *= (1.0 + depth_error / forward_distance)[:, np.newaxis]
        return points + CAMERA_POSITION_ON_BODY_METRES

    def publish_points(self):
        if not self.true_state.has_data:
            return
        points = self.measure_points_on_body().astype(np.float32)

        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name=name, offset=index * BYTES_PER_NUMBER, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(("x", "y", "z"))
        ]
        message.is_bigendian = False
        message.point_step = BYTES_PER_NUMBER * NUMBERS_PER_POINT
        message.row_step = message.point_step * len(points)
        message.data = points.tobytes()
        message.is_dense = True
        self.publisher.publish(message)


def main():
    rclpy.init()
    rclpy.spin(PretendDepthCamera())
