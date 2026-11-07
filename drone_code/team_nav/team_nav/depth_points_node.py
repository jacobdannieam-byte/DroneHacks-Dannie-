"""The ROS 2 program that makes depth points from the OAK-D's two front cameras on drone 16, about
10 times a second.

How it fits: team_nav.launch.py starts it as its own program (launch argument depth_points,
default true), so if it crashes, or OpenCV is missing, the mission and the flight carry on. It
publishes in exactly the simulator's pretend depth camera's format, so the drift correction
(drift_correction:=true, depth_topic:=/team/depth_points) works the same on the drone. The depth
itself is stereo_depth.py; drone_code/depth_check_16.sh checks it against a tape measure.

Topics (from the organizers' guide, its list of topics to record; camera_info is the standard
companion topic of each picture, and every name is a launch argument):
  hears  /oak/left/image_raw, /oak/right/image_raw       the two pictures
         /oak/left/camera_info, /oak/right/camera_info   each camera's lens numbers, read at runtime
  sends  /team/depth_points   sensor_msgs/PointCloud2, float32 x, y, z = forward, left, up in metres
                              from the drone's centre, frame base_link
It sends nothing to the drone.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from .camera_images import grey_from_image_message
from .stereo_depth import CameraLens, DepthSettings, StereoDepth

DEPTH_POINTS_TOPIC = "/team/depth_points"
# From the team: about 10 times a second.
CLOUDS_PER_SECOND = 10.0
# Our own choice: the two pictures of a pair carry the same timestamp; allow this much difference.
PAIR_STAMP_TOLERANCE_SECONDS = 0.005
# The same frame name as the simulator's pretend depth camera.
FRAME_ID = "base_link"
BYTES_PER_POINT = 12


def pack_points(points):
    """The points as a PointCloud2's fields, point_step and data: x, y, z float32, little-endian."""
    fields = [PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
              for index, name in enumerate(("x", "y", "z"))]
    return fields, BYTES_PER_POINT, points.astype("<f4").tobytes()


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class DepthPointsNode(Node):
    def __init__(self, cv2):
        super().__init__("team_depth_points")
        self.cv2 = cv2
        self.settings = DepthSettings(
            camera_forward_metres=float(self.declare_parameter("camera_forward_metres", 0.116).value),
            camera_left_metres=float(self.declare_parameter("camera_left_metres", 0.037).value),
            camera_up_metres=float(self.declare_parameter("camera_up_metres", 0.029).value))
        topics = {name: self.declare_parameter(name, default).value for name, default in (
            ("left_image_topic", "/oak/left/image_raw"), ("right_image_topic", "/oak/right/image_raw"),
            ("left_camera_info_topic", "/oak/left/camera_info"), ("right_camera_info_topic", "/oak/right/camera_info"))}
        self.latest = {"left": None, "right": None}
        self.lenses = {"left": None, "right": None}
        self.stereo = None
        self.last_pair_stamp = None
        self.warned = set()
        latest_only = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(Image, topics["left_image_topic"], lambda m: self.on_picture("left", m), latest_only)
        self.create_subscription(Image, topics["right_image_topic"], lambda m: self.on_picture("right", m), latest_only)
        self.create_subscription(CameraInfo, topics["left_camera_info_topic"], lambda m: self.on_lens("left", m), latest_only)
        self.create_subscription(CameraInfo, topics["right_camera_info_topic"], lambda m: self.on_lens("right", m), latest_only)
        self.publisher = self.create_publisher(PointCloud2, DEPTH_POINTS_TOPIC, 2)
        self.create_timer(1.0 / CLOUDS_PER_SECOND, self.make_points)
        self.get_logger().info(f"Depth points from {topics['left_image_topic']} and {topics['right_image_topic']} "
                               f"on {DEPTH_POINTS_TOPIC}; waiting for camera_info.")

    def warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)

    def on_picture(self, side, message):
        self.latest[side] = message

    def on_lens(self, side, message):
        if self.lenses[side] is None:
            self.lenses[side] = CameraLens.from_camera_info(message)

    def ready_stereo(self):
        if self.stereo is None and self.lenses["left"] is not None and self.lenses["right"] is not None:
            self.stereo = StereoDepth(self.cv2, self.lenses["left"], self.lenses["right"], self.settings)
            self.get_logger().info(f"Camera numbers read from camera_info: {self.stereo.describe()}.")
        return self.stereo is not None

    def make_points(self):
        left, right = self.latest["left"], self.latest["right"]
        if left is None or right is None or not self.ready_stereo():
            return
        if abs(stamp_seconds(left) - stamp_seconds(right)) > PAIR_STAMP_TOLERANCE_SECONDS:
            return                                       # not a pair yet: wait for the matching picture
        if self.last_pair_stamp == stamp_seconds(left):
            return                                       # this pair is done already
        self.last_pair_stamp = stamp_seconds(left)
        try:
            left_grey, right_grey = grey_from_image_message(left), grey_from_image_message(right)
        except ValueError as problem:
            self.warn_once("picture", str(problem))
            return
        if left_grey.shape != (self.lenses["left"].height, self.lenses["left"].width):
            self.warn_once("size", f"The pictures are {left_grey.shape[1]} x {left_grey.shape[0]}, but camera_info "
                                   f"says {self.lenses['left'].width} x {self.lenses['left'].height}; skipping them.")
            return
        self.publish(left.header.stamp, self.stereo.points(left_grey, right_grey))

    def publish(self, stamp, points):
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = FRAME_ID
        cloud.height, cloud.width = 1, len(points)
        cloud.fields, cloud.point_step, cloud.data = pack_points(points)
        cloud.row_step = cloud.point_step * len(points)
        cloud.is_bigendian = False
        cloud.is_dense = True
        self.publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    try:
        import cv2
    except ImportError:
        node = Node("team_depth_points")
        node.get_logger().error("OpenCV (python3-opencv) is not installed in the stack, so there are no depth points. "
                                "Flight is not affected. To add it, put 'RUN apt-get install -y python3-opencv' in "
                                "~/team_ws/Dockerfile and run host/run.sh build (organizers' guide, section 5).")
        node.destroy_node()
        rclpy.shutdown()
        return
    node = DepthPointsNode(cv2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
