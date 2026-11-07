"""The ROS 2 program that counts FAST corners in the left camera's picture, twice a second, for the
mission's pre-start check (prestart_check.py) and for drone_code/check_drone_16.sh.

How it fits: team_nav.launch.py starts it as its own program (launch argument corner_count,
default true), so if it ever crashes, the mission and the flight carry on: the mission then simply
has no corner count, which never blocks a start on its own. It only reads pictures and publishes
one number; it sends nothing to the drone.

Topics:
  hears  image_topic (parameter), by default /oak/left/image_raw, the left camera of the OAK-D
         stereo camera (from the organizers' guide, its list of topics to record)
  sends  /team/left_corner_count   std_msgs/Int32: the FAST corner count (fast_corners.py), counted
         on the picture shrunk to half size (1024 x 640 becomes 512 x 320)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from .camera_images import grey_from_image_message, shrink_by_whole_factor
from .fast_corners import count_fast_corners

# From the organizers' guide (section 5, recording a bag): the left camera's picture.
DEFAULT_IMAGE_TOPIC = "/oak/left/image_raw"
CORNER_COUNT_TOPIC = "/team/left_corner_count"
# Our own choice: twice a second is plenty for a check made while the drone waits.
COUNTS_PER_SECOND = 2.0
# Our own choice: half size, as the depth plan uses (docs/ARCHITECTURE.md, appendix).
SHRINK_FACTOR = 2
# Our own choice, OpenCV's usual FAST threshold: brightness differences above 20 out of 255.
FAST_THRESHOLD = 20


class CornerCountNode(Node):
    def __init__(self):
        super().__init__("team_corner_count")
        image_topic = self.declare_parameter("image_topic", DEFAULT_IMAGE_TOPIC).value
        self.latest_picture = None
        self.counted_picture = None
        self.warned = False
        latest_only = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, image_topic, self.on_picture, latest_only)
        self.publisher = self.create_publisher(Int32, CORNER_COUNT_TOPIC, 10)
        self.create_timer(1.0 / COUNTS_PER_SECOND, self.count_latest_picture)
        self.get_logger().info(f"Counting FAST corners in {image_topic}, twice a second, on {CORNER_COUNT_TOPIC}.")

    def on_picture(self, message):
        self.latest_picture = message

    def count_latest_picture(self):
        message = self.latest_picture
        if message is None or message is self.counted_picture:
            return
        self.counted_picture = message
        try:
            grey = shrink_by_whole_factor(grey_from_image_message(message), SHRINK_FACTOR)
        except ValueError as problem:
            if not self.warned:
                self.warned = True
                self.get_logger().warn(str(problem))
            return
        count = Int32()
        count.data = count_fast_corners(grey, FAST_THRESHOLD)
        self.publisher.publish(count)


def main(args=None):
    rclpy.init(args=args)
    node = CornerCountNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
