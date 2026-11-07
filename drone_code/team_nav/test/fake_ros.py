"""Stand-ins for the few parts of ROS 2 that mission_node.py uses, so its wiring can be tested on a
laptop without ROS installed. Test code only: it is never installed or run on the drone.

install() puts these stand-ins where Python looks for rclpy and the message packages. Each topic a
node subscribes to or publishes on is recorded, so a test can deliver a message straight to the
node's callback and read back what it published.
"""
import math
import sys
import types


class Clock:
    """The time every stand-in node reads. Tests set seconds directly."""
    seconds = 0.0

    def now(self):
        return Time(Clock.seconds)


class Time:
    def __init__(self, seconds):
        self.nanoseconds = int(round(seconds * 1e9))

    def to_msg(self):
        stamp = types.SimpleNamespace()
        stamp.sec = self.nanoseconds // 1_000_000_000
        stamp.nanosec = self.nanoseconds % 1_000_000_000
        return stamp


class Parameter:
    def __init__(self, value):
        self.value = value


class Publisher:
    def __init__(self, topic, message_type):
        self.topic = topic
        self.message_type = message_type
        self.sent = []

    def publish(self, message):
        self.sent.append(message)


class Logger:
    def __init__(self, lines):
        self.lines = lines

    def info(self, text):
        self.lines.append(("info", text))

    def warn(self, text):
        self.lines.append(("warn", text))


class Node:
    parameter_overrides = {}                     # tests set ROS parameters here before building a node

    def __init__(self, name):
        self.name = name
        self.subscriptions = {}                  # topic -> (message type, callback, quality of service)
        self.publishers = {}                     # topic -> Publisher
        self.timers = []                         # [(period seconds, callback)]
        self.log_lines = []
        self.parameters = {}

    def declare_parameter(self, name, default):
        value = Node.parameter_overrides.get(name, default)
        self.parameters[name] = value
        return Parameter(value)

    def create_subscription(self, message_type, topic, callback, quality_of_service):
        self.subscriptions[topic] = (message_type, callback, quality_of_service)

    def create_publisher(self, message_type, topic, quality_of_service):
        publisher = Publisher(topic, message_type)
        self.publishers[topic] = publisher
        return publisher

    def create_timer(self, period_seconds, callback):
        self.timers.append((period_seconds, callback))

    def get_clock(self):
        return Clock()

    def get_logger(self):
        return Logger(self.log_lines)

    def destroy_node(self):
        pass


# ---------------------------------------------------------------- message stand-ins
class Vector3:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


class Pose:
    def __init__(self):
        self.position = Vector3()
        self.orientation = Quaternion()


class PoseStamped:
    def __init__(self):
        self.header = Header()
        self.pose = Pose()


class Twist:
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class TwistStamped:
    def __init__(self):
        self.header = Header()
        self.twist = Twist()


class PoseWithCovariance:
    def __init__(self):
        self.pose = Pose()


class Odometry:
    def __init__(self):
        self.header = Header()
        self.pose = PoseWithCovariance()


class String:
    def __init__(self, data=""):
        self.data = data


class VehicleLocalPosition:
    def __init__(self):
        self.dist_bottom = math.nan
        self.dist_bottom_valid = False


class QoSProfile:
    def __init__(self, **settings):
        self.settings = settings


class Policy:
    BEST_EFFORT = "best_effort"
    RELIABLE = "reliable"
    TRANSIENT_LOCAL = "transient_local"
    VOLATILE = "volatile"
    KEEP_LAST = "keep_last"


def module(name, **contents):
    made = types.ModuleType(name)
    for key, value in contents.items():
        setattr(made, key, value)
    return made


STAND_IN_MODULES = ("rclpy", "rclpy.node", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg", "nav_msgs",
                    "nav_msgs.msg", "std_msgs", "std_msgs.msg", "px4_msgs", "px4_msgs.msg")


def uninstall():
    for name in STAND_IN_MODULES:
        sys.modules.pop(name, None)


def install():
    sys.modules["rclpy"] = module("rclpy", init=lambda args=None: None, spin=lambda node: None,
                                  ok=lambda: False, shutdown=lambda: None)
    sys.modules["rclpy.node"] = module("rclpy.node", Node=Node)
    sys.modules["rclpy.qos"] = module("rclpy.qos", QoSProfile=QoSProfile, QoSReliabilityPolicy=Policy,
                                      QoSDurabilityPolicy=Policy, QoSHistoryPolicy=Policy)
    sys.modules["geometry_msgs"] = module("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = module("geometry_msgs.msg", PoseStamped=PoseStamped, TwistStamped=TwistStamped)
    sys.modules["nav_msgs"] = module("nav_msgs")
    sys.modules["nav_msgs.msg"] = module("nav_msgs.msg", Odometry=Odometry)
    sys.modules["std_msgs"] = module("std_msgs")
    sys.modules["std_msgs.msg"] = module("std_msgs.msg", String=String)
    sys.modules["px4_msgs"] = module("px4_msgs")
    sys.modules["px4_msgs.msg"] = module("px4_msgs.msg", VehicleLocalPosition=VehicleLocalPosition)
