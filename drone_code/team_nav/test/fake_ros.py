"""Stand-ins for the few parts of ROS 2 that mission_node.py uses, so its wiring can be tested on a
laptop without ROS installed. Test code only: it is never installed or run on the drone.

install() puts these stand-ins where Python looks for rclpy and the message packages. Each topic a
node subscribes to or publishes on is recorded, so a test can deliver a message straight to the
node's callback and read back what it published.

Only imitate what ROS 2 Humble really has (the drone runs Humble). A stand-in for something Humble
lacks lets a test pass for code that crashes on the drone: that happened once, with
rclpy.parameter_client, which only exists from ROS 2 Iron onward.
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

    def error(self, text):
        self.lines.append(("error", text))

    def warn(self, text):
        self.lines.append(("warn", text))


class Node:
    parameter_overrides = {}                     # tests set ROS parameters here before building a node

    def __init__(self, name):
        self.name = name
        self.subscriptions = {}                  # topic -> (message type, callback, quality of service)
        self.publishers = {}                     # topic -> Publisher
        self.timers = []                         # [(period seconds, callback)]
        self.service_clients = {}                # service name -> ServiceClient
        self.service_requests = []               # [(kind, payload, future)], in the order made
        self.log_lines = []
        self.parameters = {}

    def declare_parameter(self, name, default, descriptor=None):
        value = Node.parameter_overrides.get(name, default)
        self.parameters[name] = value
        return Parameter(value)

    def create_subscription(self, message_type, topic, callback, quality_of_service):
        self.subscriptions[topic] = (message_type, callback, quality_of_service)

    def create_publisher(self, message_type, topic, quality_of_service):
        publisher = Publisher(topic, message_type)
        self.publishers[topic] = publisher
        return publisher

    def create_client(self, service_type, service_name):
        client = ServiceClient(self, service_type, service_name)
        self.service_clients[service_name] = client
        return client

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


class Int32:
    def __init__(self, data=0):
        self.data = data


class Image:
    """sensor_msgs/Image: a picture, row after row of bytes."""

    def __init__(self):
        self.header = Header()
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.is_bigendian = 0
        self.step = 0
        self.data = b""


class CameraInfo:
    """sensor_msgs/CameraInfo: the camera's lens numbers (k, d, r, p as flat lists)."""

    def __init__(self):
        self.header = Header()
        self.height = 0
        self.width = 0
        self.distortion_model = ""
        self.d = []
        self.k = [0.0] * 9
        self.r = [0.0] * 9
        self.p = [0.0] * 12


class VehicleLocalPosition:
    def __init__(self):
        self.dist_bottom = math.nan
        self.dist_bottom_valid = False
        self.xy_reset_counter = 0
        self.heading_reset_counter = 0


class VehicleStatus:
    def __init__(self, nav_state=0):
        self.nav_state = nav_state


class VisualSlamStatus:
    """isaac_ros_visual_slam_interfaces/VisualSlamStatus, as read on drone 16."""

    def __init__(self, vo_state=0):
        self.header = Header()
        self.vo_state = vo_state


class PointField:
    """sensor_msgs/PointField: one field of every point in a PointCloud2."""
    FLOAT32 = 7

    def __init__(self, name="", offset=0, datatype=7, count=1):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


class PointCloud2:
    """sensor_msgs/PointCloud2: points packed as bytes, point_step bytes each."""

    def __init__(self):
        self.header = Header()
        self.height = 1
        self.width = 0
        self.fields = []
        self.is_bigendian = False
        self.point_step = 0
        self.row_step = 0
        self.data = b""
        self.is_dense = False


class ParameterDescriptor:
    def __init__(self, dynamic_typing=False):
        self.dynamic_typing = dynamic_typing


class FakeFuture:
    """A request's answer, delivered when the test calls finish()."""

    def __init__(self):
        self.callbacks = []
        self.answer = None

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def result(self):
        return self.answer

    def finish(self, answer):
        self.answer = answer
        for callback in self.callbacks:
            callback(self)


class ParameterStandIn:
    """rclpy.parameter.Parameter: a name and a value."""

    def __init__(self, name, value=None):
        self.name = name
        self.value = value

    def to_parameter_msg(self):
        return types.SimpleNamespace(name=self.name, value=self.value)


class SetParameters:
    """rcl_interfaces.srv.SetParameters."""
    class Request:
        def __init__(self):
            self.parameters = []


class GetParameters:
    """rcl_interfaces.srv.GetParameters."""
    class Request:
        def __init__(self):
            self.names = []


class ServiceClient:
    """A client made by node.create_client. Tests set ready on it, and answer each recorded request
    through its future. Every request goes in the node's service_requests, in the order made."""

    def __init__(self, node, service_type, service_name):
        self.node = node
        self.service_type = service_type
        self.service_name = service_name
        self.ready = False

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = FakeFuture()
        if self.service_type is SetParameters:
            payload = [(parameter.name, parameter.value) for parameter in request.parameters]
            self.node.service_requests.append(("set", payload, future))
        else:
            self.node.service_requests.append(("get", list(request.names), future))
        return future


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


STAND_IN_MODULES = ("rclpy", "rclpy.node", "rclpy.qos", "rclpy.parameter", "geometry_msgs", "geometry_msgs.msg", "nav_msgs",
                    "nav_msgs.msg", "std_msgs", "std_msgs.msg", "sensor_msgs", "sensor_msgs.msg", "px4_msgs", "px4_msgs.msg", "rcl_interfaces",
                    "rcl_interfaces.msg", "rcl_interfaces.srv", "isaac_ros_visual_slam_interfaces",
                    "isaac_ros_visual_slam_interfaces.msg")


def uninstall():
    for name in STAND_IN_MODULES:
        sys.modules.pop(name, None)


def install():
    sys.modules["rclpy"] = module("rclpy", init=lambda args=None: None, spin=lambda node: None,
                                  ok=lambda: False, shutdown=lambda: None)
    sys.modules["rclpy.node"] = module("rclpy.node", Node=Node)
    sys.modules["rclpy.parameter"] = module("rclpy.parameter", Parameter=ParameterStandIn)
    sys.modules["rclpy.qos"] = module("rclpy.qos", QoSProfile=QoSProfile, QoSReliabilityPolicy=Policy,
                                      QoSDurabilityPolicy=Policy, QoSHistoryPolicy=Policy)
    sys.modules["geometry_msgs"] = module("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = module("geometry_msgs.msg", PoseStamped=PoseStamped, TwistStamped=TwistStamped)
    sys.modules["nav_msgs"] = module("nav_msgs")
    sys.modules["nav_msgs.msg"] = module("nav_msgs.msg", Odometry=Odometry)
    sys.modules["std_msgs"] = module("std_msgs")
    sys.modules["std_msgs.msg"] = module("std_msgs.msg", String=String, Int32=Int32)
    sys.modules["sensor_msgs"] = module("sensor_msgs")
    sys.modules["sensor_msgs.msg"] = module("sensor_msgs.msg", PointCloud2=PointCloud2, PointField=PointField,
                                            Image=Image, CameraInfo=CameraInfo)
    sys.modules["px4_msgs"] = module("px4_msgs")
    sys.modules["px4_msgs.msg"] = module("px4_msgs.msg", VehicleLocalPosition=VehicleLocalPosition,
                                         VehicleStatus=VehicleStatus)
    sys.modules["isaac_ros_visual_slam_interfaces"] = module("isaac_ros_visual_slam_interfaces")
    sys.modules["isaac_ros_visual_slam_interfaces.msg"] = module("isaac_ros_visual_slam_interfaces.msg",
                                                                 VisualSlamStatus=VisualSlamStatus)
    sys.modules["rcl_interfaces"] = module("rcl_interfaces")
    sys.modules["rcl_interfaces.msg"] = module("rcl_interfaces.msg", ParameterDescriptor=ParameterDescriptor)
    sys.modules["rcl_interfaces.srv"] = module("rcl_interfaces.srv", SetParameters=SetParameters,
                                               GetParameters=GetParameters)
