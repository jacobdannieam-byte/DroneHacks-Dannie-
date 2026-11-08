"""PRETEND / SIMULATOR ONLY. Never runs on the drone.

Tiny stand-ins for the ROS 2 modules that our flight code imports (rclpy, the message packages,
px4_msgs and the organizers' topic helper), so the real flight code can be imported on a Mac
that has no ROS.

How it fits in
--------------
install_ros_stand_ins() puts fake modules into Python's list of loaded modules (sys.modules)
before anything from team_nav is imported. Every message class is the same "PretendMessage":
an empty object that grows a child object the first time you read a field that was never set,
so code like "message.header.stamp.sec = 3" just works.

rclpy.spin_once(node, timeout_sec) is the one function that matters: it hands the call to the
bench's pretend node (bench_link.BenchNode.spin_once), which moves the pretend world forward by
timeout_sec of simulated time and delivers the messages that fall due. That is how the real
DroneLink.wait_one_cycle() drives the whole bench without being changed.

What it does not do: talk to any network, or behave like real ROS in any other way.
"""
import os
import sys
import types

# If the real ROS is installed, refuse: this file must never replace it.
REAL_ROS_MARKER_MODULE = "rclpy"


class PretendMessage:
    """Any ROS message. Reading a field that was never set gives a new empty PretendMessage, so
    nested fields can be filled in like on a real message."""

    def __init__(self, **fields):
        for name, value in fields.items():
            setattr(self, name, value)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        child = PretendMessage()
        setattr(self, name, child)
        return child


class _AnyNameModule(types.ModuleType):
    """A module where every name asked for is a message class (or a harmless stand-in)."""

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        message_class = type(name, (PretendMessage,), {})
        setattr(self, name, message_class)
        return message_class


def _spin_once(node, timeout_sec=None):
    node.spin_once(0.0 if timeout_sec is None else timeout_sec)


def _get_package_share_directory(_package_name):
    # The installed package folder holds course_maps/ and missions/; the source folder has the same.
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "drone_code", "team_nav")


def _fmu_topic(direction, base_name, _message_type_name):
    # The organizers' helper adds "_v1" to some names on the drone. The bench does not care,
    # because the pretend node matches the same names it was given.
    return f"/fmu/{direction}/{base_name}"


def install_ros_stand_ins():
    """Puts the stand-in modules into sys.modules. Safe to call more than once."""
    if REAL_ROS_MARKER_MODULE in sys.modules and not getattr(sys.modules[REAL_ROS_MARKER_MODULE], "is_pretend", False):
        raise RuntimeError("The real rclpy is already loaded. The noise bench only runs where ROS is not installed, "
                           "so that pretend code can never mix with a real ROS program. Run it on the Mac.")
    any_name_packages = ["geometry_msgs", "sensor_msgs", "nav_msgs", "std_msgs", "px4_msgs", "builtin_interfaces",
                         "rclpy.qos", "rclpy.executors", "rclpy.serialization", "rclpy.signals",
                         "rosidl_runtime_py", "rosidl_runtime_py.utilities"]
    for package_name in any_name_packages:
        top_name = package_name.split(".")[0]
        if top_name not in sys.modules and top_name != "rclpy":
            sys.modules[top_name] = _AnyNameModule(top_name)
        if "." not in package_name:
            sys.modules[package_name + ".msg"] = _AnyNameModule(package_name + ".msg")
        else:
            sys.modules[package_name] = _AnyNameModule(package_name)

    rclpy = types.ModuleType("rclpy")
    rclpy.is_pretend = True
    rclpy.spin_once = _spin_once
    rclpy.init = lambda **_settings: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    rclpy.try_shutdown = lambda: None
    sys.modules["rclpy"] = rclpy
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object     # The bench passes its own node; nothing here builds one.
    sys.modules["rclpy.node"] = rclpy_node
    rclpy_utilities = types.ModuleType("rclpy.utilities")
    rclpy_utilities.remove_ros_args = lambda words: list(words)
    sys.modules["rclpy.utilities"] = rclpy_utilities
    # The message-quality settings are read as class constants (QoSReliabilityPolicy.BEST_EFFORT),
    # so they get plain classes with those names. Their values mean nothing here.
    qos = sys.modules["rclpy.qos"]
    qos.QoSReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT="best effort", RELIABLE="reliable")
    qos.QoSDurabilityPolicy = types.SimpleNamespace(VOLATILE="volatile", TRANSIENT_LOCAL="transient local")
    qos.QoSHistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep last", KEEP_ALL="keep all")
    sys.modules["rclpy.signals"].SignalHandlerOptions = types.SimpleNamespace(NO="no")
    for name in ["qos", "executors", "serialization", "signals"]:
        setattr(rclpy, name, sys.modules["rclpy." + name])
    rclpy.node = rclpy_node
    rclpy.utilities = rclpy_utilities

    ament = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = _get_package_share_directory
    sys.modules["ament_index_python"] = ament
    sys.modules["ament_index_python.packages"] = ament_packages

    interface = types.ModuleType("skybeta_flight_interface")
    interface_topics = types.ModuleType("skybeta_flight_interface.topics")
    interface_topics.fmu_topic = _fmu_topic
    sys.modules["skybeta_flight_interface"] = interface
    sys.modules["skybeta_flight_interface.topics"] = interface_topics
