"""Publish Gazebo simulation time as ROS /clock for the classical launch."""

from gz.msgs10.clock_pb2 import Clock as GazeboClock
from gz.transport13 import Node as GazeboNode
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import try_shutdown
from rosgraph_msgs.msg import Clock

from examples.gazebo_sensor_contract import ros_clock_parts


def main():
    rclpy.init()
    node = rclpy.create_node("classical_gazebo_clock")
    try:
        publisher = node.create_publisher(Clock, "/clock", qos_profile_sensor_data)
        transport = GazeboNode()

        def forward(message):
            clock = Clock()
            try:
                clock.clock.sec, clock.clock.nanosec = ros_clock_parts(message)
            except ValueError:
                return
            publisher.publish(clock)

        if not transport.subscribe(GazeboClock, "/world/dronehacks_course/clock", forward):
            raise RuntimeError("Gazebo world clock subscription failed")
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try_shutdown()


if __name__ == "__main__":
    main()
