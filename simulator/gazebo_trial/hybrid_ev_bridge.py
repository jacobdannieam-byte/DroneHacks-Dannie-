"""Use the authored Gazebo heading reference for simulated external vision."""

import rclpy
from rclpy.parameter import Parameter
from rclpy.utilities import try_shutdown

from examples.gazebo_sensor_contract import SIM_ODOM, SIM_STATUS, gazebo_ev_pose_in_enu
from simulator.organizers_code.px4_odom_bridge import PX4OdomBridge


class GazeboExternalVision(PX4OdomBridge):
    def __init__(self):
        super().__init__(parameter_overrides=[
            Parameter("input_topic", value=SIM_ODOM),
            Parameter("status_topic", value=SIM_STATUS),
            Parameter("pose_frame", value="NED"),
            Parameter("use_sim_time", value=True),
        ], use_global_arguments=False)

    def cb(self, message):
        if message.header.frame_id != "odom" or message.child_frame_id != "base_link":
            return
        pose = message.pose.pose
        p, q = pose.position, pose.orientation
        position, quaternion, covariance = gazebo_ev_pose_in_enu(
            (p.x, p.y, p.z), (q.w, q.x, q.y, q.z), message.pose.covariance)
        p.x, p.y, p.z = position
        q.w, q.x, q.y, q.z = quaternion
        message.pose.covariance = covariance
        super().cb(message)


def main():
    rclpy.init()
    node = GazeboExternalVision()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try_shutdown()


if __name__ == "__main__":
    main()
