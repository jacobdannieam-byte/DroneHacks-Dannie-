"""Subscriber-only native-status adapter for the unchanged Classical referee."""
import json
import rclpy
from px4_msgs.msg import VehicleStatus, VehicleLandDetected
from std_msgs.msg import String
from onboard.diagnostics.phase1_probe import px4_topic
from refnav.node import PX4_QOS


def main():
    rclpy.init();node=rclpy.create_node('referee_native_status')
    publisher=node.create_publisher(String,'/skybeta/state',10)
    state={}
    def status(msg):
        state.update(armed=msg.arming_state==msg.ARMING_STATE_ARMED,offboard_active=msg.nav_state==msg.NAVIGATION_STATE_OFFBOARD)
        publisher.publish(String(data=json.dumps(state)))
    def landed(msg):
        state['landed']=bool(msg.landed)
        publisher.publish(String(data=json.dumps(state)))
    node.create_subscription(VehicleStatus,px4_topic('vehicle_status',VehicleStatus),status,PX4_QOS)
    node.create_subscription(VehicleLandDetected,px4_topic('vehicle_land_detected',VehicleLandDetected),landed,PX4_QOS)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
