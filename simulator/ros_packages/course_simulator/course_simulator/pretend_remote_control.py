"""Pretends to be the teammate holding the remote control at the start of a run.

SIMULATOR ONLY. Flight code must never do what this file does, because on the real drone
these actions belong to a person, and the rules forbid our flight code from sending them.

The start sequence it copies
----------------------------
The organizers' change log (version 0.3.2, 2026-11-06) describes this sequence:

  1. A person arms the drone on the remote. "Arming" means giving the motors permission to
     spin. The switch is not yet on Position.
  2. On the referee's signal, the person flips the mode switch to Position. "Position" is the
     mode where the drone holds its place by itself.
  3. The organizers' flight_interface program sees that flip, and asks PX4 to hand control
     to our code. This hand-over mode is called "Offboard".

Nobody flies the drone by hand at any point, so step 2 is the single start command.

This program does steps 1 and 2 when something calls its start_run service. Step 3 is done by
the organizers' real code, so the simulator tests the real rules for when the hand-over is
refused. For example, it is refused if our flight code is not already sending targets.

Why it also sends stick positions
---------------------------------
PX4 refuses to enter Position mode unless it is hearing from a remote control. So we send a
steady message that says "the sticks are in their resting positions" 20 times per second.
"""
import json

import rclpy
from px4_msgs.msg import ManualControlSetpoint, VehicleCommand
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Our own choice. PX4 treats the remote as lost if stick messages stop, so they must keep coming,
# and 20 per second is plenty.
STICK_MESSAGES_PER_SECOND = 20.0

# Our own choice: how often the pretend person checks whether to do the next step of the start.
SEQUENCE_STEPS_PER_SECOND = 5.0

# PX4's numbers for these commands and modes, from VehicleCommand.msg and PX4's mode list.
COMMAND_SET_MODE = 176
COMMAND_ARM_OR_DISARM = 400
ARM = 1.0
MODE_FLAG_USE_PX4_MODE_NUMBERS = 1.0
PX4_MODE_NUMBER_POSITION = 3.0
PX4_MODE_NUMBER_STABILIZED = 7.0

# Tells PX4 that the stick message came from a radio receiver, because drone 16 is set to accept
# hand control from the radio only.
STICK_SOURCE_RADIO = 1

# A throttle stick pulled all the way down. A person keeps it there until the hand-over.
THROTTLE_STICK_LOWEST = -1.0

# The sequence waits for arming to be confirmed before it flips the switch to Position.
WAITING_FOR_START_REQUEST = "waiting for the start request"
WAITING_FOR_ARMING = "waiting for the motors to be armed"
FINISHED = "finished"

PX4_INPUT_SETTINGS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class PretendRemoteControl(Node):

    def __init__(self):
        super().__init__("pretend_remote_control")
        self.sequence_state = WAITING_FOR_START_REQUEST
        self.motors_are_armed = False

        self.stick_publisher = self.create_publisher(
            ManualControlSetpoint, "/fmu/in/manual_control_input", PX4_INPUT_SETTINGS
        )
        self.command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", PX4_INPUT_SETTINGS
        )
        self.create_subscription(String, "/skybeta/state", self.on_drone_state, 10)
        self.create_service(Trigger, "/simulator_only/start_run", self.on_start_request)
        self.create_timer(1.0 / STICK_MESSAGES_PER_SECOND, self.publish_resting_sticks)
        self.create_timer(1.0 / SEQUENCE_STEPS_PER_SECOND, self.continue_start_sequence)

    def on_drone_state(self, message):
        self.motors_are_armed = bool(json.loads(message.data)["armed"])

    def on_start_request(self, _request, response):
        if self.sequence_state != WAITING_FOR_START_REQUEST:
            response.success = False
            response.message = "The start sequence has already been used. Restart the simulator to run again."
            return response

        self.get_logger().info("Pretend person: setting the switch away from Position, then arming.")
        self.send_command(COMMAND_SET_MODE, MODE_FLAG_USE_PX4_MODE_NUMBERS, PX4_MODE_NUMBER_STABILIZED)
        self.send_command(COMMAND_ARM_OR_DISARM, ARM)
        self.sequence_state = WAITING_FOR_ARMING
        response.success = True
        response.message = "The start sequence has begun."
        return response

    def continue_start_sequence(self):
        if self.sequence_state != WAITING_FOR_ARMING:
            return
        if not self.motors_are_armed:
            # A command can be lost on its way to PX4, so we repeat it until it is confirmed.
            self.send_command(COMMAND_ARM_OR_DISARM, ARM)
            return
        self.get_logger().info("Pretend person: motors armed, flipping the switch to Position.")
        self.send_command(COMMAND_SET_MODE, MODE_FLAG_USE_PX4_MODE_NUMBERS, PX4_MODE_NUMBER_POSITION)
        self.sequence_state = FINISHED

    def send_command(self, command_number, first_value, second_value=0.0):
        command = VehicleCommand()
        command.timestamp = self.get_clock().now().nanoseconds // 1000
        command.command = command_number
        command.param1 = first_value
        command.param2 = second_value
        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 1
        command.from_external = True
        self.command_publisher.publish(command)

    def publish_resting_sticks(self):
        time_now_microseconds = self.get_clock().now().nanoseconds // 1000
        sticks = ManualControlSetpoint()
        sticks.timestamp = time_now_microseconds
        sticks.timestamp_sample = time_now_microseconds
        sticks.valid = True
        sticks.data_source = STICK_SOURCE_RADIO
        sticks.throttle = THROTTLE_STICK_LOWEST
        self.stick_publisher.publish(sticks)


def main():
    rclpy.init()
    rclpy.spin(PretendRemoteControl())
