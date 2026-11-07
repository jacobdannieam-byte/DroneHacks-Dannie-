"""Sends the drone its targets: a position to fly to, or a speed to move at.

REAL DRONE CODE, also used unchanged in the simulator. It is the only file in this package that
publishes anything that can move the drone, and it publishes on exactly two topics, both read by
the organizers' flight_interface program:

  /skybeta/setpoint/pose    a position target. The drone flies there and holds it. It stays in
                            force until replaced; it never expires.
  /skybeta/setpoint/twist   a speed target. It expires after 1 second (flight_interface's
                            setpoint_timeout_s), after which flight_interface holds the last
                            position target instead. So speed targets must be repeated.

Positions and speeds are in camera tracking's directions ("odom"): metres forward, left and up,
measured along the way the drone pointed when camera tracking started. Camera tracking is the
program that works out the drone's position from its cameras.

What it deliberately does not do
--------------------------------
It never publishes on /fmu/in/* (PX4's own inputs), never sends PX4 a command, never arms the
drone ("arming" gives the motors permission to spin) and never changes PX4's mode. The rules
and the organizers' software reserve all of those for flight_interface and for the teammate
with the remote control. It does not check whether a target is safe; run_safety.py does that.
It does not decide when to send; flight_steps.py does.
"""
import math
import time
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped, TwistStamped

# From the organizers' code (flight_interface.py's default parameters).
POSE_TARGET_TOPIC = "/skybeta/setpoint/pose"
SPEED_TARGET_TOPIC = "/skybeta/setpoint/twist"

# From the organizers' code: flight_interface reads targets in camera tracking's frame, "odom".
TARGET_FRAME_NAME = "odom"

# Our own choice: keep up to this many unsent targets queued. Old targets are useless, so short.
TARGET_QUEUE_LENGTH = 10

POSITION = "position"
SPEED = "speed"


@dataclass
class SentTarget:
    kind: str                        # POSITION or SPEED.
    forward_left_up: tuple           # Metres for a position, metres per second for a speed.
    monotonic_seconds: float         # When it was sent, by time.monotonic().


class TargetSender:

    def __init__(self, node, on_target_sent=None):
        """node is the ROS node to publish from. on_target_sent, if given, is called with each
        SentTarget, so the flight program can record it."""
        self.pose_publisher = node.create_publisher(PoseStamped, POSE_TARGET_TOPIC, TARGET_QUEUE_LENGTH)
        self.speed_publisher = node.create_publisher(TwistStamped, SPEED_TARGET_TOPIC, TARGET_QUEUE_LENGTH)
        self.clock = node.get_clock()
        self.on_target_sent = on_target_sent
        self.last_target = None
        self.targets_sent = 0

    def send_position(self, forward_left_up_metres):
        """Fly to this position, in camera tracking's frame, and hold it."""
        values = _checked_numbers(forward_left_up_metres, "position target")
        target = PoseStamped()
        target.header.stamp = self.clock.now().to_msg()
        target.header.frame_id = TARGET_FRAME_NAME
        target.pose.position.x, target.pose.position.y, target.pose.position.z = values
        # An orientation of all zeros tells flight_interface not to command a heading, so PX4
        # keeps the heading the drone already has. We never turn the drone on purpose.
        target.pose.orientation.w = 0.0
        self.pose_publisher.publish(target)
        self._remember(POSITION, values)

    def send_speed(self, forward_left_up_metres_per_second):
        """Move at this speed, in camera tracking's directions. Must be repeated within 1 s."""
        values = _checked_numbers(forward_left_up_metres_per_second, "speed target")
        target = TwistStamped()
        target.header.stamp = self.clock.now().to_msg()
        target.header.frame_id = TARGET_FRAME_NAME
        target.twist.linear.x, target.twist.linear.y, target.twist.linear.z = values
        self.speed_publisher.publish(target)
        self._remember(SPEED, values)

    def send_downward_speed(self, metres_per_second):
        """Move straight down at this speed, with no sideways movement. Used for landing."""
        self.send_speed((0.0, 0.0, -float(metres_per_second)))

    def resend_last(self):
        """Repeats the last target, for holding still. Does nothing if nothing was ever sent."""
        if self.last_target is None:
            return
        if self.last_target.kind == POSITION:
            self.send_position(self.last_target.forward_left_up)
        else:
            self.send_speed(self.last_target.forward_left_up)

    def _remember(self, kind, values):
        self.last_target = SentTarget(kind, values, time.monotonic())
        self.targets_sent += 1
        if self.on_target_sent is not None:
            self.on_target_sent(self.last_target)


def _checked_numbers(three_values, what_it_is):
    values = tuple(float(value) for value in three_values)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(
            f"The {what_it_is} {three_values} is not three real numbers, so it was not sent. "
            f"This is a bug in the flight program; flight_interface would have ignored it anyway."
        )
    return values
