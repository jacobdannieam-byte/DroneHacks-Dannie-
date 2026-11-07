"""Watches for signs that the drone has hit something: a sudden tilt or a sudden spin.

This runs on the drone and in the simulator. It is plain arithmetic, with no ROS or PX4. The
flight program gives it PX4's report of the drone's attitude and rotation speed many times per
second. If it reports a touch, the flight program stops the run and lands where the drone is.
The team decided on 2026-11-07 that any touch the drone can detect must stop the run.

How a touch shows up
--------------------
The real drone has no touch sensor. But a propeller or the frame hitting a wall shoves the
drone: it tips over further, and turns much faster, than it ever does in normal flight. PX4
reports both: the drone's attitude (which way it is tilted and pointing) and its rotation speed.
This flags a touch when the tilt goes past LARGEST_NORMAL_TILT_DEGREES, or the rotation speed
past LARGEST_NORMAL_ROTATION_DEGREES_PER_SECOND, for at least TOUCH_SIGNS_SHORTEST_SECONDS.

A light brush that doesn't disturb the flight can't be noticed this way.

In the simulator this never fires for a wall touch, because PX4's built-in physics doesn't know
about walls: the simulated drone flies straight through them. There, the referee reports every
touch instead (simulator/ros_packages/course_simulator/course_simulator/referee.py). Running
this in the simulator still shows that it doesn't fire by mistake in normal flight.

What it does not do: land. The flight program does that.
"""
import math

# Our own choice. In six simulator flights on 2026-11-07 the drone never tilted more than 3.3
# degrees. The real drone will wobble more than the simulated one, so we leave plenty of room.
# This has not been checked on the real drone yet.
LARGEST_NORMAL_TILT_DEGREES = 20.0
# Our own choice. In the same flights the drone never turned faster than 29 degrees per second.
# The same caution applies.
LARGEST_NORMAL_ROTATION_DEGREES_PER_SECOND = 120.0
# Our own choice. A single unusual report could be a glitch. A real knock lasts longer.
TOUCH_SIGNS_SHORTEST_SECONDS = 0.05


class TouchDetector:

    def __init__(self):
        # When the tilt or rotation speed first went past its limit, or None if both are normal.
        self.unusual_motion_began_at_seconds = None

    def check(self, now_seconds, tilt_degrees, rotation_degrees_per_second):
        """Returns None while the motion is normal, or a sentence describing the suspected touch."""
        unusual = (
            tilt_degrees > LARGEST_NORMAL_TILT_DEGREES
            or rotation_degrees_per_second > LARGEST_NORMAL_ROTATION_DEGREES_PER_SECOND
        )
        if not unusual:
            self.unusual_motion_began_at_seconds = None
            return None
        if self.unusual_motion_began_at_seconds is None:
            self.unusual_motion_began_at_seconds = now_seconds
        if now_seconds - self.unusual_motion_began_at_seconds < TOUCH_SIGNS_SHORTEST_SECONDS:
            return None
        return (
            f"The drone is tilted {tilt_degrees:.0f} degrees and turning at {rotation_degrees_per_second:.0f} "
            f"degrees per second. Normal flight stays below {LARGEST_NORMAL_TILT_DEGREES:.0f} degrees and "
            f"{LARGEST_NORMAL_ROTATION_DEGREES_PER_SECOND:.0f} degrees per second, so it has probably hit something."
        )


def tilt_degrees(attitude_quaternion):
    """How far the drone is tipped from level, in degrees, whichever way it leans.

    attitude_quaternion is (w, x, y, z), the order PX4 uses. A quaternion is PX4's way of
    storing an orientation as four numbers. 1 - 2(x² + y²) is the cosine of the angle between
    the drone's up direction and the true up direction."""
    w, x, y, z = attitude_quaternion
    cosine_of_tilt = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine_of_tilt))))


def rotation_degrees_per_second(angular_velocity_radians_per_second):
    """The drone's total rotation speed, whatever the axis, in degrees per second."""
    x, y, z = angular_velocity_radians_per_second
    return math.degrees(math.sqrt(x * x + y * y + z * z))
