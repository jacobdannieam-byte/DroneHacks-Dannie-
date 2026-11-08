"""Checks, before anything is sent to the drone, that camera tracking started on the launch mark.

REAL DRONE CODE. Plain arithmetic with no ROS. full_course.py runs it once, after the drone is
ready on the ground and before the first target is sent.

Why it matters
--------------
Camera tracking (cuVSLAM, the program that works out the drone's position from its cameras)
measures position from wherever it started. It starts with the drone software, so if the
drone sat on the launch mark then, tracking reads about (0, 0, 0) on the mark. PX4 (the
software on the flight controller) takes its position from camera tracking too.

If tracking reads far from zero while the drone sits on the mark, the drone was moved after
tracking started, or tracking drifted while it sat. Either way its reports are suspect. The
flight measures everything from where the drone sits when it is armed, so it would not notice
this by itself. That is why it refuses to start.

What this file does not do: it does not check the nose direction, and it does not compare
tracking with the map.
"""
import math

# Our own choices (the team's request of 2026-11-08). In Gazebo the pretend tracking read within
# 1 cm of zero on the mark (run 20261108_122536), so these leave plenty of room for noise.
LARGEST_HORIZONTAL_OFFSET_METRES = 0.30
LARGEST_HEIGHT_OFFSET_METRES = 0.10

# The words to fix the problem, shared by every refusal.
HOW_TO_FIX = ("Put the drone on the launch mark with its nose down the first corridor, then restart the "
              "drone software so that camera tracking starts there (on the Jetson: docker rm -f skybeta-stack, "
              "wait about 40 s, then ./drone.sh start test_flights). Then start this flight again.")


def problem_with_tracking_on_the_launch_mark(tracked_forward_left_up_metres):
    """A sentence saying why the flight must not start, or None if tracking is close enough to
    where it started. The input is camera tracking's (forward, left, up) in metres, or None."""
    if tracked_forward_left_up_metres is None:
        return "Camera tracking has not reported a position yet. " + HOW_TO_FIX
    forward_metres, left_metres, up_metres = tracked_forward_left_up_metres
    horizontal_offset_metres = math.hypot(forward_metres, left_metres)
    if horizontal_offset_metres <= LARGEST_HORIZONTAL_OFFSET_METRES and abs(up_metres) <= LARGEST_HEIGHT_OFFSET_METRES:
        return None
    return (f"Camera tracking says the drone is {horizontal_offset_metres:.2f} m sideways and {up_metres:+.2f} m "
            f"in height from where tracking started, but on the launch mark it must be within "
            f"{LARGEST_HORIZONTAL_OFFSET_METRES:.2f} m sideways and {LARGEST_HEIGHT_OFFSET_METRES:.2f} m in height. "
            + HOW_TO_FIX)
