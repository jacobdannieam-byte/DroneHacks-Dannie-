"""Watches for PX4's height estimate going wrong, by comparing it with the bottom laser's raw
reading (HeightCheck) and with camera tracking's height (CameraHeightCheck).

This runs on the drone and in the simulator. It is plain arithmetic, with no ROS or PX4. The
flight program gives it PX4's height and the other heights about 10 times per second. If it
reports a problem, the flight program stops the run and lands where the drone is.

Why
---
Drone 16's PX4 uses the bottom laser as its main height source. PX4 can stop trusting the
laser, for example after the laser passes over the window's lower wall or a boulder while the
drone is moving slowly (docs/ARCHITECTURE.md, section 7). PX4 has no other good height source,
so its height then drifts, and the drone flies at a height nobody is controlling. In the window
there are only about 17 cm to spare above and below the drone.

Our flight code can read the laser's raw reading, which PX4 passes on. Over clear floor, that
reading is the drone's true height (allowing for tilt). So if PX4's height and the laser's
disagree by more than HEIGHT_GAP_LIMIT_METRES, for longer than HEIGHT_GAP_LONGEST_SECONDS,
while the laser sees clear floor, PX4's height can't be trusted, and the run should stop.

"Clear floor" means that nothing standing on the floor (walls, tunnel sides, the safety net,
boulders in the drone's map or in any other map of the course) is within
CLEAR_FLOOR_MARGIN_METRES of the drone's centre. Elsewhere the laser may be seeing the top of a
boulder or a wall, so its reading is not the height above the floor, and the check waits.

Both heights are measured from the launch mark: how far each has risen since the drone stood
there.

Camera tracking's height
------------------------
The real drone does not pass on the laser's raw reading, so HeightCheck can't run there. And
PX4's own "I use the laser" flag is not enough: in a simulator flight on 2026-11-07, the drone
crossed a boulder at 0.4 m/s, PX4 took the boulder's top for a drop in height, and its height
ended up 50 cm too low while its flag still said it was using the laser. Camera tracking
measured the true height all along. So CameraHeightCheck compares PX4's height with camera
tracking's, anywhere on the course, and reports a problem when they disagree by more than
CAMERA_HEIGHT_GAP_LIMIT_METRES for CAMERA_HEIGHT_GAP_LONGEST_SECONDS. Camera tracking's height
drifts, so this limit is much looser than the laser's; it catches big failures, not small ones.

What it does not do: fix the height, or land. The flight program does that.
"""
import math

import numpy as np

from team_nav.course_map import BlockingCircle, distance_to_shapes_metres, shapes_blocking_between_heights

# Our own choice, from six simulator flights on 2026-11-07. Over clear floor, while PX4 was
# still using the laser, PX4's height and the laser's were within 9 cm of each other almost all
# the time (briefly up to 13 cm, mostly while climbing, because the laser reports a little
# late). After PX4 stopped using the laser, they drifted 20 to 54 cm apart.
HEIGHT_GAP_LIMIT_METRES = 0.15
# Our own choice. Long enough to ignore the short gaps while climbing to the window and back.
HEIGHT_GAP_LONGEST_SECONDS = 1.0

# Our own choice. The laser's beam may be up to 27 degrees wide (we don't know yet), so at our
# 0.8 m cruise height it may see up to 0.19 m around the point below the drone. The camera
# tracking had drifted about 0.20 m by the window in simulator flights, so the drone may be that
# far from where it thinks. Together about 0.40 m, the same as stopping_spots.py uses.
CLEAR_FLOOR_MARGIN_METRES = 0.40

# Our own choice. Anything that reaches this high above the floor stands on the floor, as far as
# the laser is concerned.
FLOOR_LEVEL_HEIGHT_METRES = 0.05

# Our own choice. Nobody has measured the real camera tracking's height drift. The route follower
# already needs camera tracking's height within 10 cm of each corner's height (route_follower.py),
# so a drift near 30 cm would stall the run anyway. In simulator flights PX4's height trailed the
# true height by up to 12 cm while climbing, and the boulder failure above reached 30 cm within
# half a second and 50 cm within a second.
CAMERA_HEIGHT_GAP_LIMIT_METRES = 0.30
# Our own choice, the same as the laser comparison's.
CAMERA_HEIGHT_GAP_LONGEST_SECONDS = 1.0


class HeightCheck:

    def __init__(self, course_map, other_possible_boulders=()):
        """course_map is the drone's map. other_possible_boulders are boulders from other maps of
        the course, which the laser may also see."""
        self.shapes_on_the_floor = shapes_blocking_between_heights(course_map, 0.0, FLOOR_LEVEL_HEIGHT_METRES)
        for boulder in other_possible_boulders:
            self.shapes_on_the_floor.append(BlockingCircle(boulder.name, boulder.centre_metres, boulder.radius_metres))
        # When the two heights first disagreed by too much, or None if they agree.
        self.gap_began_at_seconds = None

    def laser_sees_clear_floor(self, drone_position_on_course_metres):
        point = np.asarray(drone_position_on_course_metres, dtype=float)[np.newaxis, :2]
        distance_metres, _ = distance_to_shapes_metres(point, self.shapes_on_the_floor)
        return float(distance_metres[0]) > CLEAR_FLOOR_MARGIN_METRES

    def check(self, now_seconds, drone_position_on_course_metres, px4_height_metres, laser_height_metres):
        """Both heights are metres risen since the launch mark. Returns None while all is well, or
        a sentence saying why the run must stop."""
        if not self.laser_sees_clear_floor(drone_position_on_course_metres):
            # The laser may be seeing a boulder or a wall, so its reading proves nothing here.
            # Keep any gap's start time, so a gap that continues past a boulder still counts.
            return None
        gap_metres = px4_height_metres - laser_height_metres
        if abs(gap_metres) <= HEIGHT_GAP_LIMIT_METRES:
            self.gap_began_at_seconds = None
            return None
        if self.gap_began_at_seconds is None:
            self.gap_began_at_seconds = now_seconds
        if now_seconds - self.gap_began_at_seconds < HEIGHT_GAP_LONGEST_SECONDS:
            return None
        return (
            f"PX4 thinks the drone has risen {px4_height_metres:.2f} m since the launch mark, but over clear floor "
            f"the bottom laser says {laser_height_metres:.2f} m, and they have disagreed by more than "
            f"{HEIGHT_GAP_LIMIT_METRES:.2f} m for {HEIGHT_GAP_LONGEST_SECONDS:.0f} second. PX4 has probably stopped "
            f"using the laser for height (docs/ARCHITECTURE.md, section 7), so its height can't be trusted."
        )


class CameraHeightCheck:
    """Compares PX4's height with camera tracking's height, both measured from the launch mark.
    Works anywhere on the course, because camera tracking doesn't look at the floor."""

    def __init__(self):
        # When the two heights first disagreed by too much, or None if they agree.
        self.gap_began_at_seconds = None

    def check(self, now_seconds, px4_height_metres, camera_height_metres):
        """Returns None while all is well, or a sentence saying why the run must stop."""
        gap_metres = px4_height_metres - camera_height_metres
        if abs(gap_metres) <= CAMERA_HEIGHT_GAP_LIMIT_METRES:
            self.gap_began_at_seconds = None
            return None
        if self.gap_began_at_seconds is None:
            self.gap_began_at_seconds = now_seconds
        if now_seconds - self.gap_began_at_seconds < CAMERA_HEIGHT_GAP_LONGEST_SECONDS:
            return None
        return (
            f"PX4 thinks the drone has risen {px4_height_metres:.2f} m since the launch mark, but camera tracking says "
            f"{camera_height_metres:.2f} m, and they have disagreed by more than {CAMERA_HEIGHT_GAP_LIMIT_METRES:.2f} m "
            f"for {CAMERA_HEIGHT_GAP_LONGEST_SECONDS:.0f} second. PX4's height has probably gone wrong, for example "
            f"after the bottom laser crossed a boulder (docs/ARCHITECTURE.md, section 7), so it can't be trusted."
        )


def laser_height_above_launch_metres(laser_reading_metres, laser_reading_on_launch_mark_metres, tilt_degrees):
    """How far the laser has risen since the launch mark. The laser is fixed to the frame, so when
    the drone tilts, it measures along a slanted line, which is longer than the height."""
    return laser_reading_metres * math.cos(math.radians(tilt_degrees)) - laser_reading_on_launch_mark_metres
