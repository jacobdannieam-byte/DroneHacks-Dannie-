"""Converts positions between the course map's directions and the camera tracking's directions.

This runs on the drone and in the simulator. It is plain arithmetic, with no ROS.

The two sets of directions
--------------------------
  - The course map (course_map.py) uses fixed directions on the course: metres east, metres
    north, and height above the floor. The route planner works in these.
  - The camera tracking only knows where the drone started. It reports the drone's position
    as metres forward and metres left of where the drone stood, measured along the way the
    drone's nose pointed at the start, and metres up. We call these the "start frame".

To convert, we need where the drone started (the launch mark, from the map) and which way its
nose pointed then, as an angle measured from the course's east direction towards north. 90
degrees means the nose pointed north. The rules don't say which way the drone will face, so the
flight program must be told (docs/ARCHITECTURE.md, section 12).

Heights are the same in both, because the launch mark is on the floor.

Drift correction: camera tracking drifts slowly. wall_alignment.py measures how far, and live
navigation stores it in drift_correction_metres (east, north). Every conversion here includes
it, so the map, the navigation, the targets and the safety checks all use the corrected position.
It stays zero unless live navigation sets it.
"""
import math

import numpy as np


class StartFrame:

    def __init__(self, launch_mark_centre_metres, start_heading_degrees):
        self.launch_mark_centre_metres = np.asarray(launch_mark_centre_metres, dtype=float)[:2]
        heading_radians = math.radians(start_heading_degrees)
        self.start_heading_radians = heading_radians
        # The course directions of the drone's starting forward and left.
        self.forward_on_course = np.array([math.cos(heading_radians), math.sin(heading_radians)])
        self.left_on_course = np.array([-math.sin(heading_radians), math.cos(heading_radians)])
        # How far the drone really is from where camera tracking says, on the course (east, north).
        self.drift_correction_metres = np.zeros(2)

    def course_from_start_frame(self, forward_left_up_metres):
        """(forward, left, up) in the start frame -> (east, north, height) on the course."""
        forward_metres, left_metres, up_metres = forward_left_up_metres
        on_floor = (self.launch_mark_centre_metres + forward_metres * self.forward_on_course
                    + left_metres * self.left_on_course + self.drift_correction_metres)
        return np.array([on_floor[0], on_floor[1], up_metres])

    def start_frame_from_course(self, east_north_height_metres):
        """(east, north, height) on the course -> (forward, left, up) in the start frame."""
        from_launch_mark = (np.asarray(east_north_height_metres[:2], dtype=float) - self.launch_mark_centre_metres
                            - self.drift_correction_metres)
        forward_metres = float(from_launch_mark @ self.forward_on_course)
        left_metres = float(from_launch_mark @ self.left_on_course)
        return np.array([forward_metres, left_metres, float(east_north_height_metres[2])])

    def course_heading_from_start_frame(self, heading_in_start_frame_radians):
        """A nose direction measured from the starting forward toward left -> measured from the
        course's east toward north."""
        return _wrapped(self.start_heading_radians + heading_in_start_frame_radians)

    def start_frame_heading_from_course(self, heading_on_course_radians):
        """The opposite of course_heading_from_start_frame."""
        return _wrapped(heading_on_course_radians - self.start_heading_radians)


def _wrapped(angle_radians):
    """The same angle, between -pi and pi."""
    return math.atan2(math.sin(angle_radians), math.cos(angle_radians))
