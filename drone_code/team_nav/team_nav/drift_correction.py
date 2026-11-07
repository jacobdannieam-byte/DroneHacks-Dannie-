"""Corrects camera tracking's slow drift, at route corners, by comparing the depth straight ahead
with the course map.

How it fits: when the route mission has stopped at a corner, and before it turns, mission.py
asks decide_correction whether the measured depth to the wall straight ahead agrees with the
map. MapRanging measures the map's distance; mission_node.py builds it from a course map file.
This file never talks to ROS and never reads a clock.

Why: camera tracking on drone 16 has no loop closure (cuVSLAM enable_localization_n_mapping is
false), so its error grows with distance flown and nothing in it ever corrects that. Late in a
route the drone can be 0.2 m from where it thinks it is (seen in the simulator), enough to touch
a wall. A wall straight ahead at a known distance tells us how far along the heading we really
are.

The rule (from the team, 2026-11-07):
* the wall straight ahead in the map must be 0.4 to 2.5 m away;
* the measured depth must differ from that by less than 0.4 m (a bigger difference means the
  depth is seeing something else, or the map is wrong there: no correction);
* then the position estimate is moved by the difference along the heading, at most 0.2 m per
  correction. If the depth is shorter than the map says, the drone is really closer to the wall,
  so the estimate moves forward.
"No good reading" (too few depth points straight ahead, or too old), the safety net straight
ahead (stereo cannot see a net), or no wall in range: skip, and say why.

The correction never changes PX4's own estimate (the organizers' rules forbid that); mission.py
adds it to camera tracking's position for the route's decisions and takes it off every target it
sends, which moves the drone to where the route really is.
"""
import math
from dataclasses import dataclass

import numpy as np

from .course_map import BlockingStrip, shapes_blocking_between_heights

# The name Kenny's course_map.py gives the course edges.
SAFETY_NET_NAME = "the safety net"
# Our own choice: the map is sliced this far above and below the height the depth was taken at.
SLICE_HALF_HEIGHT_METRES = 0.01


@dataclass
class CorrectionDecision:
    shift_metres: float | None            # along the heading, + forward; None means no correction
    reason: str


class MapRanging:
    """Distance from a point along a heading to the nearest wall in a course map, at a height.
    Points and headings are in the start frame (forward and left of the launch mark, heading from
    the nose's direction at takeoff); start_frame (Kenny's start_frame.py) converts them."""

    def __init__(self, course_map, start_frame):
        self.course_map = course_map
        self.start_frame = start_frame
        self.shapes_by_height = {}

    def shapes_at(self, height_metres):
        key = round(height_metres, 2)
        if key not in self.shapes_by_height:
            self.shapes_by_height[key] = shapes_blocking_between_heights(
                self.course_map, key - SLICE_HALF_HEIGHT_METRES, key + SLICE_HALF_HEIGHT_METRES)
        return self.shapes_by_height[key]

    def distance_ahead(self, forward_metres, left_metres, heading_from_start_radians, height_metres):
        """(metres to the nearest wall straight ahead, its name), or (infinity, None)."""
        start = self.start_frame.course_from_start_frame((forward_metres, left_metres, height_metres))[:2]
        forward_on_course = self.start_frame.forward_on_course
        left_on_course = self.start_frame.left_on_course
        direction = (math.cos(heading_from_start_radians) * forward_on_course
                     + math.sin(heading_from_start_radians) * left_on_course)
        nearest, nearest_name = math.inf, None
        for shape in self.shapes_at(height_metres):
            distance = distance_along_ray_to_shape(start, direction, shape)
            if distance < nearest:
                nearest, nearest_name = distance, shape.name
        return nearest, nearest_name


def distance_along_ray_to_shape(start, direction, shape):
    if isinstance(shape, BlockingStrip):
        return distance_along_ray_to_strip(start, direction, shape)
    return distance_along_ray_to_circle(start, direction, shape.centre_metres, shape.radius_metres)


def distance_along_ray_to_strip(start, direction, strip):
    """A strip is a rectangle around the line from start_metres to end_metres, half_thickness_metres
    wide on each side; with no thickness it is just the line."""
    a, b = np.asarray(strip.start_metres, float)[:2], np.asarray(strip.end_metres, float)[:2]
    along = b - a
    length = np.linalg.norm(along)
    if length < 1e-9:
        return math.inf
    across = np.array([-along[1], along[0]]) / length * strip.half_thickness_metres
    corners = [a + across, b + across, b - across, a - across]
    if strip.half_thickness_metres <= 0.0:
        return distance_along_ray_to_segment(start, direction, a, b)
    nearest = math.inf
    for index in range(4):
        nearest = min(nearest, distance_along_ray_to_segment(start, direction, corners[index], corners[(index + 1) % 4]))
    return nearest


def distance_along_ray_to_segment(start, direction, segment_start, segment_end):
    along = segment_end - segment_start
    denominator = direction[0] * along[1] - direction[1] * along[0]
    if abs(denominator) < 1e-12:
        return math.inf
    to_start = segment_start - start
    distance = (to_start[0] * along[1] - to_start[1] * along[0]) / denominator
    fraction = (to_start[0] * direction[1] - to_start[1] * direction[0]) / denominator
    return distance if distance > 0.0 and 0.0 <= fraction <= 1.0 else math.inf


def distance_along_ray_to_circle(start, direction, centre, radius):
    to_centre = np.asarray(centre, float)[:2] - start
    along = float(to_centre @ direction)
    closest_squared = float(to_centre @ to_centre) - along * along
    if closest_squared > radius * radius:
        return math.inf
    distance = along - math.sqrt(radius * radius - closest_squared)
    return distance if distance > 0.0 else math.inf


def decide_correction(measured_metres, map_metres, map_wall_name, settings):
    """Whether, and how far, to move the position estimate along the heading."""
    if measured_metres is None:
        return CorrectionDecision(None, "no good depth reading straight ahead")
    if map_wall_name is None or not math.isfinite(map_metres):
        return CorrectionDecision(None, "no wall straight ahead in the map")
    if map_wall_name == SAFETY_NET_NAME:
        return CorrectionDecision(None, "the map has only the safety net straight ahead, which the depth cannot see")
    if not settings.drift_nearest_wall_metres <= map_metres <= settings.drift_farthest_wall_metres:
        return CorrectionDecision(None, f"the map's wall straight ahead ({map_wall_name}) is {map_metres:.2f} m away, "
                                        f"outside {settings.drift_nearest_wall_metres:g} to {settings.drift_farthest_wall_metres:g} m")
    difference = map_metres - measured_metres
    if abs(difference) >= settings.drift_largest_difference_metres:
        return CorrectionDecision(None, f"depth {measured_metres:.2f} m and map {map_metres:.2f} m ({map_wall_name}) "
                                        f"differ by {abs(difference):.2f} m, too much to trust")
    largest = settings.drift_largest_correction_metres
    shift = max(-largest, min(largest, difference))
    return CorrectionDecision(shift, f"depth {measured_metres:.2f} m, map {map_metres:.2f} m to {map_wall_name}: "
                                     f"moved the estimate {shift:+.2f} m along the heading")
