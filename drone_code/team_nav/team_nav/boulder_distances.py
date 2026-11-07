"""Measures how close the drone's centre comes to boulders, seen from above.

This runs on the drone and in the simulator. clearance_grid.py and route_planner.py use it.

The drone's bottom laser points straight down from the middle of the drone and measures its
height above the floor. Over a boulder, the laser reads the boulder's top instead of the floor,
so the drone's height reading suddenly drops by the boulder's height. The route planner
therefore tries to keep the drone's centre away from every boulder in the map, using these
measurements.

A boulder here is any low obstacle in the map (course_map.py), seen from above as a circle,
whatever its height. Distances are measured to the boulder's edge. Unlike the distances to walls
in course_map.py, which stop at zero, these go negative above a boulder: -0.1 means 10 cm in
from its edge. That lets the planner tell the edge of a boulder from its middle.

This file only measures. It does not decide how close is too close; route_planner.py does.
"""
import numpy as np

from team_nav.course_map import distance_to_line_piece_metres


def distance_to_nearest_boulder_metres(points_metres, boulders):
    """For an array of points seen from above (N x 2), returns each point's distance to the
    edge of the nearest boulder (negative above one), and that boulder's name. The distance
    is infinite, and the name empty, if there are no boulders."""
    points_metres = np.asarray(points_metres, dtype=float)
    nearest_distance_metres = np.full(len(points_metres), np.inf)
    nearest_name = np.full(len(points_metres), "", dtype=object)
    for boulder in boulders:
        distance_metres = np.linalg.norm(points_metres - boulder.centre_metres, axis=1) - boulder.radius_metres
        closer = distance_metres < nearest_distance_metres
        nearest_distance_metres[closer] = distance_metres[closer]
        nearest_name[closer] = boulder.name
    return nearest_distance_metres, nearest_name


def distance_to_each_boulder_metres(points_metres, boulders):
    """For an array of points seen from above (N x 2), returns an array (N points x number of
    boulders) of each point's distance to the edge of each boulder, negative above it."""
    points_metres = np.asarray(points_metres, dtype=float)
    distances_metres = np.empty((len(points_metres), len(boulders)))
    for boulder_number, boulder in enumerate(boulders):
        distances_metres[:, boulder_number] = np.linalg.norm(points_metres - boulder.centre_metres, axis=1) - boulder.radius_metres
    return distances_metres


def closest_approach_to_each_boulder_metres(boulders, start_metres, end_metres):
    """For a straight leg from start to end, returns a list of (boulder name, the closest the
    leg comes to that boulder's edge), one for every boulder, in the same order as boulders."""
    start_metres = np.asarray(start_metres, dtype=float)[:2]
    end_metres = np.asarray(end_metres, dtype=float)[:2]
    approaches = []
    for boulder in boulders:
        centre = np.asarray(boulder.centre_metres, dtype=float)[np.newaxis, :]
        distance_from_centre_metres = float(distance_to_line_piece_metres(centre, start_metres, end_metres)[0])
        approaches.append((boulder.name, distance_from_centre_metres - boulder.radius_metres))
    return approaches

