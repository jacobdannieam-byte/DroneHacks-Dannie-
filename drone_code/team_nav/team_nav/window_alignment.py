"""Locate a mapped window in stereo depth points and measure tracking translation error.

Real flight arithmetic, with no ROS or simulator interfaces. The flight controller supplies
time-matched points in course coordinates. Missing depth never counts as an opening by
itself: both jambs must have dense wall returns on the same measured plane.
"""
from dataclasses import dataclass
import numpy as np

# Our choices: reject large map mismatches; use nearby wall returns around the aperture.
MAXIMUM_TRANSLATION_METRES = .40
PLANE_BIN_METRES = .01
PLANE_BAND_METRES = .025
MINIMUM_WALL_POINTS = 80
MINIMUM_JAMB_POINTS = 15
EDGE_BIN_METRES = .008
# Our choice: twelve returns per 8 mm strip reject sparse stereo edge smearing.
# Recorded window clouds have tens of returns per genuine jamb strip. The separate
# flight obstacle guard still uses its stricter four-point obstacle trigger.
MINIMUM_BIN_POINTS = 12
WIDTH_ERROR_LIMIT_METRES = .025
CENTRE_HEIGHT_BAND_METRES = .12


@dataclass
class WindowObservation:
    correction_metres: np.ndarray
    width_metres: float
    plane_spread_metres: float
    wall_points: int


def observe_window(points, wall):
    """Return a measured map-frame correction, or None if either jamb is unobserved."""
    opening = wall.openings[0]
    along = wall.direction()
    normal = np.array([-along[1], along[0]])
    centre_along = (opening.start_along_wall_metres + opening.end_along_wall_metres) / 2
    centre = wall.point_along_wall_metres(centre_along)
    height = (opening.bottom_height_metres + opening.top_height_metres) / 2
    points = np.asarray(points)
    if len(points) < MINIMUM_WALL_POINTS:
        return None
    coordinates = points[:, :2] - centre
    sideways, forward = coordinates @ along, coordinates @ normal
    nearby = (np.isfinite(points).all(axis=1) & (np.abs(sideways) < .85)
              & (np.abs(forward) < MAXIMUM_TRANSLATION_METRES)
              & (np.abs(points[:, 2]-height) < CENTRE_HEIGHT_BAND_METRES))
    sideways, forward = sideways[nearby], forward[nearby]
    if len(forward) < MINIMUM_WALL_POINTS:
        return None
    bins = np.arange(-MAXIMUM_TRANSLATION_METRES, MAXIMUM_TRANSLATION_METRES + PLANE_BIN_METRES, PLANE_BIN_METRES)
    counts, edges = np.histogram(forward, bins)
    peak = int(np.argmax(counts))
    plane = (edges[peak]+edges[peak+1])/2
    on_plane = np.abs(forward-plane) < PLANE_BAND_METRES
    sideways, forward = sideways[on_plane], forward[on_plane]
    if len(forward) < MINIMUM_WALL_POINTS:
        return None
    gap = observed_gap(sideways, opening.end_along_wall_metres-opening.start_along_wall_metres)
    if gap is None:
        return None
    left, right = gap
    # The approach sees the near face, half a wall thickness before its centre plane.
    plane_error = float(np.median(forward)) + wall.thickness_metres/2
    along_error = (left+right)/2
    correction = -plane_error*normal - along_error*along
    if np.linalg.norm(correction) > MAXIMUM_TRANSLATION_METRES:
        return None
    spread = float(np.quantile(np.abs(forward-np.median(forward)), .95))
    return WindowObservation(correction, right-left, spread, len(forward))


def observed_gap(sideways, expected_width):
    """Find a bounded gap with observed wall on each side, rejecting sparse speckles."""
    bins = np.arange(-.85, .85+EDGE_BIN_METRES, EDGE_BIN_METRES)
    counts, edges = np.histogram(sideways, bins)
    occupied = np.flatnonzero(counts >= MINIMUM_BIN_POINTS)
    candidates = []
    for left_bin, right_bin in zip(occupied[:-1], occupied[1:]):
        left, right = edges[left_bin+1], edges[right_bin]
        if abs((right-left)-expected_width) > WIDTH_ERROR_LIMIT_METRES:
            continue
        left_support = np.count_nonzero((sideways >= left-.12) & (sideways <= left))
        right_support = np.count_nonzero((sideways >= right) & (sideways <= right+.12))
        if min(left_support, right_support) < MINIMUM_JAMB_POINTS:
            continue
        candidates.append((abs((left+right)/2),left,right))
    if not candidates:
        return None
    _, left, right = min(candidates)
    return left, right
