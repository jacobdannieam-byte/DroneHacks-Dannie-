"""Finds the tunnel's opening in a camera picture and in stereo depth points, for the tunnel_first
mission's LOOK step: how far the tunnel's centre is to the drone's left or right.

REAL DRONE CODE, also used in the simulator and the tests. It is plain arithmetic on NumPy arrays,
with no ROS. tunnel_look_camera.py feeds it pictures and points; tunnel_first_run.py acts on it.

Words
-----
"Rectified" picture: one corrected for the lens, so straight lines stay straight and a pixel's
column says its direction exactly. fx is the focal length in pixels and cx the column straight
ahead; both come from the camera's camera_info message, never from this file.
"Camera-left offset" e: how far the tunnel's centre line is to the left of the camera, in metres.
Positive is left.

Method 1, the edges (the team's design, 2026-11-08)
--------------------------------------------------
In a horizontal band across the middle of the picture, the tunnel's two inner edges (where its
inside surface meets the rim) are near-vertical lines. A column is "supported" if most rows of the
band have a strong left-right brightness change there (a Sobel filter): random texture rarely lines
up over many rows as the strongest change among its neighbours; an edge does. The left and right edges uL and uR are the WIDEST pair of supported
columns, one in each part of the picture, whose spacing fits a tunnel between NEAREST_Z_METRES and
FARTHEST_Z_METRES away and that passes the depth check below. Widest, because the mouth's rim is the
outermost tunnel edge in view; the far exit's edges and the inside's texture lie within it (Gazebo,
2026-11-08). Without depth points (the hand test before depth arrives, the tests) the best supported
pair is taken instead. Then, with W = TUNNEL_INSIDE_WIDTH_METRES:
    w = uR - uL                 the opening's width in pixels
    uc = (uL + uR) / 2          its centre column
    Z = fx * W / w              its distance from the camera
    e = -W * (uc - cx) / w      the camera-left offset (a centre right of cx is to the right)
If no edge is found in one part of the picture but one is in the other, that edge is "clipped" by
the picture's border: the tunnel is too far to that side, or too close.

Method 2, the stereo depth points (a second opinion)
---------------------------------------------------
Points within CAMERA_HEIGHT_BAND_METRES of the camera's height and between Z and
Z + DEPTH_LOOK_LENGTH_METRES ahead lie on the tunnel's inside walls. L is the median distance to
the left of those on the left, R the median distance to the right of those on the right, and the
offset is (L - R) / 2. If the two methods differ by more than LARGEST_DISAGREEMENT_METRES, the
look is "unsure".

The depth check of every frame's edges
--------------------------------------
Gazebo, 2026-11-08: at S the tunnel's mouth was wider than the picture, and the edge finder took
the far exit's left edge and the tunnel's outside on the right as a pair: 316 px, "1.48 m away",
offset the wrong way. So each frame's pair must also match the stereo depth: the median distance
of the depth points near each edge column (within EDGE_DEPTH_COLUMN_PIXELS, near the camera's
height; the nearer quarter of them, EDGE_DEPTH_PERCENTILE, because an edge belongs to the nearer
surface and the background shows just beside it) must be within EDGE_DEPTH_TOLERANCE_FRACTION of
Z = fx * W / w. A pair with no depth
points at an edge, or a mismatch, is not a tunnel opening at that distance, and the frame counts
as showing no edges.

Accepting a look
----------------
LOOK_FRAMES_NEEDED frames within about LOOK_WINDOW_SECONDS, whose centres differ by at most
LARGEST_CENTRE_JITTER_PIXELS and widths by at most LARGEST_WIDTH_CHANGE_FRACTION, with Z between
NEAREST_Z_METRES and FARTHEST_Z_METRES.

What it does not do: move the drone, or subscribe to anything.
"""
from dataclasses import dataclass

import numpy as np

# From the organizers' drawing (the tube "58 x 72 x 60" inches): the inside is about 1.47 m wide.
TUNNEL_INSIDE_WIDTH_METRES = 1.47
# The team's choices (2026-11-08).
NEAREST_Z_METRES = 0.8
FARTHEST_Z_METRES = 1.9
CAMERA_HEIGHT_BAND_METRES = 0.3
DEPTH_LOOK_LENGTH_METRES = 1.5
LARGEST_DISAGREEMENT_METRES = 0.10
LOOK_FRAMES_NEEDED = 5
LOOK_WINDOW_SECONDS = 1.0
LARGEST_CENTRE_JITTER_PIXELS = 5.0
LARGEST_WIDTH_CHANGE_FRACTION = 0.05
# Our own choices for the edge finder (tuned on Gazebo pictures, 2026-11-08, where the rim's edge is
# faint and slightly curved; every pair must then pass the depth check, which removes false ones):
# the band is this fraction of the picture's height, centred;
# a column is supported if this fraction of the band's rows has a strong change there; "strong" is
# this many times the band's median change.
BAND_HEIGHT_FRACTION = 0.15
SUPPORTED_ROW_FRACTION = 0.5
STRONG_CHANGE_TIMES_MEDIAN = 3.0
# Our own choice: columns this close to the picture's border can't hold a trustworthy edge.
BORDER_PIXELS = 4
# Our own choice: a change must be the largest within this many columns on each side to count.
LOCAL_PEAK_PIXELS = 3
EDGE_WANDER_PIXELS = 2
# Our own choice: the stereo method needs at least this many points on each wall.
LEAST_POINTS_PER_WALL = 20
# Our own choices for the depth check of each frame's edges (see above).
EDGE_DEPTH_COLUMN_PIXELS = 6.0
EDGE_DEPTH_TOLERANCE_FRACTION = 0.20
LEAST_POINTS_PER_EDGE = 3
EDGE_DEPTH_PERCENTILE = 25


@dataclass
class EdgeLook:
    """One picture's answer. uL and uR are None when not found."""
    left_column: object = None
    right_column: object = None
    clipped_left: bool = False
    clipped_right: bool = False
    width_pixels: float = float("nan")
    centre_column: float = float("nan")
    distance_metres: float = float("nan")
    left_offset_metres: float = float("nan")
    band_rows: tuple = (0, 0)
    support: object = None          # The band's supported fraction for every column, for the PNG.
    rejected_because: str = ""      # Why the widest pairs failed the depth check, if they did.

    def found_both(self):
        return self.left_column is not None and self.right_column is not None


def middle_band_rows(picture_height):
    band = max(3, int(round(picture_height * BAND_HEIGHT_FRACTION)))
    top = (picture_height - band) // 2
    return top, top + band


def column_support(gray_band):
    """For each column, the fraction of the band's rows with a strong left-right change there."""
    band = np.asarray(gray_band, dtype=np.float32)
    # Sobel's left-right filter: [-1 0 1] across, smoothed [1 2 1] down.
    padded = np.pad(band, 1, mode="edge")
    across = padded[:, 2:] - padded[:, :-2]
    change = np.abs(across[:-2] + 2.0 * across[1:-1] + across[2:])
    threshold = STRONG_CHANGE_TIMES_MEDIAN * max(float(np.median(change)), 1.0)
    # A real edge is the strongest change among its neighbours in nearly every row; a texture peak
    # wanders from row to row. So a row counts for a column only where the change there is strong
    # and the largest within LOCAL_PEAK_PIXELS on each side.
    padded_change = np.pad(change, ((0, 0), (LOCAL_PEAK_PIXELS, LOCAL_PEAK_PIXELS)), mode="constant")
    neighbourhood_largest = np.zeros_like(change)
    for shift in range(2 * LOCAL_PEAK_PIXELS + 1):
        neighbourhood_largest = np.maximum(neighbourhood_largest, padded_change[:, shift:shift + change.shape[1]])
    peak = (change > threshold) & (change >= neighbourhood_largest)
    # The rim is part of an oval, so its edge may wander between rows: count a row if the peak is
    # within EDGE_WANDER_PIXELS columns.
    near = peak.copy()
    for shift in range(1, EDGE_WANDER_PIXELS + 1):
        near[:, shift:] |= peak[:, :-shift]
        near[:, :-shift] |= peak[:, shift:]
    return near.mean(axis=0)


def find_opening_edges(gray_picture, fx_pixels, cx_pixels, points_from_camera=None, check_depth=False):
    """The edge method on one rectified picture (height x width brightness). With check_depth, the
    pair must also pass edges_match_depth with these points (from the camera)."""
    gray = np.asarray(gray_picture, dtype=np.float32)
    height, width = gray.shape
    top, bottom = middle_band_rows(height)
    support = column_support(gray[top:bottom])
    support[:BORDER_PIXELS] = 0.0
    support[width - BORDER_PIXELS:] = 0.0
    look = EdgeLook(band_rows=(top, bottom), support=support)
    narrowest = fx_pixels * TUNNEL_INSIDE_WIDTH_METRES / FARTHEST_Z_METRES
    widest = fx_pixels * TUNNEL_INSIDE_WIDTH_METRES / NEAREST_Z_METRES
    candidates = np.nonzero(support >= SUPPORTED_ROW_FRACTION)[0]
    left_candidates = candidates[candidates < cx_pixels]
    right_candidates = candidates[candidates > cx_pixels]
    pairs = _pairs_widest_first(left_candidates, right_candidates, narrowest, widest)
    if not check_depth:
        # Without depth points to weed out texture, the best supported pair (texture rarely lines up
        # over the whole band as well as a real edge does).
        pairs.sort(key=lambda pair: -(support[pair[0]] + support[pair[1]]))
    if not pairs:
        look.clipped_left = len(left_candidates) == 0 and len(right_candidates) > 0
        look.clipped_right = len(right_candidates) == 0 and len(left_candidates) > 0
        return look
    if not check_depth:
        look.left_column, look.right_column = pairs[0]
        _measure(look, fx_pixels, cx_pixels)
        return look
    depths = edge_depths_metres(points_from_camera, np.r_[left_candidates, right_candidates], fx_pixels, cx_pixels)
    for left, right in pairs:
        look.left_column, look.right_column = left, right
        _measure(look, fx_pixels, cx_pixels)
        matches, why = _depths_match(look, depths.get(left), depths.get(right))
        if matches:
            return look
        look.rejected_because = look.rejected_because or f"widest pair {left}-{right}: {why}"
    rejected = look.rejected_because
    look = EdgeLook(band_rows=(top, bottom), support=support, rejected_because=rejected)
    return look


def _pairs_widest_first(left_candidates, right_candidates, narrowest, widest):
    """Every (left, right) pair of supported columns whose spacing fits, widest first. Neighbouring
    columns of one edge make near-duplicate pairs; only pairs at least 3 px apart from the last
    kept one on both sides are kept."""
    pairs = []
    for left in left_candidates:
        for right in right_candidates:
            if narrowest <= right - left <= widest:
                pairs.append((int(left), int(right)))
    pairs.sort(key=lambda pair: pair[0] - pair[1])
    kept = []
    for pair in pairs:
        if all(abs(pair[0] - other[0]) >= 3 or abs(pair[1] - other[1]) >= 3 for other in kept):
            kept.append(pair)
    return kept


def _measure(look, fx_pixels, cx_pixels):
    look.width_pixels = float(look.right_column - look.left_column)
    look.centre_column = (look.left_column + look.right_column) / 2.0
    look.distance_metres = fx_pixels * TUNNEL_INSIDE_WIDTH_METRES / look.width_pixels
    look.left_offset_metres = -TUNNEL_INSIDE_WIDTH_METRES * (look.centre_column - cx_pixels) / look.width_pixels


def edge_depths_metres(points_from_camera, edge_columns, fx_pixels, cx_pixels):
    """{column: the nearer-quarter distance of the depth points near that column}, for the columns
    with at least LEAST_POINTS_PER_EDGE points near them. Empty without points."""
    if points_from_camera is None or len(points_from_camera) == 0:
        return {}
    points = np.asarray(points_from_camera, dtype=float)
    in_front = (points[:, 0] > 0.1) & (np.abs(points[:, 2]) <= CAMERA_HEIGHT_BAND_METRES)
    points = points[in_front]
    columns = cx_pixels - fx_pixels * points[:, 1] / points[:, 0]
    depths = {}
    for edge_column in set(int(column) for column in edge_columns):
        near_edge = np.abs(columns - edge_column) <= EDGE_DEPTH_COLUMN_PIXELS
        if np.count_nonzero(near_edge) >= LEAST_POINTS_PER_EDGE:
            depths[edge_column] = float(np.percentile(points[near_edge, 0], EDGE_DEPTH_PERCENTILE))
    return depths


def edges_match_depth(look, points_from_camera, fx_pixels, cx_pixels):
    """(True, why) if the depth points at both edge columns are about Z away, else (False, why)."""
    if points_from_camera is None or len(points_from_camera) == 0:
        return False, "no stereo depth points to check the edges with"
    depths = edge_depths_metres(points_from_camera, [look.left_column, look.right_column], fx_pixels, cx_pixels)
    return _depths_match(look, depths.get(look.left_column), depths.get(look.right_column))


def _depths_match(look, left_depth_metres, right_depth_metres):
    for name, edge_distance in (("left", left_depth_metres), ("right", right_depth_metres)):
        if edge_distance is None:
            return False, f"no depth points at the {name} edge"
        if abs(edge_distance - look.distance_metres) > EDGE_DEPTH_TOLERANCE_FRACTION * look.distance_metres:
            return False, (f"the {name} edge is {edge_distance:.2f} m away by depth, but the width says "
                           f"{look.distance_metres:.2f} m")
    return True, "both edges are at the width's distance"


def depth_left_offset_metres(points_from_camera, distance_metres):
    """The stereo method. points_from_camera is N x 3 (forward, left, up) in metres from the camera.
    Returns (offset, L, R), or (None, L, R) if a wall has too few points."""
    points = np.asarray(points_from_camera, dtype=float)
    if points.size == 0:
        return None, None, None
    near_camera_height = np.abs(points[:, 2]) <= CAMERA_HEIGHT_BAND_METRES
    ahead = (points[:, 0] >= distance_metres) & (points[:, 0] <= distance_metres + DEPTH_LOOK_LENGTH_METRES)
    chosen = points[near_camera_height & ahead]
    left_side = chosen[chosen[:, 1] > 0.0, 1]
    right_side = -chosen[chosen[:, 1] < 0.0, 1]
    if len(left_side) < LEAST_POINTS_PER_WALL or len(right_side) < LEAST_POINTS_PER_WALL:
        return None, None, None
    left_metres, right_metres = float(np.median(left_side)), float(np.median(right_side))
    return (left_metres - right_metres) / 2.0, left_metres, right_metres


@dataclass
class LookVerdict:
    accepted: bool
    reason: str
    left_offset_metres: float = float("nan")      # The edge method's average, when accepted.
    distance_metres: float = float("nan")
    clipped_side: object = None                   # "left", "right" or None.


def judge_frames(looks, stamps_seconds):
    """Decides whether the latest LOOK_FRAMES_NEEDED edge looks, taken within about
    LOOK_WINDOW_SECONDS, agree well enough to trust."""
    found = [(look, stamp) for look, stamp in zip(looks, stamps_seconds) if look.found_both()]
    clipped = _clipped_side(looks[-LOOK_FRAMES_NEEDED:])
    if len(found) < LOOK_FRAMES_NEEDED:
        return LookVerdict(False, f"only {len(found)} of the frames show both edges", clipped_side=clipped)
    recent = found[-LOOK_FRAMES_NEEDED:]
    span_seconds = recent[-1][1] - recent[0][1]
    if span_seconds > LOOK_WINDOW_SECONDS * 1.5:
        return LookVerdict(False, f"the last {LOOK_FRAMES_NEEDED} good frames span {span_seconds:.1f} s")
    centres = np.array([look.centre_column for look, _ in recent])
    widths = np.array([look.width_pixels for look, _ in recent])
    distances = np.array([look.distance_metres for look, _ in recent])
    if centres.max() - centres.min() > LARGEST_CENTRE_JITTER_PIXELS:
        return LookVerdict(False, f"the centre moved {centres.max() - centres.min():.1f} px between frames")
    if (widths.max() - widths.min()) / widths.mean() > LARGEST_WIDTH_CHANGE_FRACTION:
        return LookVerdict(False, f"the width changed by {100 * (widths.max() - widths.min()) / widths.mean():.0f}%")
    distance = float(np.mean(distances))
    if not NEAREST_Z_METRES <= distance <= FARTHEST_Z_METRES:
        return LookVerdict(False, f"the tunnel looks {distance:.2f} m away, outside {NEAREST_Z_METRES} to {FARTHEST_Z_METRES} m")
    offset = float(np.mean([look.left_offset_metres for look, _ in recent]))
    return LookVerdict(True, "steady", offset, distance)


def _clipped_side(looks):
    lefts = sum(1 for look in looks if look.clipped_left)
    rights = sum(1 for look in looks if look.clipped_right)
    if lefts > len(looks) / 2:
        return "left"
    if rights > len(looks) / 2:
        return "right"
    return None


def body_left_on_course(course_heading_radians):
    """The drone's left, as an (east, north) direction on the course, for a nose heading measured
    from east toward north. The team's formula: left = (-sin(theta), cos(theta))."""
    return np.array([-np.sin(course_heading_radians), np.cos(course_heading_radians)])


def band_picture_with_edges(gray_picture, look):
    """The picture's band as a colour (height x width x 3, red-green-blue) array, with the chosen
    edges drawn in red and the support profile in green along the bottom, for the records."""
    top, bottom = look.band_rows
    band = np.clip(np.asarray(gray_picture, dtype=np.float32)[top:bottom], 0, 255).astype(np.uint8)
    colour = np.stack([band, band, band], axis=2).copy()
    if look.support is not None:
        bar_rows = max(1, (bottom - top) // 4)
        heights = (look.support * bar_rows).astype(int)
        for column, filled in enumerate(heights):
            if filled > 0:
                colour[-filled:, column] = (0, 200, 0)
    for column in (look.left_column, look.right_column):
        if column is not None:
            colour[:, max(column - 1, 0):column + 2] = (255, 0, 0)
    return colour
