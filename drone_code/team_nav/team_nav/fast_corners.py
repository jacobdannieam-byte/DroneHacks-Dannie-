"""Counts FAST corners in a grey picture: the small high-contrast spots that camera tracking
follows. A covered lens, darkness or a blank wall gives almost none.

How it fits: corner_count_node.py counts the corners in the left camera's picture twice a second;
mission.py uses the count to tell a blind camera from a drone that is simply standing still
(prestart_check.py). This file is plain numpy, so it works even if OpenCV is missing on the drone.

The test ("FAST-9", Rosten and Drummond 2006): look at the 16 pixels on a circle of radius 3
around a pixel. It is a corner if 9 of them in an unbroken run are all brighter than it by more
than the threshold, or all darker by more than the threshold.

What it deliberately does not do: keep only the strongest pixel of each corner (OpenCV's
"non-maximum suppression"). One corner can count as a few neighbouring pixels, so counts here are
larger than OpenCV's for the same picture. The blind threshold (prestart_blind_below_corners) is
for this count.
"""
import numpy as np

# The 16 pixels on a circle of radius 3, in order round the circle, as (row step, column step).
CIRCLE = [(-3, 0), (-3, 1), (-2, 2), (-1, 3), (0, 3), (1, 3), (2, 2), (3, 1),
          (3, 0), (3, -1), (2, -2), (1, -3), (0, -3), (-1, -3), (-2, -2), (-3, -1)]
RADIUS_PIXELS = 3
# From the FAST-9 test: this many neighbours in an unbroken run.
RUN_NEEDED = 9


def count_fast_corners(grey, threshold):
    """How many pixels of the grey picture pass the FAST-9 test with this brightness threshold."""
    grey = np.asarray(grey, dtype=np.int16)
    height, width = grey.shape
    if height <= 2 * RADIUS_PIXELS or width <= 2 * RADIUS_PIXELS:
        return 0
    centre = grey[RADIUS_PIXELS:height - RADIUS_PIXELS, RADIUS_PIXELS:width - RADIUS_PIXELS]
    brighter, darker = [], []
    for row_step, column_step in CIRCLE:
        ring = grey[RADIUS_PIXELS + row_step:height - RADIUS_PIXELS + row_step,
                    RADIUS_PIXELS + column_step:width - RADIUS_PIXELS + column_step]
        brighter.append(ring > centre + threshold)
        darker.append(ring < centre - threshold)
    corners = longest_unbroken_run(brighter) >= RUN_NEEDED
    corners |= longest_unbroken_run(darker) >= RUN_NEEDED
    return int(np.count_nonzero(corners))


def longest_unbroken_run(on_circle):
    """For every pixel, the longest run of True round its circle. The circle wraps round, so the
    walk goes round once and then far enough again to finish a run that crosses the start."""
    run = np.zeros(on_circle[0].shape, dtype=np.int16)
    longest = np.zeros(on_circle[0].shape, dtype=np.int16)
    for step in range(len(on_circle) + RUN_NEEDED - 1):
        run = (run + 1) * on_circle[step % len(on_circle)]
        np.maximum(longest, run, out=longest)
    return longest
