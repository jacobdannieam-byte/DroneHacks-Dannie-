"""Turns a pair of stereo camera pictures into distances: how far away each thing in the picture is.

REAL DRONE CODE, also used unchanged in the simulator.

How stereo depth works, in one paragraph
----------------------------------------
The drone's camera (an OAK-D) has two eyes side by side, 7.5 cm apart. A near object appears
in a different place in the left and right pictures; a far object appears in nearly the same
place. The shift, in pixels, is called the "disparity". Distance = focal length x eye spacing /
disparity. So for each small patch of the left picture, we look for the same patch in the right
picture, a few pixels to the left, and keep the shift that matches best.

How we find the best match
--------------------------
For every shift from 1 to LARGEST_SHIFT_PIXELS we compare each 9 x 9 patch of the left picture
with the patch that many pixels to the left in the right picture, and add up the differences
("sum of absolute differences"). Before comparing, each picture has its local average brightness
removed, so the two eyes can have slightly different exposure. The shift with the smallest
difference wins. A pixel is thrown away when:
  - its patch has almost no texture (a plain wall or a dark corner can match anywhere), or
  - the best shift is not clearly better than the second best (an unsure match), or
  - matching the other way, from the right picture to the left, gives a different shift (the
    "left-right check"), or
  - it belongs to a small speck: fewer than SMALLEST_SPECK_PIXELS neighbouring pixels with about
    the same shift. Real walls, poles and people make large patches; wrong matches make specks.
Thrown-away pixels have no distance (NaN). Fewer, trustworthy distances are better than many
wrong ones, because a wrong near distance makes the drone stop or turn for nothing.

Why the last two checks exist: on 2026-11-08 every depth picture in flight had thousands of
wrong near points, scattered as small white specks. The live map blocks a square after two hits,
so the drone found no path and turned on the spot or went the wrong way. On 20 picture pairs
taken in flight (record 20261108_010857), the two checks removed about 90% of the points nearer
than 1.2 m, and kept the net, the poles and the people.

Two ways to run the same calculation
------------------------------------
On the drone, PyTorch runs it on the Jetson's graphics processor (GPU), in a few milliseconds.
Where PyTorch is missing (the simulator box), NumPy runs the identical steps on the main
processor, more slowly. Both give the same answer. test_live_navigation.py checks the NumPy
version; we have not written a test that compares the two.

What it does not do: read ROS messages, or decide anything. depth_camera.py does the ROS part.
"""
import math

import numpy as np

# Our own choice. We shrink pictures to this width before matching. Full width (1024 pixels)
# costs 4 times as much and the extra detail doesn't help us: we only need to see walls.
WORKING_WIDTH_PIXELS = 512

# Our own choice. At 512 pixels wide the focal length is about 317 pixels (the real camera's
# calibration, 634.5 pixels at 1024 wide, halved). With the 7.5 cm eye spacing, a 64-pixel shift
# is 317 x 0.0751 / 64 = 0.37 m. Nearer than that we can't measure, which is fine: the drone
# must never be that close to anything.
LARGEST_SHIFT_PIXELS = 64

# Our own choice. The patch compared between the two pictures is this many pixels wide and high.
# Larger patches are more reliable on plain surfaces but blur the edges of thin objects.
PATCH_SIZE_PIXELS = 9

# Our own choice. Brightness varies 0 to 255. A patch whose brightness varies less than this on
# average (after removing its average) is too plain to match.
SMALLEST_TEXTURE = 4.0

# Our own choice. The best match must be at least this much better than the second best match
# (more than one pixel away from it), or we don't trust it.
UNIQUENESS_RATIO = 0.9

# Our own choice. The left-right check: matching from the right picture must give a shift within
# this many pixels of the shift found from the left picture.
LEFT_RIGHT_AGREEMENT_PIXELS = 1.0

# Our own choice, checked on flight pictures of 2026-11-08 (see the top of this file). A group of
# neighbouring pixels with about the same shift must have at least this many pixels, out of the
# 512 x 320 working picture. A 0.5 m wide wall 3 m away is about 50 pixels wide, so it passes.
SMALLEST_SPECK_PIXELS = 150
# Our own choice. "About the same shift": two touching pixels are in the same group if their
# shifts differ by at most this much. A wall seen at an angle changes its shift slowly.
SPECK_NEIGHBOUR_SHIFT_PIXELS = 2
# Marks "no shift" for OpenCV's speck filter: -1 pixel, in sixteenths. Real shifts are positive.
NO_SHIFT_SIXTEENTHS = -16


def gray_from_colour(picture):
    """A colour picture (height x width x 3, blue-green-red) as one brightness channel, 0 to 255."""
    if picture.ndim == 2:
        return picture.astype(np.float32)
    blue, green, red = picture[..., 0], picture[..., 1], picture[..., 2]
    return (0.114 * blue + 0.587 * green + 0.299 * red).astype(np.float32)


def shrink_to_working_width(gray_picture):
    """Shrinks a picture by a whole factor so that it is about WORKING_WIDTH_PIXELS wide.
    Returns the picture and the factor (1 means unchanged)."""
    factor = max(1, int(round(gray_picture.shape[1] / WORKING_WIDTH_PIXELS)))
    if factor == 1:
        return gray_picture, 1
    height = gray_picture.shape[0] // factor * factor
    width = gray_picture.shape[1] // factor * factor
    blocks = gray_picture[:height, :width].reshape(height // factor, factor, width // factor, factor)
    return blocks.mean(axis=(1, 3)), factor


def distances_metres(left_gray, right_gray, focal_length_pixels, eye_spacing_metres, use_gpu=True):
    """The distance straight ahead of the camera, in metres, for every pixel of the (already
    shrunk) left picture, or NaN where there is no trustworthy match. focal_length_pixels must
    match the shrunk pictures."""
    disparity = disparity_pixels(left_gray, right_gray, use_gpu)
    with np.errstate(divide="ignore", invalid="ignore"):
        return focal_length_pixels * eye_spacing_metres / disparity


def disparity_pixels(left_gray, right_gray, use_gpu=True):
    """The shift between the two pictures, in pixels with fractions, or NaN where untrusted."""
    torch = _torch_with_gpu() if use_gpu else None
    if torch is not None:
        disparity = _disparity_with_torch(torch, left_gray, right_gray)
    else:
        disparity = _disparity_with_numpy(left_gray, right_gray)
    return _without_small_specks(disparity)


def which_calculator():
    """'GPU' or 'NumPy', for the records."""
    return "GPU" if _torch_with_gpu() is not None else "NumPy"


# ---- The NumPy version ---------------------------------------------------------------------------

def _disparity_with_numpy(left_gray, right_gray):
    left = _remove_local_average_numpy(left_gray)
    right = _remove_local_average_numpy(right_gray)
    height, width = left.shape
    costs = np.full((LARGEST_SHIFT_PIXELS + 1, height, width), np.inf, dtype=np.float32)
    for shift in range(1, LARGEST_SHIFT_PIXELS + 1):
        difference = np.abs(left[:, shift:] - right[:, :-shift])
        costs[shift, :, shift:] = _patch_sum_numpy(difference)
    texture = _patch_sum_numpy(np.abs(left)) / PATCH_SIZE_PIXELS ** 2
    return _best_shift(costs, texture, np)


def _remove_local_average_numpy(gray):
    return gray - _patch_sum_numpy(gray) / PATCH_SIZE_PIXELS ** 2


def _patch_sum_numpy(values):
    """The sum over the PATCH_SIZE x PATCH_SIZE square around each pixel, same size as the input.
    Uses running totals (an "integral image"), so the cost doesn't depend on the patch size."""
    half = PATCH_SIZE_PIXELS // 2
    padded = np.pad(values, half, mode="edge")
    totals = np.zeros((padded.shape[0] + 1, padded.shape[1] + 1), dtype=np.float64)
    totals[1:, 1:] = padded.cumsum(axis=0).cumsum(axis=1)
    size = PATCH_SIZE_PIXELS
    sums = totals[size:, size:] - totals[:-size, size:] - totals[size:, :-size] + totals[:-size, :-size]
    return sums.astype(np.float32)


# ---- The PyTorch (GPU) version ------------------------------------------------------------------

def _torch_with_gpu():
    try:
        import torch
    except ImportError:
        return None
    return torch if torch.cuda.is_available() else None


def _disparity_with_torch(torch, left_gray, right_gray):
    """The same steps as the NumPy version, all on the GPU. Only the final shift picture comes
    back to the main processor. Measured on drone 16 on 2026-11-08: sending the whole cost table
    back and choosing on the main processor took about 150 ms a picture."""
    device = torch.device("cuda")
    # PyTorch needs the pixels stored in plain order (a flipped or cut NumPy view isn't).
    left_pixels = np.ascontiguousarray(left_gray, dtype=np.float32)
    right_pixels = np.ascontiguousarray(right_gray, dtype=np.float32)
    left = _remove_local_average_torch(torch, torch.as_tensor(left_pixels, device=device))
    right = _remove_local_average_torch(torch, torch.as_tensor(right_pixels, device=device))
    height, width = left.shape
    costs = torch.full((LARGEST_SHIFT_PIXELS + 1, height, width), float("inf"), device=device)
    for shift in range(1, LARGEST_SHIFT_PIXELS + 1):
        difference = (left[:, shift:] - right[:, :-shift]).abs()
        costs[shift, :, shift:] = _patch_sum_torch(torch, difference)
    texture = _patch_sum_torch(torch, left.abs()) / PATCH_SIZE_PIXELS ** 2
    return _best_shift_torch(torch, costs, texture)


def _best_shift_torch(torch, costs, texture):
    """_best_shift and _fraction_of_a_pixel, written with PyTorch so they run on the GPU."""
    last = costs.shape[0] - 1
    best_cost, best = costs.min(dim=0)
    masked = costs.clone()
    for offset in (-1, 0, 1):
        masked.scatter_(0, (best + offset).clamp(0, last)[None], float("inf"))
    second_best_cost = masked.min(dim=0).values
    trusted = (best >= 1) & (best < last) & (texture >= SMALLEST_TEXTURE)
    trusted &= best_cost < UNIQUENESS_RATIO * second_best_cost
    trusted &= _left_and_right_agree_torch(torch, costs, best)
    before = costs.gather(0, (best - 1).clamp(0, last)[None])[0]
    after = costs.gather(0, (best + 1).clamp(0, last)[None])[0]
    curvature = before - 2.0 * best_cost + after
    fraction = 0.5 * (before - after) / curvature
    fraction = torch.where(torch.isfinite(fraction), fraction, torch.zeros_like(fraction)).clamp(-0.5, 0.5)
    disparity = best.float() + fraction
    disparity[~trusted] = float("nan")
    return disparity.cpu().numpy()


def _left_and_right_agree_torch(torch, costs, best):
    """The same left-right check as _left_and_right_agree_numpy, on the GPU."""
    shift_count, height, width = costs.shape
    costs_from_right = torch.full_like(costs, float("inf"))
    for shift in range(1, shift_count):
        costs_from_right[shift, :, :width - shift] = costs[shift, :, shift:]
    best_from_right = costs_from_right.argmin(dim=0)
    columns = torch.arange(width, device=costs.device).expand(height, width)
    matched_columns = (columns - best).clamp(0, width - 1)
    return (best_from_right.gather(1, matched_columns) - best).abs() <= LEFT_RIGHT_AGREEMENT_PIXELS


def _remove_local_average_torch(torch, gray):
    return gray - _patch_sum_torch(torch, gray) / PATCH_SIZE_PIXELS ** 2


def _patch_sum_torch(torch, values):
    """The same patch sum as _patch_sum_numpy, with edge padding, on the GPU."""
    half = PATCH_SIZE_PIXELS // 2
    padded = torch.nn.functional.pad(values[None, None], (half, half, half, half), mode="replicate")
    averages = torch.nn.functional.avg_pool2d(padded, PATCH_SIZE_PIXELS, stride=1)
    return averages[0, 0] * PATCH_SIZE_PIXELS ** 2


# ---- Choosing the best shift (shared) ---------------------------------------------------------

def _best_shift(costs, texture, numbers):
    """From the matching costs (shift x height x width), the best shift per pixel, refined to a
    fraction of a pixel, or NaN where the match is untrustworthy."""
    best = numbers.argmin(costs, axis=0)
    rows, columns = numbers.indices(best.shape)
    best_cost = costs[best, rows, columns]
    # The second best cost, ignoring the best shift and its two neighbours (they are always close).
    masked = costs.copy()
    for offset in (-1, 0, 1):
        neighbour = numbers.clip(best + offset, 0, costs.shape[0] - 1)
        masked[neighbour, rows, columns] = numbers.inf
    second_best_cost = masked.min(axis=0)
    trusted = (best >= 1) & (best < costs.shape[0] - 1) & (texture >= SMALLEST_TEXTURE)
    trusted &= best_cost < UNIQUENESS_RATIO * second_best_cost
    trusted &= _left_and_right_agree_numpy(costs, best, rows, columns)
    disparity = best.astype(np.float32) + _fraction_of_a_pixel(costs, best, rows, columns)
    disparity[~trusted] = np.nan
    return disparity


def _left_and_right_agree_numpy(costs, best, rows, columns):
    """The left-right check. costs[shift, row, column] compares left pixel `column` with right
    pixel `column - shift`. So for right pixel x, its costs are costs[shift, row, x + shift]. We
    find each right pixel's best shift, and keep a left pixel only if the right pixel it matched
    chose (about) the same shift back."""
    shift_count, _height, width = costs.shape
    costs_from_right = np.full_like(costs, np.inf)
    for shift in range(1, shift_count):
        costs_from_right[shift, :, :width - shift] = costs[shift, :, shift:]
    best_from_right = costs_from_right.argmin(axis=0)
    matched_columns = np.clip(columns - best, 0, width - 1)
    return np.abs(best_from_right[rows, matched_columns] - best) <= LEFT_RIGHT_AGREEMENT_PIXELS


def _without_small_specks(disparity):
    """Removes groups of neighbouring pixels, with about the same shift, that are smaller than
    SMALLEST_SPECK_PIXELS. Done on the main processor for both versions, with OpenCV's
    filterSpeckles (installed with the organizers' drone software), which is written in C++ and
    fast. OpenCV wants whole numbers, so the shifts are stored in sixteenths of a pixel, as
    OpenCV's own stereo code does; pixels without a shift get a value OpenCV skips."""
    import cv2  # Imported here so that the rest of this file works without OpenCV.
    sixteenths = np.full(disparity.shape, NO_SHIFT_SIXTEENTHS, dtype=np.int16)
    has_shift = np.isfinite(disparity)
    sixteenths[has_shift] = np.round(disparity[has_shift] * 16.0).astype(np.int16)
    cv2.filterSpeckles(sixteenths, NO_SHIFT_SIXTEENTHS, SMALLEST_SPECK_PIXELS, SPECK_NEIGHBOUR_SHIFT_PIXELS * 16)
    cleaned = disparity.copy()
    cleaned[sixteenths == NO_SHIFT_SIXTEENTHS] = np.nan
    return cleaned


def _fraction_of_a_pixel(costs, best, rows, columns):
    """Fits a parabola through the costs at the best shift and its two neighbours. Its lowest
    point lies between them, which gives the shift to a fraction of a pixel."""
    last = costs.shape[0] - 1
    before = costs[np.clip(best - 1, 0, last), rows, columns]
    at = costs[best, rows, columns]
    after = costs[np.clip(best + 1, 0, last), rows, columns]
    with np.errstate(divide="ignore", invalid="ignore"):
        # At the picture's left edge some costs are infinite; those pixels give NaN here and get 0.
        curvature = before - 2.0 * at + after
        fraction = 0.5 * (before - after) / curvature
    fraction[~np.isfinite(fraction)] = 0.0
    return np.clip(fraction, -0.5, 0.5).astype(np.float32)


# ---- From distances to points -------------------------------------------------------------------

def points_ahead_of_camera(distances, focal_length_pixels, centre_column, centre_row, every_nth_pixel=4):
    """Turns a distance picture into 3D points in the drone's own directions, measured from the
    camera: (forward, left, up) in metres. Only every_nth_pixel in each direction is used, and
    only pixels with a distance. The camera looks straight forward, along the drone's nose."""
    sampled = distances[::every_nth_pixel, ::every_nth_pixel]
    rows, columns = np.nonzero(np.isfinite(sampled))
    forward = sampled[rows, columns]
    pixel_rows = rows * every_nth_pixel
    pixel_columns = columns * every_nth_pixel
    # A camera's own directions are x right, y down, z forward. Ours are forward, left, up.
    right = (pixel_columns - centre_column) * forward / focal_length_pixels
    down = (pixel_rows - centre_row) * forward / focal_length_pixels
    return np.stack([forward, -right, -down], axis=1).astype(np.float32)


def angle_of_view_degrees(width_pixels, focal_length_pixels):
    """The camera's horizontal angle of view, edge to edge."""
    return math.degrees(2.0 * math.atan(width_pixels / 2.0 / focal_length_pixels))
