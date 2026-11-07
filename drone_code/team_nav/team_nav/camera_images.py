"""Turns a ROS camera picture (sensor_msgs/Image) into a plain grey picture: a 2-D array of numbers
from 0 (black) to 255 (white), one per pixel.

How it fits: corner_count_node.py and depth_points_node.py both read the OAK camera's pictures
through this. It uses only numpy, never OpenCV or cv_bridge, so reading a picture cannot fail
because a package is missing on the drone. It does not resize or rectify anything.
"""
import numpy as np

# How many numbers each pixel has, for the picture formats ("encodings") we accept.
CHANNELS_PER_ENCODING = {"mono8": 1, "8UC1": 1, "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4,
                         "mono16": 1, "16UC1": 1}
SIXTEEN_BIT_ENCODINGS = ("mono16", "16UC1")


def grey_from_image_message(message):
    """The picture as a height x width array of uint8. Raises ValueError for a format we do not read."""
    encoding = message.encoding
    if encoding not in CHANNELS_PER_ENCODING:
        raise ValueError(f"The camera picture's format is {encoding!r}; only {', '.join(CHANNELS_PER_ENCODING)} "
                         "are read. Check which topic the image_topic launch arguments name.")
    channels = CHANNELS_PER_ENCODING[encoding]
    bytes_per_number = 2 if encoding in SIXTEEN_BIT_ENCODINGS else 1
    height, width, row_bytes = int(message.height), int(message.width), int(message.step)
    data = np.frombuffer(bytes(message.data), dtype=np.uint8)
    if row_bytes * height > len(data) or width * channels * bytes_per_number > row_bytes:
        raise ValueError(f"The camera picture says {width} x {height} pixels in {encoding}, but has only {len(data)} "
                         "bytes. It is damaged, so it was skipped.")
    rows = data[:row_bytes * height].reshape(height, row_bytes)[:, :width * channels * bytes_per_number]
    if bytes_per_number == 2:
        order = ">u2" if message.is_bigendian else "<u2"
        return (rows.copy().view(order).reshape(height, width) >> 8).astype(np.uint8)
    pixels = rows.reshape(height, width, channels)
    if channels == 1:
        return pixels[:, :, 0].copy()
    # Grey from colour with the usual weights for how bright each colour looks.
    if encoding.startswith("bgr"):
        blue, green, red = pixels[:, :, 0], pixels[:, :, 1], pixels[:, :, 2]
    else:
        red, green, blue = pixels[:, :, 0], pixels[:, :, 1], pixels[:, :, 2]
    grey = 0.299 * red + 0.587 * green + 0.114 * blue
    return np.clip(np.rint(grey), 0, 255).astype(np.uint8)


def shrink_by_whole_factor(grey, factor):
    """Every factor-th row and column, averaged over each factor x factor block, so 1024 x 640
    becomes 512 x 320 for factor 2."""
    if factor <= 1:
        return grey
    height = grey.shape[0] // factor * factor
    width = grey.shape[1] // factor * factor
    blocks = grey[:height, :width].reshape(height // factor, factor, width // factor, factor)
    return blocks.mean(axis=(1, 3)).astype(np.uint8)
