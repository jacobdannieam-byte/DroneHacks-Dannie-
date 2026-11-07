"""Turns a ROS camera picture message into a PNG file's bytes, using only Python's own zlib.

REAL DRONE CODE, used only in practice mode. The recorder (flight_recorder.py) saves about one
picture a second from each camera this way, so that after a practice flight we can see what the
cameras saw at each moment, next to the rest of the record.

Why not a picture library
-------------------------
The organizers' software on the drone may not include OpenCV or Pillow, and we must not install
anything into it. PNG is a simple format: a few labelled blocks, one of which holds the pixel
rows squeezed with zlib, which every Python has. So we write it ourselves.

The picture message
-------------------
A ROS picture (sensor_msgs/Image) has a width and height in pixels, an "encoding" saying what
each pixel's bytes mean, a "step" (the number of bytes in one row, which may include padding
at the end), and the bytes themselves. Drone 16's cameras send "bgr8": three bytes per pixel,
blue first (seen with drone_code/hardware_checks/check_drone_16.sh on 2026-11-07). PNG wants
red first, so we swap the first and third bytes of each pixel.

What it does not do
-------------------
It does not decide which pictures to save or how often (recorded_topics.py does), and it does
not write files. Encodings it does not know raise ValueError.
"""
import struct
import zlib

# Our own choice: zlib's fastest setting. The best setting makes files only a little smaller
# and takes several times longer, and the processor matters more than the disk here.
ZLIB_COMPRESSION_LEVEL = 1

# From the PNG specification: the colour type numbers.
PNG_GREY = 0
PNG_RED_GREEN_BLUE = 2
PNG_RED_GREEN_BLUE_AND_OPACITY = 6

# The encodings we can save: encoding -> (bytes per pixel, PNG colour type, bits per value,
# whether the first and third byte of each pixel must be swapped).
KNOWN_ENCODINGS = {
    "mono8": (1, PNG_GREY, 8, False),
    "8UC1": (1, PNG_GREY, 8, False),
    "mono16": (2, PNG_GREY, 16, False),
    "16UC1": (2, PNG_GREY, 16, False),
    "rgb8": (3, PNG_RED_GREEN_BLUE, 8, False),
    "bgr8": (3, PNG_RED_GREEN_BLUE, 8, True),
    "8UC3": (3, PNG_RED_GREEN_BLUE, 8, True),  # OpenCV's usual order is blue first.
    "rgba8": (4, PNG_RED_GREEN_BLUE_AND_OPACITY, 8, False),
    "bgra8": (4, PNG_RED_GREEN_BLUE_AND_OPACITY, 8, True),
}

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def picture_message_to_png(picture):
    """PNG file bytes for one sensor_msgs/Image message."""
    if picture.encoding not in KNOWN_ENCODINGS:
        raise ValueError(
            f"Cannot save a picture with the encoding '{picture.encoding}'. The known encodings "
            f"are {sorted(KNOWN_ENCODINGS)}. Add it to KNOWN_ENCODINGS in camera_pictures.py."
        )
    bytes_per_pixel, colour_type, bits_per_value, swap_first_and_third = KNOWN_ENCODINGS[picture.encoding]
    pixel_bytes = _rows_without_padding(picture, bytes_per_pixel)
    if swap_first_and_third:
        pixel_bytes = _swap_first_and_third_byte_of_each_pixel(pixel_bytes, bytes_per_pixel)
    if bits_per_value == 16 and not picture.is_bigendian:
        pixel_bytes = _swap_byte_pairs(pixel_bytes)  # PNG stores 16-bit values big end first.
    return _png_file(picture.width, picture.height, colour_type, bits_per_value, pixel_bytes,
                     picture.width * bytes_per_pixel)


def _rows_without_padding(picture, bytes_per_pixel):
    data = bytes(picture.data)
    row_bytes = picture.width * bytes_per_pixel
    if picture.step == row_bytes:
        return data[:row_bytes * picture.height]
    rows = []
    for row_number in range(picture.height):
        row_start = row_number * picture.step
        rows.append(data[row_start:row_start + row_bytes])
    return b"".join(rows)


def _swap_first_and_third_byte_of_each_pixel(pixel_bytes, bytes_per_pixel):
    # Slicing with a step copies every third (or fourth) byte at once, which is much faster
    # than a Python loop over two million bytes.
    swapped = bytearray(pixel_bytes)
    swapped[0::bytes_per_pixel] = pixel_bytes[2::bytes_per_pixel]
    swapped[2::bytes_per_pixel] = pixel_bytes[0::bytes_per_pixel]
    return bytes(swapped)


def _swap_byte_pairs(pixel_bytes):
    swapped = bytearray(pixel_bytes)
    swapped[0::2] = pixel_bytes[1::2]
    swapped[1::2] = pixel_bytes[0::2]
    return bytes(swapped)


def _png_file(width, height, colour_type, bits_per_value, pixel_bytes, row_bytes):
    # Each row starts with one "filter" byte; 0 means the row is stored as it is.
    filtered_rows = []
    for row_number in range(height):
        filtered_rows.append(b"\x00")
        filtered_rows.append(pixel_bytes[row_number * row_bytes:(row_number + 1) * row_bytes])
    squeezed = zlib.compress(b"".join(filtered_rows), ZLIB_COMPRESSION_LEVEL)
    # IHDR: width, height, bits per value, colour type, then three zeros meaning the only
    # compression, filtering and line order that PNG defines.
    header = struct.pack(">IIBBBBB", width, height, bits_per_value, colour_type, 0, 0, 0)
    return PNG_SIGNATURE + _png_block(b"IHDR", header) + _png_block(b"IDAT", squeezed) + _png_block(b"IEND", b"")


def _png_block(kind, content):
    """One PNG block: its length, its four-letter kind, its content, and a checksum."""
    checksum = zlib.crc32(kind + content) & 0xFFFFFFFF
    return struct.pack(">I", len(content)) + kind + content + struct.pack(">I", checksum)
