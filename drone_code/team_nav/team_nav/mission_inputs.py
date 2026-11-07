"""What the detectors have told the mission, and whether each report can still be trusted.

How it fits: other programs (a line detector, a pad detector, the depth program, a camera tracking
monitor) will write their latest results into a MissionInputs record; the bottom laser height is
filled in by mission.py. course_stages.py then calls read_inputs() to get a judged view of them.
None of these detectors exists yet, so every input starts out missing, which counts as invalid.

Each report carries the time its data was captured, in seconds on the mission's clock. A report
that is missing, older than its limit, or stamped in the future is invalid, exactly as if the
detector had said "not found".

Directions follow the ROS convention: forward and left of the drone's nose are positive, and
angles are positive anticlockwise seen from above (turning left).
"""
import math
from dataclasses import dataclass


@dataclass
class Clearances:
    """How far the depth camera saw free space from one picture, in four directions.

    Each value is None when the picture does not show that direction well enough to say (unknown,
    never free), infinity when it was seen to be clear out to judged_out_to_metres, or the distance
    in metres to the nearest obstacle. left and right look about 20 degrees either side of ahead;
    low looks ahead below flight height, for boulders."""
    ahead_metres: float | None = None
    left_metres: float | None = None
    right_metres: float | None = None
    low_metres: float | None = None
    judged_out_to_metres: float = 1.9


@dataclass
class MissionInputs:
    line_stamp_seconds: float | None = None
    line_valid: bool = False
    line_offset_left_metres: float = math.nan     # how far the line is to the left of the drone
    line_angle_left_radians: float = math.nan     # how far the line turns left of the nose
    line_end_seen: bool = False
    pad_stamp_seconds: float | None = None
    pad_valid: bool = False
    pad_offset_forward_left_metres: tuple | None = None   # (forward, left) from the drone to the pad centre
    clearances_stamp_seconds: float | None = None
    clearances: Clearances | None = None
    vision_stamp_seconds: float | None = None
    vision_ok: bool = False                       # camera tracking is following real visual detail
    height_stamp_seconds: float | None = None
    height_above_floor_metres: float | None = None    # from the bottom laser sensor

    def set_line(self, stamp_seconds, valid, offset_left_metres=math.nan, angle_left_radians=math.nan,
                 end_seen=False):
        self.line_stamp_seconds = stamp_seconds
        self.line_valid = bool(valid)
        self.line_offset_left_metres = float(offset_left_metres)
        self.line_angle_left_radians = float(angle_left_radians)
        self.line_end_seen = bool(end_seen)

    def set_pad(self, stamp_seconds, valid, offset_forward_left_metres=None):
        self.pad_stamp_seconds = stamp_seconds
        self.pad_valid = bool(valid)
        self.pad_offset_forward_left_metres = offset_forward_left_metres

    def set_clearances(self, stamp_seconds, clearances):
        self.clearances_stamp_seconds = stamp_seconds
        self.clearances = clearances

    def set_vision(self, stamp_seconds, ok):
        self.vision_stamp_seconds = stamp_seconds
        self.vision_ok = ok is True

    def set_height(self, stamp_seconds, height_above_floor_metres):
        self.height_stamp_seconds = stamp_seconds
        self.height_above_floor_metres = height_above_floor_metres


@dataclass
class InputView:
    """The inputs as judged at one moment. Each condition is "ok", "missing", "stale",
    "from the future" or "lost" (reported, but saying "not found" or holding bad numbers)."""
    inputs: MissionInputs
    line_ok: bool
    line_condition: str
    pad_ok: bool
    clearances: Clearances | None                 # None unless the clearances are ok
    clearances_condition: str
    vision_ok: bool
    vision_condition: str
    height_above_floor_metres: float | None       # None unless the height is ok
    height_condition: str


def is_finite_number(value):
    return value is not None and math.isfinite(value)


def report_condition(stamp_seconds, now_seconds, max_age_seconds, looks_valid, future_tolerance_seconds):
    if stamp_seconds is None or not math.isfinite(stamp_seconds):
        return "missing"
    if stamp_seconds > now_seconds + future_tolerance_seconds:
        return "from the future"
    if now_seconds - stamp_seconds > max_age_seconds:
        return "stale"
    if not looks_valid:
        return "lost"
    return "ok"


def read_inputs(inputs, now_seconds, settings):
    tolerance = settings.future_stamp_tolerance_seconds
    line_looks_valid = (inputs.line_valid and is_finite_number(inputs.line_offset_left_metres)
                        and is_finite_number(inputs.line_angle_left_radians))
    line = report_condition(inputs.line_stamp_seconds, now_seconds, settings.line_max_age_seconds,
                            line_looks_valid, tolerance)
    pad_offset = inputs.pad_offset_forward_left_metres
    pad_looks_valid = (inputs.pad_valid and pad_offset is not None and len(pad_offset) == 2
                       and is_finite_number(pad_offset[0]) and is_finite_number(pad_offset[1]))
    pad = report_condition(inputs.pad_stamp_seconds, now_seconds, settings.pad_max_age_seconds,
                           pad_looks_valid, tolerance)
    clearances = report_condition(inputs.clearances_stamp_seconds, now_seconds,
                                  settings.clearances_max_age_seconds, inputs.clearances is not None, tolerance)
    vision = report_condition(inputs.vision_stamp_seconds, now_seconds, settings.vision_max_age_seconds,
                              inputs.vision_ok is True, tolerance)
    height_looks_valid = (is_finite_number(inputs.height_above_floor_metres)
                          and inputs.height_above_floor_metres >= 0.0)
    height = report_condition(inputs.height_stamp_seconds, now_seconds, settings.laser_report_max_age_seconds,
                              height_looks_valid, tolerance)
    return InputView(
        inputs=inputs,
        line_ok=line == "ok", line_condition=line,
        pad_ok=pad == "ok",
        clearances=inputs.clearances if clearances == "ok" else None, clearances_condition=clearances,
        vision_ok=vision == "ok", vision_condition=vision,
        height_above_floor_metres=float(inputs.height_above_floor_metres) if height == "ok" else None,
        height_condition=height,
    )
