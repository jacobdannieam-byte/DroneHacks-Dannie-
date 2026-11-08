"""Read saved native Gazebo bags without playing them onto ROS topics."""

from collections import Counter
import json
import math
from pathlib import Path
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


FIELDS = {
    "DistanceSensor": ("current_distance", "min_distance", "max_distance", "signal_quality", "orientation"),
    "EstimatorStatusFlags": ("cs_rng_hgt", "cs_rng_fault", "cs_rng_terrain", "cs_baro_hgt",
                             "cs_ev_pos", "cs_ev_yaw", "cs_yaw_align"),
    "VehicleLocalPosition": ("z", "dist_bottom", "dist_bottom_valid", "z_valid", "xy_valid",
                             "heading_good_for_control", "z_reset_counter", "xy_reset_counter", "heading_reset_counter"),
    "VehicleStatus": ("arming_state", "nav_state"),
    "VehicleLandDetected": ("landed",),
    "VehicleOdometry": ("pose_frame", "velocity_frame", "reset_counter"),
    "Bool": ("data",),
}


def record_ranges(samples, names):
    result = {}
    for name in names:
        values = [getattr(sample, name) for sample in samples]
        if all(isinstance(value, float) for value in values):
            finite = [value for value in values if math.isfinite(value)]
            result[name] = {"min": min(finite, default=None), "max": max(finite, default=None),
                            "nonfinite": len(values) - len(finite)}
        else:
            result[name] = dict(Counter(str(value) for value in values))
    return result


def summarize(path):
    output = {}
    with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as db:
        for topic, name, kind in db.execute("select id,name,type from topics"):
            command = "select count(*),min(timestamp),max(timestamp) from messages where topic_id=?"
            count, first, last = db.execute(command, (topic,)).fetchone()
            row = dict(type=kind, count=count, rate_sim_hz=(count - 1) * 1e9 / (last - first) if count > 1 and last > first else None)
            output[name] = row
            fields = FIELDS.get(kind.rsplit("/", 1)[-1])
            if not fields:
                continue
            message_type = get_message(kind)
            entries = [(stamp, deserialize_message(data, message_type)) for stamp, data in db.execute(
                "select timestamp,data from messages where topic_id=? order by timestamp", (topic,))]
            settled = [message for stamp, message in entries if stamp >= first + 10_000_000_000]
            if not settled:
                continue
            row["after_initial_10_sim_s"] = record_ranges(settled, fields)
            if hasattr(settled[0], "timestamp"):
                residuals = [(stamp - message.timestamp * 1000) / 1e6 for stamp, message in entries]
                row["bag_receive_minus_source_ms"] = [min(residuals), max(residuals)]
    return output


if __name__ == "__main__":
    print(json.dumps(summarize(Path(sys.argv[1])), indent=2, allow_nan=False))
