"""Assemble launch bindings from reviewed H1 and mission files. Supplies no guessed calibration."""

import argparse
import json
from pathlib import Path

from examples.gazebo_sensor_contract import SIM_BINDINGS, SIM_ESTIMATOR
from examples.h1_observation import PX4_TYPES, TF, TRACKING, HybridBagProfile
from examples.hybrid_launch import configured
from examples.spatial_bag_input import TOPICS


# Pinned Humble image schema. The container checks these against installed classes.
OWNER = {name: f"/fmu/out/{name}{suffix}" for name, suffix in (
    ("vehicle_attitude", ""), ("vehicle_status", "_v1"), ("vehicle_land_detected", ""))}


def assemble(h1, course, output, *, mode, profile_id):
    out_dir = Path(output).resolve().parent
    h1, course = Path(h1).resolve(), Path(course).resolve()
    if not all(path.parent == out_dir for path in (h1, course)):
        raise ValueError("keep both input files next to the launcher profile")
    profile = HybridBagProfile.from_dict(json.loads(h1.read_text()))
    if profile.tracking_source != SIM_ESTIMATOR:
        raise ValueError("H1 must explicitly select simulated_estimator")
    topics = {**TOPICS, TF: "tf2_msgs/msg/TFMessage", TRACKING: "diagnostic_msgs/msg/DiagnosticArray",
              **{name: PX4_TYPES[role] for role, name in profile.px4_topics.items()}}
    return {"strategy": mode, "profile_id": profile_id, "review_status": "reviewed",
            "h1_profile": h1.name, "mission_profile": course.name, "owner_topics": OWNER,
            "sources": {name: {"topic": SIM_BINDINGS.get(name, name), "type": kind,
                               "provenance": "Gazebo/Humble isolated test graph; see H1 evidence"}
                        for name, kind in topics.items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--h1", type=Path, required=True)
    parser.add_argument("--course", type=Path, required=True)
    parser.add_argument("--mode", choices=("hybrid_shadow", "hybrid_sitl"), default="hybrid_shadow")
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = assemble(args.h1, args.course, args.output, mode=args.mode, profile_id=args.profile_id)
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(result, output, indent=2, allow_nan=False)
        output.write("\n")
    try:
        configured(args.output)
    except (ValueError, TypeError, KeyError, OSError) as error:
        args.output.unlink()
        parser.error(str(error))


if __name__ == "__main__":
    main()
