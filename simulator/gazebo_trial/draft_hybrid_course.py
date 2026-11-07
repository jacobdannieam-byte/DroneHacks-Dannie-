"""Draft map-frame Hybrid stages from a permitted teammate map; never run during flight."""

import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
# Imported packages retain their historical structure and are installed in the image.
sys.path.insert(0, str(ROOT / "drone_code/team_nav"))
from team_nav.course_map import load_course_map
from team_nav.mission import load_mission
from team_nav.route_planner import plan_route
from team_nav.start_frame import StartFrame

from make_gazebo_world import START_HEADING_DEGREES, HEIGHT_OF_CENTRE_ABOVE_FLOOR_WHEN_STANDING_METRES


KINDS = {"First corridor": "corridor", "Maze": "gate", "Tunnel": "tunnel", "Chicane": "chicane"}


def draft(map_file, mission_file, *, epoch, clock_id):
    course = load_course_map(map_file)
    mission = load_mission(mission_file)
    route = plan_route(course, mission)
    if not 2 <= len(route.waypoints_metres) <= 32:
        raise ValueError("the offline route needs manual reduction to at most 32 reviewed stages")
    start = StartFrame(course.launch_mark_centre_metres, START_HEADING_DEGREES)
    marks = {index: (name, line) for name, line, index in route.checkpoints_cleared}
    if len(marks) != len(mission.checkpoints):
        raise ValueError("route did not mark each required checkpoint")
    stages = []
    for index, waypoint in enumerate(route.waypoints_metres):
        forward, left, up = start.start_frame_from_course(waypoint)
        up -= HEIGHT_OF_CENTRE_ABOVE_FLOOR_WHEN_STANDING_METRES
        if index == 0 and abs(forward) < 1e-6 and abs(left) < 1e-6:
            continue
        checkpoint = marks.get(index)
        stage = {"stage_id": f"course_{index:02}", "kind": KINDS[checkpoint[0]] if checkpoint else "waypoint",
                 "position_m": [round(float(value), 5) for value in (forward, left, up)],
                 "arrival_m": .15, "timeout_s": 30}
        stages.append(stage)
    stages[-1].update(kind="landing", landing_site_review=None)
    return {"reference": {"frame_id": "odom", "clock_id": clock_id, "epoch": epoch},
            "route_review": None, "stages": stages,
            "draft_evidence": {"map_sha256": sha256(Path(map_file).read_bytes()).hexdigest(),
                               "mission_sha256": sha256(Path(mission_file).read_bytes()).hexdigest(),
                               "checkpoint_waypoints": route.checkpoints_cleared,
                               "notice": "Review map permissions, registered coordinates, stages, arrival margin and landing site. A local goal never counts as referee credit."}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--epoch", type=int, required=True)
    parser.add_argument("--clock-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = draft(args.map, args.mission, epoch=args.epoch, clock_id=args.clock_id)
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(report, output, indent=2, allow_nan=False)
        output.write("\n")


if __name__ == "__main__":
    main()
