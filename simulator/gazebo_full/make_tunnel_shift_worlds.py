"""Makes the Phase 3 test worlds for tunnel_first: the phone scan map with only the tunnel moved
sideways (east or west) by -0.25, 0 and +0.25 m. SIMULATOR ONLY.

The drone keeps its own unchanged map (course_run's --map phone_scan_2026-11-06.json), so the map's
tunnel and the real one disagree by the shift: what the camera's LOOK must find and correct.
Positive is east, which is the drone's left when its nose faces south after the turn.

The scan map has no cradles under its tunnel, so only the tube moves. With -0.25 m the tube's west
side reaches into the C1 chicane wall's north end; Gazebo then has two overlapping solids there,
which the drone never goes near.

Run it from the top of the repository, in the simulator box:
    docker run --rm -v "$PWD:/repository" -w /repository -e PYTHONPATH=/repository/drone_code/team_nav \\
        drone-simulator python3 simulator/gazebo_full/make_tunnel_shift_worlds.py
It writes simulator/generated_files/tunnel_shift_worlds/tunnel_{west_0.25,centre,east_0.25}.json.
"""
import copy
import os

import numpy as np

from team_nav.course_map import load_course_map, save_course_map

SCAN_MAP = "drone_code/team_nav/course_maps/phone_scan_2026-11-06.json"
OUTPUT_FOLDER = "simulator/generated_files/tunnel_shift_worlds"
# The team's choice (2026-11-08): these shifts, east positive.
SHIFTS_METRES = {"tunnel_west_0.25": -0.25, "tunnel_centre": 0.0, "tunnel_east_0.25": 0.25}


def shifted_world(scan, name, east_metres):
    world = copy.deepcopy(scan)
    tube = world.tubes[0]
    tube.entrance_centre_metres = tube.entrance_centre_metres + np.array([east_metres, 0.0])
    tube.exit_centre_metres = tube.exit_centre_metres + np.array([east_metres, 0.0])
    world.name = f"Phone scan with the tunnel moved {east_metres:+.2f} m east ({name}, SIMULATOR ONLY)"
    return world


def main():
    scan = load_course_map(SCAN_MAP)
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)
    for name, east_metres in SHIFTS_METRES.items():
        path = os.path.join(OUTPUT_FOLDER, f"{name}.json")
        save_course_map(shifted_world(scan, name, east_metres), path)
        print(f"{path}: tunnel centre line at x = {scan.tubes[0].entrance_centre_metres[0] + east_metres:.3f} m (entrance).")


if __name__ == "__main__":
    main()
