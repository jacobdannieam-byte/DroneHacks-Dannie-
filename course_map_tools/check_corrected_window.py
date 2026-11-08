"""Measure horizontal slices of Dannie's corrected mesh, independently of map rounding.

Laptop analysis only. Imports the mesh reader from the map builder and writes evidence;
it never changes the map, the world, flight limits, or a running drone's position.
Run from the repository with PYTHONPATH=drone_code/team_nav:course_map_tools.
"""
import json
from pathlib import Path
import numpy as np
from make_map_from_corrected_model import read_corners_and_triangles, MODEL_FILE, FLOOR_TOP_METRES
from team_nav.course_map import load_course_map

# Our choices: sample the entire proposed 1.36 m crossing envelope at 2 cm intervals.
LOWEST_CHECK_HEIGHT_METRES = 1.16
HIGHEST_CHECK_HEIGHT_METRES = 1.56
HEIGHT_STEP_METRES = .02
# Our choice: numerical tolerance for joining triangles on the same wall surface.
JOIN_TOLERANCE_METRES = .00001


def slice_intervals(triangles, wall, height):
    intervals = []
    for triangle in triangles:
        points = []
        for start, end in zip(triangle, np.roll(triangle, -1, axis=0)):
            if start[2] <= height < end[2] or end[2] <= height < start[2]:
                points.append(start + (end-start) * (height-start[2]) / (end[2]-start[2]))
        if len(points) == 2:
            along = (np.array(points)[:, :2] - wall.start_metres) @ wall.direction()
            intervals.append(sorted(along))
    merged = []
    for low, high in sorted(intervals):
        if merged and low <= merged[-1][1] + JOIN_TOLERANCE_METRES:
            merged[-1][1] = max(high, merged[-1][1])
        else:
            merged.append([low, high])
    return merged


def measure():
    root = Path(__file__).resolve().parents[1]
    course_map = load_course_map(root/'drone_code/team_nav/course_maps/real_course_2026-11-08.json')
    wall = next(wall for wall in course_map.walls if wall.openings)
    corners, faces = read_corners_and_triangles(MODEL_FILE)
    triangles = corners[faces]
    centres = triangles.mean(axis=1)
    direction = wall.direction()
    normal = np.array([-direction[1], direction[0]])
    offset = centres[:, :2] - wall.start_metres
    # Select triangles in this wall's slab and between its ends.
    nearby = ((np.abs(offset @ normal) < wall.thickness_metres / 2 + JOIN_TOLERANCE_METRES)
              & (offset @ direction >= 0) & (offset @ direction <= wall.length_metres()))
    triangles = triangles[nearby]
    samples = []
    for height in np.arange(LOWEST_CHECK_HEIGHT_METRES, HIGHEST_CHECK_HEIGHT_METRES + HEIGHT_STEP_METRES/2, HEIGHT_STEP_METRES):
        intervals = slice_intervals(triangles, wall, height + FLOOR_TOP_METRES)
        if len(intervals) != 2:
            raise ValueError(f'Expected two solid sides at height {height:.3f}, found {intervals}. Inspect the mesh.')
        samples.append({'height_metres': float(height), 'opening_start_metres': float(intervals[0][1]),
                        'opening_end_metres': float(intervals[1][0])})
    width = min(row['opening_end_metres'] for row in samples) - max(row['opening_start_metres'] for row in samples)
    return {'source_commit': '2b56750b6ee84446302a49bd0e9af43834333b8a', 'mesh_width_metres': width,
            'map_width_metres': wall.openings[0].end_along_wall_metres-wall.openings[0].start_along_wall_metres,
            'samples': samples}


if __name__ == '__main__':
    print(json.dumps(measure(), indent=2))
