"""Reads a mission file: the order in which the drone must cross the course's checkpoint lines.

This runs on the drone and in the simulator. route_planner.py uses the mission together with
a course map: the mission says which lines to cross and in what order, and the map says where
those lines are. Keeping the two apart means the same mission works on any map, so nothing
about the route is written into the code.

A checkpoint can list more than one line, such as "through the window" or "Gate B". Any one of
them clears it, and the planner tries them in the order given.

This file does not check that the lines exist in a particular map; check_mission_fits_map does
that once both are loaded.
"""
import json
import math
from dataclasses import dataclass

MISSION_FORMAT_VERSION = 1


class MissionError(ValueError):
    """A mission file is missing something, or does not fit the map it is used with."""


@dataclass
class Checkpoint:
    name: str
    line_names_in_order_of_preference: list


@dataclass
class Mission:
    name: str
    description: str
    cruise_height_metres: float
    checkpoints: list


def load_mission(path_to_mission_file):
    """Reads a mission file and returns a Mission. Raises MissionError, with a sentence saying
    what is wrong, if anything is missing."""
    with open(path_to_mission_file, encoding="utf-8") as mission_file:
        try:
            contents = json.load(mission_file)
        except json.JSONDecodeError as error:
            raise MissionError(f"{path_to_mission_file} is not valid JSON ({error}). Check for a missing comma or bracket.") from error
    return mission_from_dictionary(contents, path_to_mission_file)


def mission_from_dictionary(contents, where_it_came_from="the mission"):
    expected_entries = {
        "mission_format_version", "name", "description", "source",
        "cruise_height_metres", "cruise_height_source", "checkpoints",
    }
    _check_entries(contents, expected_entries, set(), where_it_came_from)
    if contents["mission_format_version"] != MISSION_FORMAT_VERSION:
        raise MissionError(
            f"{where_it_came_from} uses mission format version {contents['mission_format_version']}, but this code "
            f"reads version {MISSION_FORMAT_VERSION}. Update the file or the code so they match."
        )
    cruise_height_metres = contents["cruise_height_metres"]
    if not isinstance(cruise_height_metres, (int, float)) or not math.isfinite(cruise_height_metres) or cruise_height_metres <= 0:
        raise MissionError(f"'cruise_height_metres' in {where_it_came_from} must be a number of metres above zero.")

    checkpoints = []
    for number, entry in enumerate(contents["checkpoints"]):
        place = f"checkpoint number {number + 1} in {where_it_came_from}"
        _check_entries(entry, {"name", "cross_one_of"}, {"why_this_order"}, place)
        line_names = entry["cross_one_of"]
        if not isinstance(line_names, list) or not line_names or not all(isinstance(name, str) for name in line_names):
            raise MissionError(f"'cross_one_of' in {place} must be a list of one or more checkpoint line names.")
        checkpoints.append(Checkpoint(entry["name"], list(line_names)))
    if not checkpoints:
        raise MissionError(f"{where_it_came_from} lists no checkpoints.")

    return Mission(contents["name"], contents["description"], float(cruise_height_metres), checkpoints)


def check_mission_fits_map(mission, course_map):
    """Raises MissionError if the mission names a checkpoint line the map does not have."""
    line_names_in_map = [line.name for line in course_map.checkpoint_lines]
    for checkpoint in mission.checkpoints:
        for line_name in checkpoint.line_names_in_order_of_preference:
            if line_name not in line_names_in_map:
                raise MissionError(
                    f"The mission's checkpoint '{checkpoint.name}' needs a line called '{line_name}', but the map "
                    f"'{course_map.name}' has only these lines: {', '.join(line_names_in_map)}."
                )


def _check_entries(contents, required, optional, where):
    if not isinstance(contents, dict):
        raise MissionError(f"Expected a section with named entries in {where}.")
    missing = sorted(required - set(contents))
    if missing:
        raise MissionError(f"{where} is missing: {', '.join(missing)}.")
    unexpected = sorted(set(contents) - required - optional)
    if unexpected:
        raise MissionError(f"{where} has entries this code does not know: {', '.join(unexpected)}. Check the spelling.")
