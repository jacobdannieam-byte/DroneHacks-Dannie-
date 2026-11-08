"""The fallback missions: course_run flies one of the two high routes over the walls instead of the
course route, when TEAM_NAV_MISSION says so.

REAL DRONE CODE, also run unchanged in the simulator (simulator/test_flights/real_program_test.py).

    TEAM_NAV_MISSION=fallback_pad ./drone.sh course_run        climb, fly to the pad, land
    TEAM_NAV_MISSION=fallback_tunnel ./drone.sh course_run     climb, over the maze, through the
                                                               tunnel, over the chicane, land
Without TEAM_NAV_MISSION (as in the competition launch file, launch/course_run.launch.py),
course_run flies the normal course route, exactly as before.

How it fits
-----------
fallback_routes.py plans the route from the same map file course_run uses. This file wraps it in
a plan that looks like course_run's CoursePlan, and FallbackRun is course_run's CourseRun with two
things changed: the route follower gets the fallback route's speeds (fast up high, 0.5 m/s in the
tunnel and on climbs), and the safety checks allow targets up to 2.45 m, above the wall tops, and
do not stop the run merely because PX4 reports it has stopped using the bottom laser. And the
targets' heights are corrected so the drone holds camera tracking's height, not PX4's
(HeightHeldByCameraTracking below).

Why that one check is off: in the simulator (2026-11-08), PX4 stopped using the laser the moment
the barrier's top passed under it at 2.3 m, as expected, and flew on with its air pressure height,
which stayed within about 5 cm. On the real drone that report alone would stop the run and land the
drone wherever it is, in the middle of the course. PX4's height is still watched: camera tracking's
height is compared with PX4's (30 cm limit, height_check.py), and a faulty-laser report still
stops the run.

Everything else is course_run's: the wait for the start switch, the takeoff to the mission's
cruise height (0.8 m), the other safety checks (camera tracking silent for 0.5 s, touches, targets
outside the net) and the landing.

The stages, which practice mode can stop after with TEAM_NAV_STOP_AFTER_STAGE:
    fallback_pad:     climb, high climb
    fallback_tunnel:  climb, high climb, over the maze, tunnel
"climb" is the takeoff to 0.8 m; "high climb" goes on up to 2.3 m on the launch mark. Stopping after
a stage lands where it ends, which is always over open floor.

What this file does not do: change the normal course route, the competition launch file, or any
PX4 setting. If the planned route breaks one of the team's laser rules (high_path_rules.py), it
refuses to start.
"""
import json
import os

import numpy as np

from team_nav.course_map import load_course_map
from team_nav.course_run import (CLIMB_STAGE, CourseRun, announce_the_inputs, check_stop_after_stage,
                                 find_package_file)
from team_nav.fallback_routes import (CRUISE_HEIGHT_AT_THE_START_METRES, FALLBACK_ROUTES,
                                      HIGHEST_TARGET_HEIGHT_METRES, TARGET_LEAD_METRES, plan_fallback_route)
from team_nav.mission import load_mission
from team_nav.route_follower import RouteFollower
from team_nav.run_safety import RunSafety
from team_nav.start_frame import StartFrame

# The setting that chooses a fallback route. drone.sh passes it into the container.
MISSION_SETTING = "TEAM_NAV_MISSION"

# Our own choices for HeightHeldByCameraTracking. The gap is smoothed over about a second so that
# camera tracking's small jitter does not shake the target. The correction is limited: never more
# than 0.30 m down (beyond that camera tracking's height check, height_check.py, stops the run
# anyway), and only 0.14 m up, so a corrected 2.3 m target stays below run_safety's 2.45 m limit.
HEIGHT_GAP_SMOOTHING_SECONDS = 1.0
LARGEST_DOWNWARD_CORRECTION_METRES = 0.30
LARGEST_UPWARD_CORRECTION_METRES = 0.14
# Our own choice: corrections this large are written in the records.
ANNOUNCED_CORRECTION_METRES = 0.10


def fallback_route_from_environment():
    """The fallback route named by TEAM_NAV_MISSION, or None for the normal course route."""
    value = os.environ.get(MISSION_SETTING, "").strip().lower()
    if value == "":
        return None
    if value not in FALLBACK_ROUTES:
        raise ValueError(f"{MISSION_SETTING} is '{value}'. Use one of {', '.join(FALLBACK_ROUTES)}, or leave it out "
                         f"to fly the normal course route.")
    return value


class FallbackPlan:
    """The map, the mission and the fallback route, with the same parts course_run's CoursePlan
    has, so that CourseRun can fly it."""

    def __init__(self, course_map, mission, other_possible_boulders, route, start_heading_degrees):
        self.course_map = course_map
        self.mission = mission
        self.other_possible_boulders = other_possible_boulders
        self.route = route
        self.start_frame = StartFrame(course_map.launch_mark_centre_metres, start_heading_degrees)
        self.route_stages = list(route.stages)

    def stage_names(self):
        return [CLIMB_STAGE] + [name for name, _ in self.route_stages]

    def number_of_waypoints_to_fly(self, stop_after_stage):
        for name, last_waypoint_number in self.route_stages:
            if name == stop_after_stage:
                return last_waypoint_number + 1
        return len(self.route.waypoints_metres)

    def waypoints_to_fly(self, stop_after_stage):
        return self.route.waypoints_metres[:self.number_of_waypoints_to_fly(stop_after_stage)]

    def leg_speeds_to_fly(self, stop_after_stage):
        return self.route.leg_speeds_metres_per_second[:self.number_of_waypoints_to_fly(stop_after_stage)]


class FallbackRun(CourseRun):
    """course_run's CourseRun, with the fallback route's speeds and height limit."""

    def make_run_safety(self):
        return RunSafety(self.plan.course_map, self.plan.other_possible_boulders,
                         highest_target_height_metres=HIGHEST_TARGET_HEIGHT_METRES,
                         stop_when_px4_stops_using_the_laser=False)

    def make_route_follower(self):
        follower = RouteFollower(self.plan.waypoints_to_fly(self.stop_after_stage),
                                 leg_speeds_metres_per_second=self.plan.leg_speeds_to_fly(self.stop_after_stage),
                                 target_lead_metres=TARGET_LEAD_METRES)
        return HeightHeldByCameraTracking(follower, self.link)


class HeightHeldByCameraTracking:
    """Wraps the route follower so that the drone holds the route's height as camera tracking
    measures it, not as PX4 measures it.

    Why: PX4 flies to its own height estimate. Up high, PX4 stops using the bottom laser when a wall
    top passes under it, and then uses its air pressure sensor, which drifts. In the simulator on
    2026-11-08 the drone rose from 2.2 m to 2.7 m in 12 s after the second maze wall while PX4 still
    believed 2.3 m, which is 0.35 m from the net roof. Camera tracking measured the true height.

    How: PX4 reads the target's height in its own terms, so each target's height is moved by the
    gap between PX4's height and camera tracking's (both measured from the launch mark), smoothed
    over HEIGHT_GAP_SMOOTHING_SECONDS. The follower itself still works on camera tracking's
    positions, so its corner checks are unchanged."""

    def __init__(self, follower, link):
        self.follower = follower
        self.link = link
        self.smoothed_gap_metres = 0.0
        self.announced_large_gap = False

    @property
    def next_waypoint_number(self):
        return self.follower.next_waypoint_number

    def has_finished(self, drone_position_metres):
        return self.follower.has_finished(drone_position_metres)

    def next_target(self, drone_position_metres, seconds_since_last_target):
        target = self.follower.next_target(drone_position_metres, seconds_since_last_target)
        self._update_gap(seconds_since_last_target)
        target[2] += self.smoothed_gap_metres
        return target

    def _update_gap(self, seconds_since_last_target):
        px4_height_metres = self.link.px4_height_risen_metres()
        from_launch = self.link.position_from_launch()
        if px4_height_metres is None or from_launch is None:
            return   # Keep the last correction until both heights are back.
        gap_metres = px4_height_metres - from_launch[2]
        share = min(1.0, seconds_since_last_target / HEIGHT_GAP_SMOOTHING_SECONDS)
        smoothed = self.smoothed_gap_metres + (gap_metres - self.smoothed_gap_metres) * share
        self.smoothed_gap_metres = min(max(smoothed, -LARGEST_DOWNWARD_CORRECTION_METRES), LARGEST_UPWARD_CORRECTION_METRES)
        self._announce(gap_metres)

    def _announce(self, gap_metres):
        large = abs(self.smoothed_gap_metres) >= ANNOUNCED_CORRECTION_METRES
        if large != self.announced_large_gap:
            self.announced_large_gap = large
            self.link.log.event("height_correction",
                                f"PX4's height and camera tracking's now differ by {gap_metres:+.2f} m; targets are "
                                f"moved by {self.smoothed_gap_metres:+.2f} m so the drone holds camera tracking's height.",
                                gap_metres=round(gap_metres, 3), correction_metres=round(self.smoothed_gap_metres, 3))


def plan_the_fallback(arguments, run_mode, log, route_name):
    """Loads the files, plans the fallback route, records it, and checks the stage to stop after.
    Raises ValueError (with a full sentence) if the route can't be flown."""
    map_path = find_package_file(arguments.map, "course_maps")
    mission_path = find_package_file(arguments.mission, "missions")
    other_map_paths = [find_package_file(name, "course_maps") for name in arguments.other_boulder_maps]
    course_map = load_course_map(map_path)
    mission = load_mission(mission_path)
    check_cruise_height(mission)
    other_possible_boulders = []
    for path in other_map_paths:
        other_possible_boulders += load_course_map(path).low_obstacles

    log.event("fallback", f"FALLBACK MISSION '{route_name}' ({MISSION_SETTING}): flying high over the walls, which "
                          f"breaks the rulebook's height rule on purpose.", fallback_route=route_name)
    announce_the_inputs(log, map_path, mission_path, other_map_paths, arguments.start_heading_degrees)
    route = plan_fallback_route(course_map, route_name)
    plan = FallbackPlan(course_map, mission, other_possible_boulders, route, arguments.start_heading_degrees)
    record_the_fallback_route(log, route)
    refuse_if_a_rule_is_broken(route)
    check_stop_after_stage(run_mode, plan.stage_names())
    return plan


def check_cruise_height(mission):
    if abs(mission.cruise_height_metres - CRUISE_HEIGHT_AT_THE_START_METRES) > 1e-6:
        raise ValueError(f"The mission's cruise height is {mission.cruise_height_metres:.2f} m, but the fallback routes "
                         f"start from {CRUISE_HEIGHT_AT_THE_START_METRES:.2f} m (fallback_routes.py). Use a mission "
                         f"with that cruise height, or change both together.")


def record_the_fallback_route(log, route):
    log.event("route", route.describe(), length_metres=route.length_metres(),
              waypoint_count=len(route.waypoints_metres), stages=route.stages)
    log.save_text_file("route.txt", route.describe() + "\n")
    record = {
        "fallback_route": route.name,
        "map_name": route.map_name,
        "waypoints_east_north_height_metres": np.round(route.waypoints_metres, 4).tolist(),
        "leg_speeds_metres_per_second": route.leg_speeds_metres_per_second,
        "stages": route.stages,
        "wall_crossings": [
            {"part": part, "wall": crossing.wall_name, "degrees_off_square": round(crossing.degrees_off_perpendicular, 1),
             "crossing_point_metres": np.round(crossing.crossing_point_metres, 3).tolist()}
            for part, check in route.path_checks for crossing in check.crossings
        ],
        "rule_breaks": route.rule_breaks(),
        "cautions": [text for _, check in route.path_checks for text in check.cautions],
    }
    log.save_text_file("route.json", json.dumps(record, indent=2) + "\n")


def refuse_if_a_rule_is_broken(route):
    broken = route.rule_breaks()
    if broken:
        raise ValueError(f"The fallback route '{route.name}' would break the team's laser rules on this map: "
                         + " ".join(broken) + " Check the map file, or fly the normal course route.")
