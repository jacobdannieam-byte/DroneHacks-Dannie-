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
targets' heights are corrected, within strict limits, so the drone holds camera tracking's height,
not PX4's (height_hold.py).

Two more safety rules (the team's review, 2026-11-08):
  - The route is anchored to camera tracking's pose on the launch mark (flight_steps.py takes the
    launch readings every cycle while the drone sits disarmed, and every target is sent relative to
    them). The run refuses to start if that pose is more than 0.3 m from camera tracking's origin,
    where the drone software started, because flight_interface measures its height limits from
    there.
  - The abort splits in two (fallback_abort.py): with a healthy position (position_health.py) the
    drone flies at its current height to the pad, or the nearest open floor, and lands; with the
    position lost it descends where it is, without moving sideways.

Why that one check is off: in the simulator (2026-11-08), PX4 stopped using the laser the moment
the barrier's top passed under it at 2.3 m, as expected, and flew on with its air pressure height,
which stayed within about 5 cm. On the real drone that report alone would stop the run and land the
drone wherever it is, in the middle of the course. PX4's height is still watched: camera tracking's
height is compared with PX4's (30 cm limit, height_check.py), and a faulty-laser report still
stops the run.

Everything else is course_run's: the wait for the start switch, the takeoff to the mission's
cruise height (0.8 m), the other safety checks (camera tracking silent for 0.5 s, touches, targets
outside the net) and the landing itself.

The stages, which practice mode can stop after with TEAM_NAV_STOP_AFTER_STAGE:
    fallback_pad:     climb, high climb
    fallback_tunnel:  climb, high climb, over the maze, tunnel
"climb" is the takeoff to 0.8 m; "high climb" goes on up to the cruise height (2.3 m unless
TEAM_NAV_FALLBACK_CRUISE_METRES says otherwise) on the launch mark. Stopping after a stage lands
where it ends, which is always over open floor.

What this file does not do: change the normal course route, the competition launch file, or any
PX4 setting. If the planned route breaks one of the team's laser rules (high_path_rules.py), it
refuses to start.
"""
import json
import os

import numpy as np

from team_nav import flight_steps
from team_nav.course_map import load_course_map
from team_nav.course_run import (CLIMB_STAGE, EXIT_REFUSED_TO_START, MAP_ROUTE_NAVIGATION, CourseRun, announce_the_inputs,
                                 check_stop_after_stage, find_package_file)
from team_nav.fallback_abort import ABORT_SPEED_METRES_PER_SECOND, choose_landing_spot, longest_abort_seconds
from team_nav.fallback_routes import (CRUISE_HEIGHT_AT_THE_START_METRES, FALLBACK_ROUTES, HIGH_HEIGHT_METRES,
                                      HIGHEST_TARGET_HEIGHT_METRES, TARGET_LEAD_METRES, plan_fallback_route)
from team_nav.flight_steps import ControlIsNoLongerOurs, LaunchPoseRefused, RunMustStop, SafetyWatch
from team_nav.height_hold import HeightHeldByCameraTracking
from team_nav.mission import load_mission
from team_nav.position_health import PositionHealth
from team_nav.route_follower import RouteFollower
from team_nav.run_safety import AIRBORNE_HEIGHT_METRES, RunSafety
from team_nav.start_frame import StartFrame

# The setting that chooses a fallback route. drone.sh passes it into the container.
MISSION_SETTING = "TEAM_NAV_MISSION"

# Our own choice, from the team's review (2026-11-08). flight_interface measures its height limits
# (0.3 m floor, 3.00 m ceiling) from where camera tracking started, the software start origin, while
# our route is anchored to the drone's pose on the launch mark. If the two are more than this apart,
# the drone was moved after the software started, and the limits no longer mean what we think.
LARGEST_LAUNCH_POSE_FROM_ORIGIN_METRES = 0.3

# The team's choice (2026-11-08), after simulator runs of 42783f6: the landing starts above the pad
# once the drone is within this distance sideways, not route_follower.py's 5 cm. With 5 cm the drone
# hovered 6-7 cm off for up to 11 s while PX4's air pressure height drifted, until camera tracking's
# height check stopped the run. Also used for the abort's flight to its landing spot.
LANDING_START_SIDEWAYS_TOLERANCE_METRES = 0.10


def fallback_route_from_environment():
    """The fallback route named by TEAM_NAV_MISSION, or None for the normal course route."""
    value = os.environ.get(MISSION_SETTING, "").strip().lower()
    if value == "":
        return None
    if value not in FALLBACK_ROUTES:
        raise ValueError(f"{MISSION_SETTING} is '{value}'. Use one of {', '.join(FALLBACK_ROUTES)} or tunnel_first, or leave it out "
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
    """course_run's CourseRun, with the fallback route's speeds, height limit and height hold, a
    check of the launch pose before the start, and the two-way abort."""

    def __init__(self, plan, run_mode, link, navigation=None):
        # The fallback routes always fly their planned route with the route follower, whatever
        # course_run's --navigation says: live navigation (live_flight.py) is for the course route.
        super().__init__(plan, run_mode, link, MAP_ROUTE_NAVIGATION)

    def run(self):
        self.refusal = None
        exit_code = super().run()
        return EXIT_REFUSED_TO_START if self.refusal else exit_code

    # ---- The start ------------------------------------------------------------------------------

    def wait_for_the_start(self, cruise_height_metres):
        """Like course_run's, but refuses to start if camera tracking's pose on the launch mark is
        more than LARGEST_LAUNCH_POSE_FROM_ORIGIN_METRES from its origin. Checked before anything is
        sent, every cycle while waiting for the switch, and once more on the remembered launch pose
        when control arrives (in case the switch came within the same cycle)."""
        flight_steps.wait_until_ready_on_the_ground(self.link)
        problem = self.launch_pose_problem(self.link)
        if problem is None:
            try:
                flight_steps.hold_above_launch_until_control_is_ours(self.link, cruise_height_metres,
                                                                     launch_pose_problem=self.launch_pose_problem)
            except LaunchPoseRefused as refusal:
                problem = str(refusal)
            else:
                problem = self.pose_problem(self.link.launch.tracked_position_metres)
                if problem is not None:
                    # Control is already ours: put the drone straight back down, with course_run's
                    # landing in place, never the abort's flight to the pad.
                    self.refuse(problem)
                    CourseRun.land_after_a_problem(self, land_using_laser=False)
                    return False
        if problem is not None:
            self.refuse(problem)
            return False
        return True

    def launch_pose_problem(self, link):
        """tunnel_first_run.py replaces this and pose_problem with its own limits."""
        return launch_pose_problem(link)

    def pose_problem(self, tracked_position_metres):
        return pose_too_far_from_origin(tracked_position_metres)

    def refuse(self, problem):
        self.refusal = problem
        self.log.event("refused", f"REFUSING TO START THE FLIGHT. {problem}")

    # ---- Parts course_run builds ----------------------------------------------------------------

    def make_safety_watch(self):
        return FallbackSafetyWatch(self.link, self.make_run_safety(), self.plan.start_frame)

    def make_run_safety(self):
        return RunSafety(self.plan.course_map, self.plan.other_possible_boulders,
                         highest_target_height_metres=HIGHEST_TARGET_HEIGHT_METRES,
                         tighten_camera_check_when_laser_stops=False)

    def make_route_follower(self):
        follower = RouteFollower(self.plan.waypoints_to_fly(self.stop_after_stage),
                                 leg_speeds_metres_per_second=self.plan.leg_speeds_to_fly(self.stop_after_stage),
                                 target_lead_metres=TARGET_LEAD_METRES,
                                 finish_tolerance_metres=LANDING_START_SIDEWAYS_TOLERANCE_METRES)
        # Kept, so that an abort carries on with the same height correction (fly_straight_to).
        self.height_hold = HeightHeldByCameraTracking(follower, self.link)
        return self.height_hold

    # ---- The abort (fallback_abort.py) ----------------------------------------------------------

    def land_after_a_problem(self, land_using_laser):
        """Position healthy: fly at the current height to the pad, or the nearest open floor, then
        land. Position lost: land where the drone is, without moving sideways (course_run's landing)."""
        if self.link.control_is_ours() and self.link.is_armed():
            try:
                self.fly_to_a_landing_spot_if_position_is_healthy()
            except ControlIsNoLongerOurs as reason:
                self.log.event("control_lost", f"Control was taken away during the abort's flight. {reason}")
                return
            except KeyboardInterrupt:
                self.log.event("interrupted", "Interrupted again: the abort is abandoned. The drone holds its last "
                                              "position target until a teammate takes over or stops the motors.")
                return
            except Exception as error:
                self.log.event("error", f"ERROR during the abort's flight ({error}); landing where the drone is.")
        super().land_after_a_problem(land_using_laser)

    def fly_to_a_landing_spot_if_position_is_healthy(self):
        watch = self.watch
        problem = watch.position_health.problem()
        drone = watch.drone_on_course()
        if problem is not None or drone is None:
            self.log.event("abort", f"POSITION LOST: descending where the drone is, without moving sideways. "
                                    f"{problem or 'No position from camera tracking.'}")
            return
        if drone[2] < AIRBORNE_HEIGHT_METRES:
            self.log.event("abort", "The drone is barely off the ground: landing where it is.")
            return
        # Never above the route's height: a run often stops because PX4's height drifted, which can
        # leave the drone above 2.3 m, and run_safety refuses targets above 2.45 m (simulator, 2026-11-08).
        abort_height_metres = min(float(drone[2]), self.highest_abort_height_metres())
        spot, why = choose_landing_spot(self.plan.course_map, [drone[0], drone[1], abort_height_metres])
        if spot is None:
            self.log.event("abort", f"Position is healthy, but there is {why}: landing where the drone is.")
            return
        self.log.event("abort", f"POSITION HEALTHY: flying at {abort_height_metres:.2f} m to {why}, "
                                f"({spot[0]:+.2f}, {spot[1]:+.2f}), then landing.", landing_spot_metres=spot.tolist())
        self.link.stage = "abort: flying to a landing spot"
        self.fly_straight_to(spot, drone, abort_height_metres)

    def highest_abort_height_metres(self):
        """The route's cruise height; tunnel_first_run.py replaces it with its own."""
        return HIGH_HEIGHT_METRES

    def fly_straight_to(self, spot, drone, height_metres):
        """Slides a target from above the drone to above the spot, both at height_metres. If the drone
        is higher, the first target simply takes it down to that height where it is."""
        follower = RouteFollower([[drone[0], drone[1], height_metres], [spot[0], spot[1], height_metres]],
                                 [0.0, ABORT_SPEED_METRES_PER_SECOND],
                                 finish_tolerance_metres=LANDING_START_SIDEWAYS_TOLERANCE_METRES)
        # The route's height hold carries on with its present correction. A fresh one would start
        # from zero and, in the simulator (2026-11-08), let the drone climb to 2.70 m by the whole of
        # PX4's height error before catching up at 0.1 m/s.
        held = getattr(self, "height_hold", None)
        if held is None:
            held = HeightHeldByCameraTracking(follower, self.link)
        held.follower = follower
        deadline_seconds = self.link.now_seconds() + longest_abort_seconds(drone, spot)
        while not held.has_finished(drone):
            if not (self.link.control_is_ours() and self.link.is_armed()):
                raise ControlIsNoLongerOurs("The drone is disarmed or no longer in Offboard mode.")
            self.watch.observe_position()
            problem = self.watch.position_health.problem()
            if problem is not None or self.link.now_seconds() > deadline_seconds:
                self.log.event("abort", f"Stopping the abort's flight here and landing: "
                                        f"{problem or 'it took too long to reach the landing spot.'}")
                return
            try:
                self.watch.send_target_on_course(held.next_target(drone, flight_steps.CYCLE_SECONDS))
            except RunMustStop as refusal:
                self.log.event("abort", f"Stopping the abort's flight here and landing: {refusal}")
                return
            self.link.record_drone_condition()
            self.link.wait_one_cycle()
            drone = self.watch.drone_on_course()
            if drone is None:
                return


class FallbackSafetyWatch(SafetyWatch):
    """course_run's SafetyWatch, which also keeps position_health.py up to date every cycle."""

    def __init__(self, link, run_safety, start_frame):
        super().__init__(link, run_safety, start_frame)
        self.position_health = PositionHealth()
        self.announced_problem = None

    def check(self):
        self.observe_position()
        return super().check()

    def observe_position(self):
        link = self.link
        px4_position = link.reports.latest.get("px4_position")
        reset_counts = None
        if px4_position is not None:
            reset_counts = (getattr(px4_position, "xy_reset_counter", None), getattr(px4_position, "z_reset_counter", None))
        self.position_health.observe(link.tracked_position_arrival_seconds, link.tracked_position(),
                                     link.camera_tracking_age_seconds(), link.camera_tracking_says_lost(), reset_counts)
        lasting = self.position_health.lasting_problem
        if lasting is not None and lasting != self.announced_problem:
            self.announced_problem = lasting
            link.log.event("position", f"POSITION NO LONGER TRUSTED for the rest of the run: {lasting}")


def launch_pose_problem(link):
    """A sentence if camera tracking's present pose is too far from its origin, else None."""
    tracked = link.tracked_position()
    return None if tracked is None else pose_too_far_from_origin(tracked)


def pose_too_far_from_origin(tracked_position_metres):
    distance_metres = float(np.linalg.norm(tracked_position_metres))
    if distance_metres <= LARGEST_LAUNCH_POSE_FROM_ORIGIN_METRES:
        return None
    return (f"Camera tracking puts the drone {distance_metres:.2f} m from where the drone software started (its "
            f"origin), more than the {LARGEST_LAUNCH_POSE_FROM_ORIGIN_METRES:.1f} m allowed. The drone was probably moved "
            f"after the software started, and flight_interface measures its height limits from that origin. Put the "
            f"drone on the launch mark and restart the drone software (docker rm -f skybeta-stack), then try again.")


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
