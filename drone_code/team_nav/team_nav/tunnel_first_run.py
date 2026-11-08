"""The tunnel_first mission: course_run flies high to above the tunnel's north end, turns round
slowly, comes down, and flies straight through the tunnel, when TEAM_NAV_MISSION=tunnel_first.

REAL DRONE CODE, also run unchanged in the simulators.

    TEAM_NAV_MISSION=tunnel_first ./drone.sh course_run

How it fits
-----------
tunnel_first_route.py plans and checks the route on the drone's map before anything is sent.
TunnelFirstRun is fallback_run.py's FallbackRun (itself course_run's CourseRun) with its own
flight: it keeps the fallback's height hold (height_hold.py: targets corrected so the drone holds
camera tracking's height, within strict limits), its two-way abort (fly to the pad or nearby open
floor if the position is healthy, else descend in place) and course_run's landing and safety
checks (camera tracking silent for 0.5 s, touches, targets outside the net).

The flight, stage by stage (the team's design, 2026-11-08)
---------------------------------------------------------
  climb      course_run's tested climb, straight up to 0.70 m above the launch mark.
  settle     wait until PX4 reports no ground contact and the drone stays within 0.10 m of the
             target, slower than 0.05 m/s, for 1 s. Our real climbs slid backward 3 to 34 cm
             and sideways up to 65 cm, so the route starts only once that has been flown back.
             Longer than 8 s: the run stops and lands.
  forward and high climb   0.8 m north at 0.70 m, then up to 2.4 m.
  to S       at 2.4 m, the nose unchanged, to S above open floor before the tunnel.
  turn       180 degrees to the left, slowly (heading_turn.py), holding position at S.
  descent    down to 0.90 m at S, the nose locked; then a 1 s hold, recording whether PX4 is
             using the bottom laser again.
  look       the camera finds the tunnel's centre and the drone corrects sideways
             (tunnel_first_look.py); if unsure, it flies the map's line (TEAM_NAV_TUNNEL_IF_UNSURE).
  tunnel     GO: south at 0.25 m/s along the tunnel's axis at 0.90 m, the nose locked, to past the exit.
  Then the normal landing there (TEAM_NAV_AFTER_TUNNEL=land, the default), or with
  TEAM_NAV_AFTER_TUNNEL=pad a climb to 2.4 m and the fallback's way to the pad.
Practice mode can stop after any stage with TEAM_NAV_STOP_AFTER_STAGE (for example "turn": land
at S after the turn); the drone then lands where that stage ended.

Two more checks
---------------
  - The start: refused if camera tracking puts the drone more than 0.3 m sideways or 0.1 m in
    height from the launch mark (its origin, where the drone software started). On a real run
    that started 1 m off, the drone flew anyway and hit the barrier.
  - The heading guard (heading_turn.py), outside the turn: if PX4's heading and camera tracking's
    drift more than 10 degrees apart from their launch difference for 0.5 s, hold; still apart
    3 s later, stop the run and land.
During the turn, a PX4 heading or position reset, or PX4 no longer using camera tracking's heading
for more than 1 s, stops the run.

What this file does not do: change the other missions, the competition launch file, or any PX4
setting. It never waits for depth pictures.
"""
import math
import os

import numpy as np

from team_nav import flight_steps
from team_nav.course_map import load_course_map
from team_nav.course_run import CLIMB_STAGE, LONGEST_SLIDE_STEP_SECONDS, announce_the_inputs, check_stop_after_stage, find_package_file
from team_nav.fallback_routes import shortest_high_path
from team_nav.fallback_run import FallbackRun, FallbackSafetyWatch
from team_nav.flight_steps import RunMustStop
from team_nav.heading_turn import (GUARD_HOLD, GUARD_LAND, GUARD_OK, LONGEST_TURN_SECONDS, STEP_SECONDS, HeadingGuard,
                                   SettledTimer, TurnRamp, TurnTracker, odom_heading_from_px4_heading, turn_is_settled,
                                   wrapped_degrees)
from team_nav.height_hold import LARGEST_UPWARD_CORRECTION_METRES, HeightHeldByCameraTracking
from team_nav.high_path_rules import open_floor_clearances
from team_nav.mission import load_mission
from team_nav.route_follower import RouteFollower
from team_nav.run_safety import RunSafety
from team_nav.start_frame import StartFrame
from team_nav.tunnel_first_look import LookStep, if_unsure_from_environment
from team_nav.tunnel_first_route import (CLIMB_HEIGHT_METRES, CLIMB_SPEED_METRES_PER_SECOND, CRUISE_HEIGHT_METRES,
                                         DESCENT_FAST_METRES_PER_SECOND, DESCENT_SLOW_METRES_PER_SECOND,
                                         DESCENT_SLOW_PART_METRES, DESCENT_STAGE, FORWARD_STAGE, HIGH_LEG_STAGE,
                                         HIGH_SPEED_METRES_PER_SECOND, SETTLE_STAGE, TUNNEL_HEIGHT_METRES,
                                         TUNNEL_SPEED_METRES_PER_SECOND, TUNNEL_STAGE, TURN_STAGE,
                                         TunnelFirstRouteError, plan_tunnel_first_route)

TUNNEL_FIRST = "tunnel_first"
MISSION_SETTING = "TEAM_NAV_MISSION"
AFTER_TUNNEL_SETTING = "TEAM_NAV_AFTER_TUNNEL"
AFTER_TUNNEL_LAND = "land"
AFTER_TUNNEL_PAD = "pad"
TO_THE_PAD_AFTER_TUNNEL_STAGE = "climb and to the pad"
LOOK_STAGE = "look"

# The team's choices (2026-11-08) for the start check.
LARGEST_START_OFFSET_SIDEWAYS_METRES = 0.3
LARGEST_START_OFFSET_HEIGHT_METRES = 0.1
# The team's choices for the settle stage.
SETTLE_POSITION_ERROR_METRES = 0.10
SETTLE_SPEED_METRES_PER_SECOND = 0.05
SETTLE_SECONDS = 1.0
LONGEST_SETTLE_SECONDS = 8.0
# The team's choice: PX4 not using camera tracking's heading for longer than this stops the turn.
LONGEST_HEADING_FUSION_LOSS_SECONDS = 1.0
# The team's choice: the hold at the tunnel height before GO.
SECONDS_TO_HOLD_BEFORE_GO = 1.0
# Our own choices. Up high and on climbs the target leads the drone by up to 1.0 m, as in the
# fallback routes; low down, near the tunnel, by only 0.3 m, so the drone keeps to the slow speed.
HIGH_TARGET_LEAD_METRES = 1.0
LOW_TARGET_LEAD_METRES = 0.3
# Our own choice, as for the fallback routes: a stage's end counts as reached within this distance.
FINISH_TOLERANCE_METRES = 0.10
# Our own choice: no stage of this route takes longer than this.
LONGEST_STAGE_SECONDS = 90.0


def tunnel_first_is_chosen():
    return os.environ.get(MISSION_SETTING, "").strip().lower() == TUNNEL_FIRST


def after_tunnel_from_environment():
    value = os.environ.get(AFTER_TUNNEL_SETTING, AFTER_TUNNEL_LAND).strip().lower() or AFTER_TUNNEL_LAND
    if value not in (AFTER_TUNNEL_LAND, AFTER_TUNNEL_PAD):
        raise ValueError(f"{AFTER_TUNNEL_SETTING} is '{value}'. Use '{AFTER_TUNNEL_LAND}' (land at the end of the tunnel "
                         f"line, the default) or '{AFTER_TUNNEL_PAD}' (climb and fly on to the pad).")
    return value


# =================================================================================================
# The plan
# =================================================================================================

class TunnelFirstPlan:
    """The map, the mission and the route, with the parts CourseRun needs."""

    def __init__(self, course_map, mission, other_possible_boulders, route, start_heading_degrees, after_tunnel):
        self.course_map = course_map
        self.mission = mission
        self.other_possible_boulders = other_possible_boulders
        self.route = route
        self.after_tunnel = after_tunnel
        self.start_frame = StartFrame(course_map.launch_mark_centre_metres, start_heading_degrees)
        self.pad_leg_metres = None    # (east, north) points from the end of the GO line to the pad.

    def stage_names(self):
        names = [CLIMB_STAGE, SETTLE_STAGE, FORWARD_STAGE, HIGH_LEG_STAGE, TURN_STAGE, DESCENT_STAGE, LOOK_STAGE, TUNNEL_STAGE]
        if self.after_tunnel == AFTER_TUNNEL_PAD:
            names.append(TO_THE_PAD_AFTER_TUNNEL_STAGE)
        return names


def plan_tunnel_first(arguments, run_mode, log):
    """Loads the files, plans and checks the route, records it. Raises ValueError (a full
    sentence) if it can't be flown."""
    map_path = find_package_file(arguments.map, "course_maps")
    mission_path = find_package_file(arguments.mission, "missions")
    other_map_paths = [find_package_file(name, "course_maps") for name in arguments.other_boulder_maps]
    course_map = load_course_map(map_path)
    other_possible_boulders = []
    for path in other_map_paths:
        other_possible_boulders += load_course_map(path).low_obstacles
    after_tunnel = after_tunnel_from_environment()
    if_unsure_from_environment()      # Refuses a wrong TEAM_NAV_TUNNEL_IF_UNSURE before the start.
    log.event("mission", f"MISSION tunnel_first ({MISSION_SETTING}): high to S, turn, down, straight through the tunnel; "
                         f"after the tunnel: {after_tunnel}.", mission_name=TUNNEL_FIRST, after_tunnel=after_tunnel)
    announce_the_inputs(log, map_path, mission_path, other_map_paths, arguments.start_heading_degrees)
    try:
        route = plan_tunnel_first_route(course_map)
    except TunnelFirstRouteError as problem:
        raise ValueError(str(problem)) from problem
    plan = TunnelFirstPlan(course_map, load_mission(mission_path), other_possible_boulders, route,
                           arguments.start_heading_degrees, after_tunnel)
    if after_tunnel == AFTER_TUNNEL_PAD:
        plan.pad_leg_metres = plan_the_pad_leg(course_map, route)
    log.event("route", route.describe())
    log.save_text_file("route.txt", route.describe() + "\n")
    check_stop_after_stage(run_mode, plan.stage_names())
    return plan


def plan_the_pad_leg(course_map, route):
    """With TEAM_NAV_AFTER_TUNNEL=pad: the climb at the end of the GO line, and the shortest path that
    keeps the laser rules from there to the pad. Refuses the start if it breaks a rule."""
    end = route.go_end_metres
    for name, margin_metres in open_floor_clearances(course_map, end, CRUISE_HEIGHT_METRES):
        is_low = name in [obstacle.name for obstacle in course_map.low_obstacles]
        if margin_metres < 0.0 and not is_low:
            raise ValueError(f"With {AFTER_TUNNEL_SETTING}=pad the drone climbs at the end of the GO line, but {name} is "
                             f"under the laser there. Use {AFTER_TUNNEL_SETTING}=land.")
    path, check = shortest_high_path(course_map, end, course_map.landing_pad_centre_metres, CRUISE_HEIGHT_METRES)
    if check.rule_breaks:
        raise ValueError(f"With {AFTER_TUNNEL_SETTING}=pad, the way from the tunnel to the pad breaks the laser rules: "
                         + " ".join(check.rule_breaks) + f" Use {AFTER_TUNNEL_SETTING}=land.")
    route.notes.append(f"After the tunnel: climb to {CRUISE_HEIGHT_METRES:.1f} m and {len(path)} points to the pad.")
    return [np.asarray(point, dtype=float) for point in path]


# =================================================================================================
# Readings for the turn and the heading guard
# =================================================================================================

def px4_heading_degrees(link):
    """PX4's heading written as camera tracking writes headings (see heading_turn.py), or None."""
    position = link.reports.latest.get("px4_position")
    heading = None if position is None else getattr(position, "heading", None)
    if heading is None or not math.isfinite(float(heading)):
        return None
    return math.degrees(odom_heading_from_px4_heading(float(heading)))


def camera_heading_degrees(link):
    heading = link.tracked_heading_radians()
    return None if heading is None else math.degrees(heading)


def heading_split_degrees(link):
    """PX4's heading minus camera tracking's, or None if either is missing."""
    px4, camera = px4_heading_degrees(link), camera_heading_degrees(link)
    return None if px4 is None or camera is None else wrapped_degrees(px4 - camera)


def px4_speed_metres_per_second(link):
    position = link.reports.latest.get("px4_position")
    if position is None:
        return math.inf
    return float(math.sqrt(position.vx ** 2 + position.vy ** 2 + position.vz ** 2))


def yaw_rate_degrees_per_second(link):
    """PX4's own turn rate about the vertical (vehicle_odometry), else the total rotation rate."""
    odometry = link.reports.latest.get("px4_odometry")
    if odometry is not None:
        return math.degrees(float(odometry.angular_velocity[2]))
    return link.rotation_degrees_per_second() or 0.0


def px4_reset_counters(link):
    """(heading resets, sideways position resets, attitude resets), each None if not reported."""
    position = link.reports.latest.get("px4_position")
    attitude = link.reports.latest.get("px4_attitude")
    return (None if position is None else getattr(position, "heading_reset_counter", None),
            None if position is None else getattr(position, "xy_reset_counter", None),
            None if attitude is None else getattr(attitude, "quat_reset_counter", None))


def px4_uses_camera_heading(link):
    """True if PX4 is fusing camera tracking's heading (estimator flag cs_ev_yaw), None if unknown."""
    flags = link.reports.latest.get("estimator_flags")
    return None if flags is None else bool(getattr(flags, "cs_ev_yaw", False))


def px4_reports_ground_contact(link):
    landed = link.reports.latest.get("px4_land_detected")
    if landed is None:
        return True
    return bool(landed.landed or getattr(landed, "ground_contact", False) or getattr(landed, "maybe_landed", False))


# =================================================================================================
# The flight
# =================================================================================================

class TunnelFirstRun(FallbackRun):

    # ---- Parts course_run and fallback_run build ------------------------------------------------

    def make_safety_watch(self):
        return TunnelFirstSafetyWatch(self.link, self.make_run_safety(), self.plan.start_frame)

    def make_run_safety(self):
        # Targets may reach the cruise height plus the height hold's largest upward correction.
        return RunSafety(self.plan.course_map, self.plan.other_possible_boulders,
                         highest_target_height_metres=CRUISE_HEIGHT_METRES + LARGEST_UPWARD_CORRECTION_METRES + 0.01,
                         tighten_camera_check_when_laser_stops=False)

    def highest_abort_height_metres(self):
        return CRUISE_HEIGHT_METRES

    def launch_pose_problem(self, link):
        tracked = link.tracked_position()
        return None if tracked is None else self.pose_problem(tracked)

    def pose_problem(self, tracked_position_metres):
        return start_offset_problem(tracked_position_metres)

    # ---- The stages -------------------------------------------------------------------------------

    def fly(self):
        if not self.wait_for_the_start(CLIMB_HEIGHT_METRES):
            return
        self.flight_start_seconds = self.link.now_seconds()
        self.height_hold = HeightHeldByCameraTracking(None, self.link)
        self.watch.height_hold = self.height_hold
        self.locked_heading_on_course = None
        self.tunnel_camera = None
        self.go_start_metres = np.asarray(self.plan.route.s_metres, dtype=float)
        self.watch.start_the_heading_guard(heading_split_degrees(self.link))
        route = self.plan.route
        launch, climb_point, s_point, end = route.launch_metres, route.high_climb_point_metres, route.s_metres, route.go_end_metres

        steps = [
            (CLIMB_STAGE, lambda: flight_steps.climb_above_launch(self.link, self.watch, CLIMB_HEIGHT_METRES)),
            (SETTLE_STAGE, self.settle_above_the_launch_mark),
            (FORWARD_STAGE, lambda: self.follow([_at(launch, CLIMB_HEIGHT_METRES), _at(climb_point, CLIMB_HEIGHT_METRES),
                                                 _at(climb_point, CRUISE_HEIGHT_METRES)],
                                                CLIMB_SPEED_METRES_PER_SECOND, HIGH_TARGET_LEAD_METRES)),
            (HIGH_LEG_STAGE, lambda: self.follow([_at(point, CRUISE_HEIGHT_METRES) for point in route.high_leg_metres],
                                                 HIGH_SPEED_METRES_PER_SECOND, HIGH_TARGET_LEAD_METRES)),
            (TURN_STAGE, self.turn_left_180),
            (DESCENT_STAGE, self.descend_at_s),
            (LOOK_STAGE, self.look),
            (TUNNEL_STAGE, self.go_through_the_tunnel),
        ]
        if self.plan.after_tunnel == AFTER_TUNNEL_PAD:
            steps.append((TO_THE_PAD_AFTER_TUNNEL_STAGE, self.climb_and_fly_to_the_pad))
        for name, step in steps:
            self.begin_stage(name)
            step()
            self.finish_stage()
            if name == self.stop_after_stage:
                self.land_here(f"PRACTICE RUN FINISHED: landed after the stage '{name}', as asked.")
                return
        if self.plan.after_tunnel == AFTER_TUNNEL_PAD:
            self.land_here("RUN FINISHED: landed above the pad after the tunnel.")
        else:
            self.land_here("RUN FINISHED: landed at the end of the tunnel line.")

    def look(self):
        """The LOOK step (tunnel_first_look.py): sets where GO starts."""
        from team_nav.tunnel_look_camera import TunnelLookCamera   # Needs ROS, so only loaded to fly.
        if self.tunnel_camera is None:
            self.tunnel_camera = TunnelLookCamera(self.link.node)
        self.go_start_metres = LookStep(self, self.tunnel_camera).run_look()

    def go_through_the_tunnel(self):
        """GO: from where the LOOK left the drone, along the tunnel's axis on the map, as far as the
        planned GO line goes. Unchanged from Phase 1 when GO starts at S."""
        route = self.plan.route
        start = np.asarray(self.go_start_metres, dtype=float)
        end = np.asarray(route.go_end_metres, dtype=float)
        if np.linalg.norm(start - route.s_metres) > 1e-6:
            tube = self.plan.course_map.tubes[0]
            axis = (tube.exit_centre_metres - tube.entrance_centre_metres) / tube.length_metres()
            end = start + axis * float(np.linalg.norm(route.go_end_metres - route.s_metres))
        self.log.event("go", f"GO from ({start[0]:+.3f}, {start[1]:+.3f}) to ({end[0]:+.3f}, {end[1]:+.3f}).",
                       go_start_metres=start.tolist(), go_end_metres=end.tolist())
        self.follow([_at(start, TUNNEL_HEIGHT_METRES), _at(end, TUNNEL_HEIGHT_METRES)], TUNNEL_SPEED_METRES_PER_SECOND,
                    LOW_TARGET_LEAD_METRES)

    def tunnel_height_metres(self):
        return TUNNEL_HEIGHT_METRES

    def low_target_lead_metres(self):
        return LOW_TARGET_LEAD_METRES

    def settle_above_the_launch_mark(self):
        """Holds the target above the launch mark until the drone has steadied there."""
        target = _at(self.plan.route.launch_metres, CLIMB_HEIGHT_METRES)
        timer = SettledTimer(SETTLE_SECONDS)
        deadline_seconds = self.link.now_seconds() + LONGEST_SETTLE_SECONDS

        def steady():
            drone = self.watch.drone_on_course()
            if drone is None or px4_reports_ground_contact(self.link):
                return False
            error_metres = float(np.linalg.norm(drone - target))
            return error_metres < SETTLE_POSITION_ERROR_METRES and px4_speed_metres_per_second(self.link) < SETTLE_SPEED_METRES_PER_SECOND

        while not timer.update(self.link.now_seconds(), steady()):
            if self.link.now_seconds() > deadline_seconds:
                raise RunMustStop(f"The drone did not steady within {SETTLE_POSITION_ERROR_METRES:.2f} m of the point above "
                                  f"the launch mark, slower than {SETTLE_SPEED_METRES_PER_SECOND:.2f} m/s and with no ground "
                                  f"contact, within {LONGEST_SETTLE_SECONDS:.0f} s of the climb. Landing.")
            self.send_one_cycle(target)
        drone = self.watch.drone_on_course()
        self.log.event("settle", f"Steady above the launch mark: {np.linalg.norm(drone - target):.2f} m from the target.")

    def follow(self, waypoints_metres, speed_metres_per_second, target_lead_metres):
        """Slides a target along these waypoints at one speed (see follow_with_speeds)."""
        speeds = [0.0] + [speed_metres_per_second] * (len(waypoints_metres) - 1)
        self.follow_with_speeds(waypoints_metres, speeds, target_lead_metres)

    def send_one_cycle(self, target_on_course, heading_on_course_radians=None, cycle_seconds=flight_steps.CYCLE_SECONDS):
        """One cycle of: the safety checks, the target (or the last one again while holding), the record."""
        if self.watch.check():
            self.link.targets.resend_last()
        else:
            self.watch.send_target_on_course(target_on_course, heading_on_course_radians=heading_on_course_radians)
        self.link.record_drone_condition()
        self.link.wait_one_cycle(cycle_seconds)

    def descend_at_s(self):
        s_point = self.plan.route.s_metres
        slow_from_metres = TUNNEL_HEIGHT_METRES + DESCENT_SLOW_PART_METRES
        self.follow_with_speeds([_at(s_point, CRUISE_HEIGHT_METRES), _at(s_point, slow_from_metres), _at(s_point, TUNNEL_HEIGHT_METRES)],
                                [0.0, DESCENT_FAST_METRES_PER_SECOND, DESCENT_SLOW_METRES_PER_SECOND], LOW_TARGET_LEAD_METRES)
        flight_steps.hold_still_for(self.link, self.watch, SECONDS_TO_HOLD_BEFORE_GO)
        position = self.link.reports.latest.get("px4_position")
        uses_laser = self.link.reports.px4_uses_laser_for_height()
        distance = None if position is None else round(float(position.dist_bottom), 3)
        self.log.event("laser", f"At the tunnel height before GO: PX4 {'IS' if uses_laser else 'is NOT'} using the bottom "
                                f"laser for height (flag {uses_laser}); its distance to the ground {distance} m.",
                       px4_uses_laser_for_height=uses_laser, px4_dist_bottom_metres=distance)

    def follow_with_speeds(self, waypoints_metres, speeds, target_lead_metres):
        """Slides a target along these waypoints (speeds[i] into waypoint i), the height held by camera
        tracking, the nose locked once the turn has set it."""
        follower = RouteFollower(waypoints_metres, leg_speeds_metres_per_second=speeds, target_lead_metres=target_lead_metres,
                                 finish_tolerance_metres=FINISH_TOLERANCE_METRES)
        self.height_hold.follower = follower
        last_seconds = [self.link.now_seconds()]

        def send_next_target():
            now = self.link.now_seconds()
            seconds_to_slide = min(now - last_seconds[0], LONGEST_SLIDE_STEP_SECONDS)
            last_seconds[0] = now
            target = self.height_hold.next_target(self.watch.drone_on_course(), seconds_to_slide)
            self.watch.send_target_on_course(target, heading_on_course_radians=self.locked_heading_on_course)

        flight_steps.fly_until(self.link, lambda: self.height_hold.has_finished(self.watch.drone_on_course()),
                               send_next_target, LONGEST_STAGE_SECONDS, f"the end of the stage '{self.stage_name}'", self.watch)

    def height_hold_correction_target(self, target_on_course, seconds_since_last_target=STEP_SECONDS):
        """The target with its height moved by the height hold's correction, updated for one step."""
        self.height_hold.update(seconds_since_last_target)
        corrected = list(target_on_course)
        corrected[2] += self.height_hold.correction_metres
        return corrected

    # ---- The turn ---------------------------------------------------------------------------------

    def turn_left_180(self):
        """Turns 180 degrees to the left at S, the commanded heading ramped by heading_turn.py."""
        self.watch.heading_guard_is_on = False
        turn = TurnInProgress(self)
        while not turn.is_done():
            turn.one_step()
        self.locked_heading_on_course = self.plan.start_frame.course_heading_from_start_frame(math.radians(turn.ramp.final_heading_degrees()))
        self.log.event("turn", f"Turn finished in {turn.seconds():.1f} s; the nose is locked at "
                               f"{math.degrees(self.locked_heading_on_course):.1f} degrees on the course (from east toward north).")
        self.watch.heading_guard_is_on = True

    # ---- After the tunnel -------------------------------------------------------------------------

    def climb_and_fly_to_the_pad(self):
        end = self.plan.route.go_end_metres
        self.follow([_at(end, TUNNEL_HEIGHT_METRES), _at(end, CRUISE_HEIGHT_METRES)], CLIMB_SPEED_METRES_PER_SECOND,
                    HIGH_TARGET_LEAD_METRES)
        self.follow([_at(point, CRUISE_HEIGHT_METRES) for point in self.plan.pad_leg_metres], HIGH_SPEED_METRES_PER_SECOND,
                    HIGH_TARGET_LEAD_METRES)

    def land_here(self, result_sentence):
        self.link.stage = "settling before the landing"
        flight_steps.hold_still_for(self.link, self.watch, flight_steps.SECONDS_TO_SETTLE)
        self.begin_stage("landing")
        height_source = flight_steps.choose_landing_height_source(self.link, self.watch.px4_height_is_in_doubt())
        flight_steps.land(self.link, height_source, self.watch)
        self.finish_stage()
        total_seconds = self.link.now_seconds() - self.flight_start_seconds
        self.log.event("result", f"{result_sentence} {total_seconds:.1f} s from the hand-over to motors off.",
                       total_seconds=total_seconds)


class TurnInProgress:
    """The turn's state, one 0.05 s step at a time."""

    def __init__(self, run):
        self.run, self.link, self.watch = run, run.link, run.watch
        start_heading = px4_heading_degrees(self.link)
        split = heading_split_degrees(self.link)
        if start_heading is None or split is None:
            raise RunMustStop("PX4's or camera tracking's heading is not being reported, so the turn can't be watched. Landing.")
        self.ramp = TurnRamp(start_heading)
        self.px4_turn = TurnTracker(start_heading)
        self.split_before_degrees = split
        self.counters_before = px4_reset_counters(self.link)
        self.camera_heading_was_used = px4_uses_camera_heading(self.link)
        if not self.camera_heading_was_used:
            self.link.log.event("warning", f"PX4 is not using camera tracking's heading before the turn (flag "
                                           f"{self.camera_heading_was_used}), so its loss can't be watched during the turn.")
        self.camera_heading_lost_since = None
        self.settled = SettledTimer()
        self.finished = False
        self.last_pause = None
        self.start_seconds = self.link.now_seconds()
        drone = self.watch.drone_on_course()
        s_point = run.plan.route.s_metres
        self.hold_point = np.array([s_point[0], s_point[1], CRUISE_HEIGHT_METRES])
        self.link.log.event("turn", f"Turn begins: PX4 heading {start_heading:.1f} degrees, heading split {split:+.2f} degrees, "
                                    f"reset counters {self.counters_before}, drone at {np.round(drone, 2).tolist()}.")

    def seconds(self):
        return self.link.now_seconds() - self.start_seconds

    def is_done(self):
        return self.finished

    def one_step(self):
        if self.seconds() > LONGEST_TURN_SECONDS:
            raise RunMustStop(f"The turn did not finish within {LONGEST_TURN_SECONDS:.0f} s. Landing.")
        must_hold = self.watch.check()
        self.stop_if_px4_reset_or_lost_the_camera_heading()
        heading = px4_heading_degrees(self.link)
        split = heading_split_degrees(self.link)
        if heading is None or split is None:
            raise RunMustStop("PX4's or camera tracking's heading stopped being reported during the turn. Landing.")
        turned = self.px4_turn.update(heading)
        split_change = wrapped_degrees(split - self.split_before_degrees)
        if must_hold:
            self.link.targets.resend_last()
        else:
            command = self.next_command(turned, split_change)
            target = self.run.height_hold_correction_target(self.hold_point)
            heading_on_course = self.run.plan.start_frame.course_heading_from_start_frame(math.radians(command))
            self.watch.send_target_on_course(target, heading_on_course_radians=heading_on_course)
        if self.ramp.command_has_reached_the_end():
            yaw_error = self.ramp.final_heading_degrees() - (self.ramp.start_heading_degrees + turned)
            settled = turn_is_settled(px4_speed_metres_per_second(self.link), yaw_rate_degrees_per_second(self.link),
                                      yaw_error, split_change)
            self.finished = self.settled.update(self.link.now_seconds(), settled)
        self.link.record_drone_condition()
        self.link.wait_one_cycle(STEP_SECONDS)

    def next_command(self, turned_degrees, split_change_degrees):
        command = self.ramp.next_command_degrees(turned_degrees, split_change_degrees)
        if self.ramp.paused_because != self.last_pause:
            what = f"paused: {self.ramp.paused_because}" if self.ramp.paused_because else "resumed"
            self.link.log.event("turn", f"Turn ramp {what} (commanded {self.ramp.commanded_turn_degrees:.1f} of 180 degrees, "
                                        f"PX4 has turned {turned_degrees:.1f}).")
            self.last_pause = self.ramp.paused_because
        return command

    def stop_if_px4_reset_or_lost_the_camera_heading(self):
        counters = px4_reset_counters(self.link)
        names = ["heading", "sideways position", "attitude"]
        for name, before, now in zip(names, self.counters_before, counters):
            if before is not None and now is not None and now != before:
                raise RunMustStop(f"PX4 reset its {name} estimate during the turn (its reset counter went from {before} to "
                                  f"{now}), so its heading or position just jumped. Landing.")
        if not self.camera_heading_was_used:
            return
        if px4_uses_camera_heading(self.link):
            self.camera_heading_lost_since = None
            return
        now_seconds = self.link.now_seconds()
        if self.camera_heading_lost_since is None:
            self.camera_heading_lost_since = now_seconds
        if now_seconds - self.camera_heading_lost_since > LONGEST_HEADING_FUSION_LOSS_SECONDS:
            raise RunMustStop(f"PX4 has not used camera tracking's heading for more than "
                              f"{LONGEST_HEADING_FUSION_LOSS_SECONDS:.0f} s during the turn, so its heading can drift. Landing.")


class TunnelFirstSafetyWatch(FallbackSafetyWatch):
    """fallback_run's safety watch, plus the heading guard outside the turn, and the height hold's
    correction counted as deliberate in the camera height comparison.

    Why that last part (Gazebo, 2026-11-08): in 2 of 5 runs PX4's height drifted 0.33 m above the
    truth during GO through the tunnel, while it was using the laser again (its laser use is
    "conditional", EKF2_RNG_CTRL 1, as on drone 16, so it can take a laser change for the ground
    moving). The height hold had already moved the targets by its largest upward 0.14 m to keep the
    true height, but run_safety compared PX4's raw height with camera tracking's and stopped the
    run inside the tunnel. Kenny's live navigation already has run_safety subtract the correction it
    asks for on purpose (run_safety._px4_height_less_correction); this does the same for ours. The
    height hold's own limits (height_hold.py) are unchanged, so the drone's true height can still
    be moved by camera tracking by at most 0.20 m down or 0.14 m up."""

    def __init__(self, link, run_safety, start_frame):
        super().__init__(link, run_safety, start_frame)
        self.height_hold = None     # Set by TunnelFirstRun once the flight begins.
        self.heading_guard = None
        self.heading_guard_is_on = False
        self.last_guard_verdict = GUARD_OK

    def start_the_heading_guard(self, launch_split_degrees):
        if launch_split_degrees is None:
            self.link.log.event("warning", "PX4's or camera tracking's heading isn't reported at the start, so the heading "
                                           "guard is off for this run.")
            return
        self.heading_guard = HeadingGuard(launch_split_degrees)
        self.heading_guard_is_on = True
        self.link.log.event("heading_guard", f"Heading split at the start: {launch_split_degrees:+.2f} degrees (PX4's heading "
                                             f"minus camera tracking's). The guard watches for a change of more than 10.")

    def _gather_condition(self):
        condition = super()._gather_condition()
        if self.height_hold is not None:
            condition.height_correction_metres += self.height_hold.correction_metres
        return condition

    def check(self):
        must_hold = super().check()
        if self.heading_guard is None or not self.heading_guard_is_on:
            return must_hold
        split = heading_split_degrees(self.link)
        verdict = self.heading_guard.check(self.link.now_seconds(), split)
        if verdict != self.last_guard_verdict:
            change = None if split is None else round(self.heading_guard.change_degrees(split), 1)
            self.link.log.event("heading_guard", f"Heading guard: {verdict} (the split has changed by {change} degrees).")
            self.last_guard_verdict = verdict
        if verdict == GUARD_LAND:
            raise RunMustStop("PX4's heading and camera tracking's have stayed more than 10 degrees apart (compared with the "
                              "start) for over 3.5 s, so one of them is wrong. Landing.")
        return must_hold or verdict == GUARD_HOLD


def start_offset_problem(tracked_position_metres):
    """A sentence if camera tracking's position is too far from its origin (the launch mark), else None."""
    sideways_metres = float(np.linalg.norm(tracked_position_metres[:2]))
    height_metres = abs(float(tracked_position_metres[2]))
    if sideways_metres <= LARGEST_START_OFFSET_SIDEWAYS_METRES and height_metres <= LARGEST_START_OFFSET_HEIGHT_METRES:
        return None
    return (f"Camera tracking puts the drone {sideways_metres:.2f} m sideways and {height_metres:.2f} m in height from the "
            f"launch mark, where the drone software started (the limits are {LARGEST_START_OFFSET_SIDEWAYS_METRES:.1f} m and "
            f"{LARGEST_START_OFFSET_HEIGHT_METRES:.1f} m). The route would be flown from the wrong place. Put the drone on the "
            f"launch mark and restart the drone stack (docker rm -f skybeta-stack, then start it again), then try again.")


def _at(east_north_metres, height_metres):
    return [float(east_north_metres[0]), float(east_north_metres[1]), float(height_metres)]
