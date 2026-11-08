"""The tunnel_first mission: course_run flies high to above the tunnel's north end, comes down, checks
its height, turns round slowly, looks at the tunnel and flies straight through it, when
TEAM_NAV_MISSION=tunnel_first.

REAL DRONE CODE, also run unchanged in the simulators.

    TEAM_NAV_MISSION=tunnel_first ./drone.sh course_run

How it fits
-----------
tunnel_first_route.py plans and checks the route on the drone's map before anything is sent.
TunnelFirstRun is fallback_run.py's FallbackRun (itself course_run's CourseRun) with its own flight:
it keeps the fallback's height hold (height_hold.py), its two-way abort, and course_run's landing and
safety checks. tunnel_first_watch.py adds the heading guard and the height watch; tunnel_first_turn.py
is the turn; tunnel_first_look.py the LOOK; tunnel_height.py the height gate and the rules for coming
down.

The flight, stage by stage (the team's spec, 2026-11-08)
-------------------------------------------------------
  (start)    refused if camera tracking is more than 0.3 m sideways or 0.1 m in height from the
             launch mark (a real run that started 1 m off flew anyway and hit the barrier).
  climb      course_run's tested climb to 0.70 m above the launch mark.
  settle     until PX4 reports no ground contact and the drone stays within 0.10 m of the target,
             slower than 0.05 m/s, for 1 s (our real climbs slid 3 to 65 cm); longer than 8 s: land.
             Meanwhile the height baseline b0 (PX4's height minus camera tracking's) is averaged.
  forward and high climb   0.8 m north at 0.70 m, then up to the cruise height (default 2.4 m).
  corridor end             only with TEAM_NAV_CROSS_CORRIDOR_END=on: north at cruise to north 2.34.
  to S       at cruise, 0.6 m/s with a smooth start, the nose unchanged, to S.
  descent    at S: the target slides at 0.7 m/s to 1.2 m, then 0.15 m/s to 0.90 m.
  gate       2 s of PX4 on the laser, its laser check passing, no height reset and the height
             disagreement within 0.10 m, within 4 s (TEAM_NAV_TUNNEL_GATE=relaxed: within 0.15 m
             alone). If not, the drone lands straight down at S.
  turn       180 degrees to the left at 0.90 m, slowly (heading_turn.py).
  look       backs north 0.26 m (nose south) at 0.10 m/s, settles 1 s, then the LOOK; if unsure it
             flies the map's line (TEAM_NAV_TUNNEL_IF_UNSURE=map) or lands (=land).
  tunnel     GO: south at 0.25 m/s along the tunnel's axis at 0.90 m, the nose locked.
  Then the normal landing about 0.5 m past the exit (TEAM_NAV_AFTER_TUNNEL=land, the default), or
  (=pad) a slide to the spot with most room, 1 s hold, a straight climb to cruise, and 0.6 m/s over
  the chicane to the pad, where it lands. The nose never turns after the tunnel.
Practice mode can stop after any stage with TEAM_NAV_STOP_AFTER_STAGE (gate, turn, look, ...): the
drone lands where that stage ended.

When the run must stop
----------------------
  - The height gate failed: straight down at S.
  - Above 1.5 m (camera tracking's height), for any reason: never down over a wall, never up. Over
    open floor straight down at 0.15 m/s; over a wall, at the same height along the planned path to
    the first open floor within 1.5 m (or the nearest within 1.5 m), then down (tunnel_height.py).
  - Lower: fallback_run's two-way abort (to open floor at the same height if the position is
    healthy, else straight down).
After every run a one-paragraph height report is written (and printed).

What this file does not do: change the other missions, the competition launch file, or any PX4
setting. It never waits for depth pictures.
"""
import math
import os

import numpy as np

from team_nav import flight_steps
from team_nav.course_map import load_course_map
from team_nav.course_run import CLIMB_STAGE, LONGEST_SLIDE_STEP_SECONDS, CourseRun, announce_the_inputs, check_stop_after_stage, find_package_file
from team_nav.fallback_run import FallbackRun
from team_nav.flight_steps import ControlIsNoLongerOurs, RunMustStop
from team_nav.heading_turn import STEP_SECONDS, SettledTimer
from team_nav.height_hold import LARGEST_UPWARD_CORRECTION_METRES, HeightHeldByCameraTracking
from team_nav.mission import load_mission
from team_nav.route_follower import RouteFollower
from team_nav.run_safety import RunSafety
from team_nav.start_frame import StartFrame
from team_nav.tunnel_first_look import LookStep, if_unsure_from_environment
from team_nav.tunnel_first_readings import (camera_height_metres, heading_split_degrees, px4_estimator_flag, px4_height_metres,
                                            px4_position_field, px4_reports_ground_contact, px4_speed_metres_per_second)
from team_nav.tunnel_first_route import (CLIMB_HEIGHT_METRES, CLIMB_SPEED_METRES_PER_SECOND, CORRIDOR_STAGE,
                                         DESCENT_FAST_METRES_PER_SECOND, DESCENT_SLOW_FROM_METRES,
                                         DESCENT_SLOW_METRES_PER_SECOND, DESCENT_STAGE, FORWARD_STAGE, GATE_STAGE,
                                         HIGH_LEG_STAGE, HIGH_SPEED_METRES_PER_SECOND, LOOK_BACKUP_SETTLE_SECONDS,
                                         LOOK_BACKUP_SPEED_METRES_PER_SECOND, LOOK_STAGE, PAD_STAGE, SETTLE_STAGE,
                                         TUNNEL_HEIGHT_METRES, TUNNEL_SPEED_METRES_PER_SECOND, TUNNEL_STAGE, TURN_STAGE,
                                         TunnelFirstRouteError, plan_tunnel_first_route)
from team_nav.tunnel_first_turn import TurnInProgress
from team_nav.tunnel_first_watch import TunnelFirstSafetyWatch
from team_nav.tunnel_height import (DESCENT_SEARCH_METRES, GATE_RELAXED, GATE_SETTING, GATE_STRICT, WATCH_ABOVE_METRES,
                                    HeightGate, choose_descent_point)

TUNNEL_FIRST = "tunnel_first"
MISSION_SETTING = "TEAM_NAV_MISSION"
AFTER_TUNNEL_SETTING = "TEAM_NAV_AFTER_TUNNEL"
AFTER_TUNNEL_LAND = "land"
AFTER_TUNNEL_PAD = "pad"

# The team's choices (2026-11-08).
LARGEST_START_OFFSET_SIDEWAYS_METRES = 0.3
LARGEST_START_OFFSET_HEIGHT_METRES = 0.1
SETTLE_POSITION_ERROR_METRES = 0.10
SETTLE_SPEED_METRES_PER_SECOND = 0.05
SETTLE_SECONDS = 1.0
LONGEST_SETTLE_SECONDS = 8.0
HIGH_DESCENT_METRES_PER_SECOND = 0.15
PAD_SPOT_HOLD_SECONDS = 1.0
# Our own choices. Up high and on climbs the target leads the drone by up to 1.0 m, as in the
# fallback routes; low down by only 0.3 m, so the drone keeps to the slow speeds. The high legs
# start smoothly: the target's speed rises to its full value over HIGH_LEG_SPEED_UP_SECONDS.
HIGH_TARGET_LEAD_METRES = 1.0
LOW_TARGET_LEAD_METRES = 0.3
HIGH_LEG_SPEED_UP_SECONDS = 2.0
FINISH_TOLERANCE_METRES = 0.10
LONGEST_STAGE_SECONDS = 90.0
# Our own choices for coming down from high: slide sideways at this speed, and hand over to the
# normal landing at this height.
HIGH_SIDEWAYS_METRES_PER_SECOND = 0.3
HAND_OVER_TO_LANDING_METRES = 0.5


def tunnel_first_is_chosen():
    return os.environ.get(MISSION_SETTING, "").strip().lower() == TUNNEL_FIRST


def after_tunnel_from_environment():
    value = os.environ.get(AFTER_TUNNEL_SETTING, AFTER_TUNNEL_LAND).strip().lower() or AFTER_TUNNEL_LAND
    if value not in (AFTER_TUNNEL_LAND, AFTER_TUNNEL_PAD):
        raise ValueError(f"{AFTER_TUNNEL_SETTING} is '{value}'. Use '{AFTER_TUNNEL_LAND}' (land past the tunnel, the default) "
                         f"or '{AFTER_TUNNEL_PAD}' (climb and fly on to the pad).")
    return value


def gate_mode_from_environment():
    value = os.environ.get(GATE_SETTING, GATE_STRICT).strip().lower() or GATE_STRICT
    if value not in (GATE_STRICT, GATE_RELAXED):
        raise ValueError(f"{GATE_SETTING} is '{value}'. Use '{GATE_STRICT}' (the default) or '{GATE_RELAXED}'.")
    return value


# =================================================================================================
# The plan
# =================================================================================================

class TunnelFirstPlan:
    """The map, the mission and the route, with the parts CourseRun needs."""

    def __init__(self, course_map, mission, other_possible_boulders, route, start_heading_degrees, after_tunnel, gate_mode):
        self.course_map = course_map
        self.mission = mission
        self.other_possible_boulders = other_possible_boulders
        self.route = route
        self.after_tunnel = after_tunnel
        self.gate_mode = gate_mode
        self.start_frame = StartFrame(course_map.launch_mark_centre_metres, start_heading_degrees)

    def stage_names(self):
        names = [CLIMB_STAGE, SETTLE_STAGE, FORWARD_STAGE]
        if self.route.corridor_end_metres is not None:
            names.append(CORRIDOR_STAGE)
        names += [HIGH_LEG_STAGE, DESCENT_STAGE, GATE_STAGE, TURN_STAGE, LOOK_STAGE, TUNNEL_STAGE]
        if self.after_tunnel == AFTER_TUNNEL_PAD:
            names.append(PAD_STAGE)
        return names


def plan_tunnel_first(arguments, run_mode, log):
    """Loads the files, plans and checks the route, records it. Raises ValueError (a full sentence)
    if it can't be flown."""
    map_path = find_package_file(arguments.map, "course_maps")
    mission_path = find_package_file(arguments.mission, "missions")
    other_map_paths = [find_package_file(name, "course_maps") for name in arguments.other_boulder_maps]
    course_map = load_course_map(map_path)
    other_possible_boulders = []
    for path in other_map_paths:
        other_possible_boulders += load_course_map(path).low_obstacles
    after_tunnel, gate_mode = after_tunnel_from_environment(), gate_mode_from_environment()
    if_unsure_from_environment()      # Refuses a wrong TEAM_NAV_TUNNEL_IF_UNSURE before the start.
    log.event("mission", f"MISSION tunnel_first ({MISSION_SETTING}): after the tunnel {after_tunnel}, height gate {gate_mode}.",
              mission_name=TUNNEL_FIRST, after_tunnel=after_tunnel, gate_mode=gate_mode)
    announce_the_inputs(log, map_path, mission_path, other_map_paths, arguments.start_heading_degrees)
    try:
        route = plan_tunnel_first_route(course_map, after_tunnel_to_pad=after_tunnel == AFTER_TUNNEL_PAD)
    except TunnelFirstRouteError as problem:
        raise ValueError(str(problem)) from problem
    plan = TunnelFirstPlan(course_map, load_mission(mission_path), other_possible_boulders, route,
                           arguments.start_heading_degrees, after_tunnel, gate_mode)
    log.event("route", route.describe())
    log.save_text_file("route.txt", route.describe() + "\n")
    check_stop_after_stage(run_mode, plan.stage_names())
    return plan


# =================================================================================================
# The flight
# =================================================================================================

class TunnelFirstRun(FallbackRun):

    # ---- Parts course_run and fallback_run build ------------------------------------------------

    def make_safety_watch(self):
        return TunnelFirstSafetyWatch(self.link, self.make_run_safety(), self.plan.start_frame)

    def make_run_safety(self):
        # Targets may reach the cruise height plus the height hold's largest upward correction.
        highest = self.plan.route.cruise_height_metres + LARGEST_UPWARD_CORRECTION_METRES + 0.01
        return RunSafety(self.plan.course_map, self.plan.other_possible_boulders, highest_target_height_metres=highest,
                         tighten_camera_check_when_laser_stops=False)

    def highest_abort_height_metres(self):
        return self.plan.route.cruise_height_metres

    def launch_pose_problem(self, link):
        tracked = link.tracked_position()
        return None if tracked is None else self.pose_problem(tracked)

    def pose_problem(self, tracked_position_metres):
        return start_offset_problem(tracked_position_metres)

    def run(self):
        self.land_straight_down = False
        self.path_ahead_metres = []
        exit_code = super().run()
        self.log.event("height_report", self.watch.height_record.paragraph())
        return exit_code

    # ---- The stages -------------------------------------------------------------------------------

    def fly(self):
        if not self.wait_for_the_start(CLIMB_HEIGHT_METRES):
            return
        self.prepare_the_flight()
        for name, step in self.stages():
            self.begin_stage(name)
            step()
            self.finish_stage()
            if name == self.stop_after_stage:
                self.land_here(f"PRACTICE RUN FINISHED: landed after the stage '{name}', as asked.")
                return
        if self.plan.after_tunnel == AFTER_TUNNEL_PAD:
            self.land_here("RUN FINISHED: landed above the pad after the tunnel.")
        else:
            self.land_here("RUN FINISHED: landed past the tunnel.")

    def prepare_the_flight(self):
        self.flight_start_seconds = self.link.now_seconds()
        self.watch.flight_start_seconds = self.flight_start_seconds
        self.cruise = self.plan.route.cruise_height_metres
        self.height_hold = HeightHeldByCameraTracking(None, self.link)
        self.watch.height_hold = self.height_hold
        self.locked_heading_on_course = None
        self.tunnel_camera = self.start_listening_to_the_camera()
        self.go_start_metres = np.asarray(self.plan.route.look_point_metres, dtype=float)
        self.watch.start_the_heading_guard(heading_split_degrees(self.link))

    def stages(self):
        route, cruise = self.plan.route, self.cruise
        launch, climb_point = route.launch_metres, route.high_climb_point_metres
        steps = [
            (CLIMB_STAGE, lambda: flight_steps.climb_above_launch(self.link, self.watch, CLIMB_HEIGHT_METRES)),
            (SETTLE_STAGE, self.settle_and_measure_the_height_baseline),
            (FORWARD_STAGE, lambda: self.follow([_at(launch, CLIMB_HEIGHT_METRES), _at(climb_point, CLIMB_HEIGHT_METRES),
                                                 _at(climb_point, cruise)], CLIMB_SPEED_METRES_PER_SECOND, HIGH_TARGET_LEAD_METRES)),
        ]
        if route.corridor_end_metres is not None:
            steps.append((CORRIDOR_STAGE, lambda: self.follow([_at(climb_point, cruise), _at(route.corridor_end_metres, cruise)],
                                                              HIGH_SPEED_METRES_PER_SECOND, HIGH_TARGET_LEAD_METRES,
                                                              HIGH_LEG_SPEED_UP_SECONDS)))
        steps += [
            (HIGH_LEG_STAGE, lambda: self.follow([_at(point, cruise) for point in route.high_leg_metres],
                                                 HIGH_SPEED_METRES_PER_SECOND, HIGH_TARGET_LEAD_METRES, HIGH_LEG_SPEED_UP_SECONDS)),
            (DESCENT_STAGE, self.descend_at_s),
            (GATE_STAGE, self.pass_the_height_gate),
            (TURN_STAGE, self.turn_left_180),
            (LOOK_STAGE, self.back_up_and_look),
            (TUNNEL_STAGE, self.go_through_the_tunnel),
        ]
        if self.plan.after_tunnel == AFTER_TUNNEL_PAD:
            steps.append((PAD_STAGE, self.climb_and_fly_to_the_pad))
        return steps

    def start_listening_to_the_camera(self):
        """From the start, so pictures, camera_info and depth points are arriving by the LOOK."""
        from team_nav.tunnel_look_camera import TunnelLookCamera   # Needs ROS, so only loaded to fly.
        return TunnelLookCamera(self.link.node)

    def settle_and_measure_the_height_baseline(self):
        """Holds above the launch mark until steady; b0 is the average of PX4's height minus camera
        tracking's over the steady second."""
        target = _at(self.plan.route.launch_metres, CLIMB_HEIGHT_METRES)
        timer = SettledTimer(SETTLE_SECONDS)
        deadline_seconds = self.link.now_seconds() + LONGEST_SETTLE_SECONDS
        samples = []
        while not timer.update(self.link.now_seconds(), self.steady_above(target)):
            if self.link.now_seconds() > deadline_seconds:
                raise RunMustStop(f"The drone did not steady within {SETTLE_POSITION_ERROR_METRES:.2f} m of the point above "
                                  f"the launch mark, slower than {SETTLE_SPEED_METRES_PER_SECOND:.2f} m/s and with no ground "
                                  f"contact, within {LONGEST_SETTLE_SECONDS:.0f} s of the climb. Landing.")
            samples = self.height_difference_samples(samples, timer)
            self.send_one_cycle(target)
        samples = self.height_difference_samples(samples, timer)
        if not samples:
            raise RunMustStop("PX4's or camera tracking's height was missing while the drone hovered, so the height baseline "
                              "can't be measured. Landing.")
        self.watch.start_the_height_watch(float(np.mean(samples)))
        drone = self.watch.drone_on_course()
        self.log.event("settle", f"Steady above the launch mark: {np.linalg.norm(drone - target):.2f} m from the target.")

    def steady_above(self, target):
        drone = self.watch.drone_on_course()
        if drone is None or px4_reports_ground_contact(self.link):
            return False
        error_metres = float(np.linalg.norm(drone - target))
        return error_metres < SETTLE_POSITION_ERROR_METRES and px4_speed_metres_per_second(self.link) < SETTLE_SPEED_METRES_PER_SECOND

    def height_difference_samples(self, samples, timer):
        """Keeps PX4's height minus camera tracking's for the cycles of the current steady stretch."""
        if timer.held_since_seconds is None:
            return []
        px4, camera = px4_height_metres(self.link), camera_height_metres(self.link)
        if px4 is not None and camera is not None:
            samples.append(px4 - camera)
        return samples

    def descend_at_s(self):
        s_point = self.plan.route.s_metres
        self.follow_with_speeds([_at(s_point, self.cruise), _at(s_point, DESCENT_SLOW_FROM_METRES), _at(s_point, TUNNEL_HEIGHT_METRES)],
                                [0.0, DESCENT_FAST_METRES_PER_SECOND, DESCENT_SLOW_METRES_PER_SECOND], LOW_TARGET_LEAD_METRES)

    def pass_the_height_gate(self):
        """At 0.90 m at S. On a failure the drone lands straight down at S."""
        target_point = _at(self.plan.route.s_metres, TUNNEL_HEIGHT_METRES)
        gate = HeightGate(self.link.now_seconds(), self.plan.gate_mode, px4_position_field(self.link, "z_reset_counter"))
        while True:
            self.send_one_cycle(self.height_hold_correction_target(target_point, flight_steps.CYCLE_SECONDS))
            passed = gate.update(self.link.now_seconds(), px4_estimator_flag(self.link, "cs_rng_hgt"),
                                 px4_estimator_flag(self.link, "cs_rng_kin_consistent"),
                                 px4_position_field(self.link, "z_reset_counter"), self.watch.disagreement())
            if passed:
                self.watch.height_record.gate_result = f"passed ({self.plan.gate_mode})"
                self.log.event("gate", f"HEIGHT GATE PASSED ({self.plan.gate_mode}): disagreement "
                                       f"{self.watch.disagreement():+.3f} m.")
                return
            if gate.failed_because:
                self.watch.height_record.gate_result = f"FAILED ({self.plan.gate_mode}): {gate.failed_because}"
                self.land_straight_down = True
                raise RunMustStop(f"The height gate failed after 4 s at S: {gate.failed_because} (counts of each failing "
                                  f"condition: {gate.fail_counts}). Landing straight down.")

    def turn_left_180(self):
        self.watch.heading_guard_is_on = False
        turn = TurnInProgress(self, _at(self.plan.route.s_metres, TUNNEL_HEIGHT_METRES))
        while not turn.is_done():
            turn.one_step()
        self.locked_heading_on_course = self.plan.start_frame.course_heading_from_start_frame(math.radians(turn.ramp.final_heading_degrees()))
        self.log.event("turn", f"Turn finished in {turn.seconds():.1f} s; the nose is locked at "
                               f"{math.degrees(self.locked_heading_on_course):.1f} degrees on the course (from east toward north).")
        self.watch.heading_guard_is_on = True

    def back_up_and_look(self):
        route = self.plan.route
        backup_metres = float(route.look_point_metres[1] - route.s_metres[1])
        if backup_metres > 0.0:
            self.log.event("look", f"Backing {backup_metres:.2f} m north (nose still south) to the LOOK point "
                                   f"({route.look_point_metres[0]:+.2f}, {route.look_point_metres[1]:+.2f}).")
            self.follow_with_speeds([_at(route.s_metres, TUNNEL_HEIGHT_METRES), _at(route.look_point_metres, TUNNEL_HEIGHT_METRES)],
                                    [0.0, LOOK_BACKUP_SPEED_METRES_PER_SECOND], LOW_TARGET_LEAD_METRES)
            flight_steps.hold_still_for(self.link, self.watch, LOOK_BACKUP_SETTLE_SECONDS)
        self.go_start_metres = LookStep(self, self.tunnel_camera).run_look()

    def go_through_the_tunnel(self):
        """From where the LOOK left the drone, along the tunnel's axis on the map, as far as the planned
        GO line goes; the map's GO line when GO starts at the LOOK point."""
        route = self.plan.route
        start = np.asarray(self.go_start_metres, dtype=float)
        end = np.asarray(route.go_end_metres, dtype=float)
        if np.linalg.norm(start - route.look_point_metres) > 1e-6:
            tube = self.plan.course_map.tubes[0]
            axis = (tube.exit_centre_metres - tube.entrance_centre_metres) / tube.length_metres()
            end = start + axis * float(np.linalg.norm(route.go_end_metres - route.look_point_metres))
        self.go_end_metres = end
        self.log.event("go", f"GO from ({start[0]:+.3f}, {start[1]:+.3f}) to ({end[0]:+.3f}, {end[1]:+.3f}).",
                       go_start_metres=start.tolist(), go_end_metres=end.tolist())
        self.follow([_at(start, TUNNEL_HEIGHT_METRES), _at(end, TUNNEL_HEIGHT_METRES)], TUNNEL_SPEED_METRES_PER_SECOND,
                    LOW_TARGET_LEAD_METRES)

    def climb_and_fly_to_the_pad(self):
        route = self.plan.route
        here = getattr(self, "go_end_metres", route.go_end_metres)
        self.follow([_at(here, TUNNEL_HEIGHT_METRES), _at(route.pad_spot_metres, TUNNEL_HEIGHT_METRES)],
                    TUNNEL_SPEED_METRES_PER_SECOND, LOW_TARGET_LEAD_METRES)
        flight_steps.hold_still_for(self.link, self.watch, PAD_SPOT_HOLD_SECONDS)
        self.follow([_at(route.pad_spot_metres, TUNNEL_HEIGHT_METRES), _at(route.pad_spot_metres, self.cruise)],
                    CLIMB_SPEED_METRES_PER_SECOND, HIGH_TARGET_LEAD_METRES)
        self.follow([_at(point, self.cruise) for point in route.pad_leg_metres], HIGH_SPEED_METRES_PER_SECOND,
                    HIGH_TARGET_LEAD_METRES, HIGH_LEG_SPEED_UP_SECONDS)

    # ---- Moving ---------------------------------------------------------------------------------------

    def follow(self, waypoints_metres, speed_metres_per_second, target_lead_metres, speed_up_seconds=0.0):
        speeds = [0.0] + [speed_metres_per_second] * (len(waypoints_metres) - 1)
        self.follow_with_speeds(waypoints_metres, speeds, target_lead_metres, speed_up_seconds)

    def follow_with_speeds(self, waypoints_metres, speeds, target_lead_metres, speed_up_seconds=0.0):
        """Slides a target along these waypoints (speeds[i] into waypoint i), the height held by camera
        tracking, the nose locked once the turn has set it. With speed_up_seconds the target's speed
        rises from zero to full over that time, for a smooth start."""
        follower = RouteFollower(waypoints_metres, leg_speeds_metres_per_second=speeds, target_lead_metres=target_lead_metres,
                                 finish_tolerance_metres=FINISH_TOLERANCE_METRES)
        self.height_hold.follower = follower
        start_seconds = self.link.now_seconds()
        last_seconds = [start_seconds]

        def send_next_target():
            now = self.link.now_seconds()
            seconds_to_slide = min(now - last_seconds[0], LONGEST_SLIDE_STEP_SECONDS)
            if speed_up_seconds > 0.0:
                seconds_to_slide *= min(1.0, (now - start_seconds) / speed_up_seconds)
            last_seconds[0] = now
            self.path_ahead_metres = [point[:2] for point in waypoints_metres[follower.next_waypoint_number:]]
            target = self.height_hold.next_target(self.watch.drone_on_course(), seconds_to_slide)
            self.watch.send_target_on_course(target, heading_on_course_radians=self.locked_heading_on_course)

        flight_steps.fly_until(self.link, lambda: self.height_hold.has_finished(self.watch.drone_on_course()),
                               send_next_target, LONGEST_STAGE_SECONDS, f"the end of the stage '{self.stage_name}'", self.watch)

    def send_one_cycle(self, target_on_course, heading_on_course_radians=None, cycle_seconds=flight_steps.CYCLE_SECONDS):
        """One cycle of: the safety checks, the target (or the last one again while holding), the record."""
        if self.watch.check():
            self.link.targets.resend_last()
        else:
            self.watch.send_target_on_course(target_on_course, heading_on_course_radians=heading_on_course_radians)
        self.link.record_drone_condition()
        self.link.wait_one_cycle(cycle_seconds)

    def height_hold_correction_target(self, target_on_course, seconds_since_last_target=STEP_SECONDS):
        """The target with its height moved by the height hold's correction, updated for one step."""
        self.height_hold.update(seconds_since_last_target)
        corrected = list(target_on_course)
        corrected[2] += self.height_hold.correction_metres
        return corrected

    def tunnel_height_metres(self):
        return TUNNEL_HEIGHT_METRES

    def low_target_lead_metres(self):
        return LOW_TARGET_LEAD_METRES

    # ---- Landing ----------------------------------------------------------------------------------------

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

    def land_after_a_problem(self, land_using_laser):
        """Straight down after a failed gate; by the height watch's rules above 1.5 m; otherwise
        fallback_run's two-way abort."""
        if not (self.link.control_is_ours() and self.link.is_armed()):
            super().land_after_a_problem(land_using_laser)
            return
        if getattr(self, "land_straight_down", False):
            CourseRun.land_after_a_problem(self, land_using_laser)
            return
        drone = self.watch.drone_on_course()
        if drone is not None and drone[2] > WATCH_ABOVE_METRES:
            try:
                if self.come_down_from_high(drone):
                    return
            except ControlIsNoLongerOurs as reason:
                self.log.event("control_lost", f"Control was taken away while coming down. {reason}")
                return
            except KeyboardInterrupt:
                self.log.event("interrupted", "Interrupted again while coming down: the drone holds its last target.")
                return
        super().land_after_a_problem(land_using_laser)

    def come_down_from_high(self, drone):
        """Returns True once landed; False if no open floor is near (then fallback_run's abort flies on)."""
        spot = choose_descent_point(self.plan.course_map, drone, getattr(self, "path_ahead_metres", []))
        if spot is None:
            self.log.event("abort", f"Above {WATCH_ABOVE_METRES:.1f} m over a wall, with no open floor within "
                                    f"{DESCENT_SEARCH_METRES:.1f} m: flying on at this height to a landing spot.")
            return False
        height = float(drone[2])
        if np.linalg.norm(spot - drone[:2]) > 0.02:
            self.log.event("abort", f"Over a wall at {height:.2f} m: keeping the height and moving {np.linalg.norm(spot - drone[:2]):.2f} m "
                                    f"to open floor at ({spot[0]:+.2f}, {spot[1]:+.2f}).")
            self.slide_without_checks([drone[0], drone[1], height], [spot[0], spot[1], height], HIGH_SIDEWAYS_METRES_PER_SECOND)
        self.log.event("abort", f"Over open floor: descending at {HIGH_DESCENT_METRES_PER_SECOND} m/s, then the normal landing.")
        self.slide_without_checks([spot[0], spot[1], height], [spot[0], spot[1], HAND_OVER_TO_LANDING_METRES],
                                  HIGH_DESCENT_METRES_PER_SECOND)
        CourseRun.land_after_a_problem(self, False)
        return True

    def slide_without_checks(self, start, end, speed_metres_per_second):
        """Slides a target from start to end at this speed, the nose as locked, with the height hold's
        correction frozen. Only control is checked: the run has already stopped."""
        start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        length = float(np.linalg.norm(end - start))
        seconds_needed = length / speed_metres_per_second
        begin = self.link.now_seconds()
        correction = self.height_hold.correction_metres if self.height_hold is not None else 0.0
        while True:
            if not (self.link.control_is_ours() and self.link.is_armed()):
                raise ControlIsNoLongerOurs("The drone is disarmed or no longer in Offboard mode.")
            fraction = 1.0 if seconds_needed <= 0 else min(1.0, (self.link.now_seconds() - begin) / seconds_needed)
            target = start + fraction * (end - start)
            target[2] += correction
            self.watch.send_target_on_course(target, check_distance_from_drone=False,
                                             heading_on_course_radians=self.locked_heading_on_course)
            self.link.record_drone_condition()
            self.link.wait_one_cycle()
            drone = self.watch.drone_on_course()
            if fraction >= 1.0 and (drone is None or np.linalg.norm(drone - end) < 0.15
                                    or self.link.now_seconds() - begin > seconds_needed + 5.0):
                return


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
