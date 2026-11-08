"""Flies from the end of the climb to above the landing pad with live navigation: the drone looks
with its depth camera, keeps a live map, re-plans its path and faces where it flies.

REAL DRONE CODE, also used unchanged in the simulator. course_run.py calls it after the climb,
when --navigation is "live" (the default).

The three layers, from slow to fast
-----------------------------------
  - Top, about 2 times a second: "where do I go?" live_navigation.LiveNavigator plans a path on
    the live map to the next stage's goal (from the mission and the map), and plans again as the
    map changes.
  - Middle, about 10 times a second: "what is around me?" depth_camera.py measures distances;
    depth_to_map.py places them on the live map (live_map.py). Camera tracking (cuVSLAM, the
    organizers' program) says where the drone is. wall_alignment.py could correct its slow drift
    by lining up the depth points with the walls we are sure of, and height_keeper.py could keep
    the true height if PX4's own height drifts, but both are switched off (see LiveFlight.__init__
    and USE_HEIGHT_KEEPER). Each cycle the live map's scores fade a little (live_map.py).
  - Bottom, hundreds of times a second: PX4 keeps the drone level and flies it to our target.
    We don't change it.
Between the middle and the bottom sits a reflex: every cycle, the target is put no further
ahead than the live map says is free. So a wall the camera has just seen stops the drone at
once, without waiting for a new plan. And if depth pictures stop coming, the drone holds still
(it may still turn) until they come back: it never moves into space it can't see.

Two clocks end the live flight early, and the drone then lands where it is (checkpoints already
cleared still count, and a landing is better than the organizers cutting the motors in the air):
  - the run's time budget: course_run.py passes latest_navigation_end_seconds, 165 s after the
    hand-over (15 s kept for the landing inside the rulebook's 180 s);
  - no progress: the drone has not got NO_PROGRESS_DISTANCE_METRES nearer its current goal for
    NO_PROGRESS_SECONDS. In the 10:23 and 10:44 runs of 2026-11-08 the organizers cut the motors
    after about 20 to 35 s of little progress.
All times come from link.now_seconds(), so a test bench can run faster than real time.

What it does not do: climb, land, or decide when the run must stop for safety (course_run.py,
flight_steps.py and run_safety.py do those; the safety checks run every cycle here too).
"""
import json
import math
import os

import numpy as np

from team_nav import flight_steps
from team_nav.depth_to_map import DepthToMap
from team_nav.drone_dimensions import DRONE_HALF_HEIGHT_METRES
from team_nav.flight_steps import RunMustStop
from team_nav.height_keeper import HeightKeeper
from team_nav.live_map import LiveMap
from team_nav.live_navigation import MOVING_ACTIVITIES, LiveNavigator, NoWayForward
from team_nav.live_planner import goals_from_mission
from team_nav.waypoint_route import WaypointNavigator

# Our own choice, well under the touch check's limit (touch_detector.py: 120 degrees a second
# counts as a hit). The nose is turned toward the wanted heading at most this fast. On drone 16
# on 2026-11-08 (run 20261108_050406), a 90 degree turn at 30 degrees a second made the drone
# slide about 0.5 m from the point it was told to hold: PX4 holds its place badly while it turns.
# Turning half as fast gives it less to correct: in the 10:23 and 10:44 runs, turns from rest at
# this rate slid only 0.08 to 0.12 m (measured), and PX4 followed them with a 0.40 s delay.
TURN_RATE_DEGREES_PER_SECOND = 15.0

# Our own choice. While the drone is moving, its nose follows the path at most this fast.
# Measured on drone 16 on 2026-11-08: a turn started while moving made it slide 0.28 m, against
# 0.08 to 0.12 m for a turn from rest. Big turns therefore happen only from rest
# (live_navigation.py); in flight the nose only drifts slowly after a gentle bend in the path.
FLYING_TURN_RATE_DEGREES_PER_SECOND = 5.0

# Our own choice. A last limit on the live flight, used only when course_run.py gives no time
# budget. The course is about 18 m; at 0.3 m/s with turns and holds that is about 2 minutes.
LONGEST_LIVE_FLIGHT_SECONDS = 300.0

# Our own choices. If the drone has not got this much nearer its current goal for this long, it
# lands where it is. 25 s is a little less than the 20 to 35 s of little progress after which the
# organizers cut the motors on 2026-11-08. 0.3 m is more than the drone's hover wander (1 to 3 cm,
# measured) and its turn slides (up to 0.12 m from rest, measured), so only real progress counts.
# Distance is measured in a straight line to the goal point, which a detour may not shorten at
# first; the course's detours are short compared with 25 s of flying.
NO_PROGRESS_SECONDS = 25.0
NO_PROGRESS_DISTANCE_METRES = 0.3

# Our own choice. If no depth picture has reached the live map for this long, the drone may turn
# and hold but not move: it would be flying into space it can't see. The depth program makes
# up to 10 pictures a second (about 30 ms each on drone 16's GPU, measured 2026-11-08), so 1.5 s
# means many have been missed.
LONGEST_SECONDS_WITHOUT_DEPTH = 1.5

# Our own choice, 2026-11-08: the height keeper is OFF. It trusts camera tracking's height, and
# that was wrong by about 10 cm both in Gazebo (run 20261107_211724) and probably on drone 16
# (correction 0.08 m within 4 s of run 20261108_050406). The correction then moved the drone away
# from the height the bottom laser measures, and the height safety check stopped the run. Without
# it, PX4 holds the height with the bottom laser; over a boulder it climbs about the boulder's
# height for a moment, which is still far below the walls.
USE_HEIGHT_KEEPER = False

# Our own choice. The live map and the path are saved this often, to study the flight later.
SECONDS_BETWEEN_SAVED_MAPS = 2.0


class LiveFlight:

    def __init__(self, link, watch, course_map, mission, landing_stage_name, stop_after_stage, stage_records,
                 waypoint_route=None, latest_navigation_end_seconds=None, *, full_course_practice=False):
        """stage_records is an object with begin_stage(name) and finish_stage(), as CourseRun has."""
        self.link = link
        self.watch = watch
        self.stage_records = stage_records
        cruise_height_metres = mission.cruise_height_metres
        self.live_map = LiveMap(course_map, cruise_height_metres, DRONE_HALF_HEIGHT_METRES)
        if waypoint_route is None:
            goals = goals_from_mission(course_map, mission, landing_stage_name)
            self.navigator = LiveNavigator(self.live_map, goals, cruise_height_metres, stop_after_stage,
                                           full_course_practice=full_course_practice)
        else:
            self.navigator = WaypointNavigator(self.live_map, course_map, waypoint_route, landing_stage_name,
                                               stop_after_stage, watch.start_frame.start_heading_radians)
        self.latest_navigation_end_seconds = latest_navigation_end_seconds
        # Drift correction against the map's walls (wall_alignment.py) is off in every mode. It has
        # never been checked in flight; in the 10:44 run of 2026-11-08 it moved the drone's position
        # 0.14 m in 24 s, probably on the depth camera's wrong near points. A sudden wrong shift near
        # a wall is a bigger danger than camera tracking's slow drift over a three-minute run.
        self.depth = DepthToMap(link, watch.start_frame, self.live_map, course_map, align_to_walls=False)
        self.commanded_heading_radians = None
        self.last_fade_seconds = None
        self.stages_finished = []              # Names of the stages (checkpoints) finished so far.
        # The goal number, the nearest the drone has been to that goal, and when it got there.
        self.progress_goal_number = None
        self.nearest_to_goal_metres = None
        self.nearest_to_goal_seconds = None
        self.waiting_for_depth_at = None       # Where the drone holds while depth is missing.
        self.height_keeper = HeightKeeper()
        self.cruise_height_metres = cruise_height_metres
        self.last_cycle_seconds = None
        self.finished = False
        self.saved_maps = 0
        self.last_saved_map_seconds = -math.inf
        self.maps_folder = os.path.join(link.log.folder, "live_maps")
        os.makedirs(self.maps_folder, exist_ok=True)
        self.link.log.event("live_navigation", "Live navigation goals: " + ", ".join(
            f"{goal.stage_name} at ({goal.point_metres[0]:.2f}, {goal.point_metres[1]:.2f})"
            for goal in self.navigator.goals))

    def fly(self):
        """Flies until the last goal is reached. Raises RunMustStop if there is no way forward, the
        time budget runs out, or the drone stops making progress: the drone then lands where it is."""
        self.stage_records.begin_stage(self.navigator.current_goal.stage_name)
        self._stop_if_out_of_time(self.link.now_seconds())
        try:
            # The time budget is checked every cycle, also while a safety hold stops targets from
            # being planned (_finished_or_out_of_time); this limit is only a last backstop.
            flight_steps.fly_until(self.link, self._finished_or_out_of_time, self._send_next_target,
                                   LONGEST_LIVE_FLIGHT_SECONDS, "navigation to reach the landing pad", self.watch)
        except NoWayForward as reason:
            raise RunMustStop(str(reason)) from reason
        finally:
            self._save_map(self.link.now_seconds(), final=True)
        self.stage_records.finish_stage()

    def _send_next_target(self):
        now = self.link.now_seconds()
        self._stop_if_out_of_time(now)
        drone, heading = self._drone_now()
        self._stop_if_stuck(drone, now)
        self._fade_map(now)
        step = self.navigator.next_step(drone, heading, now, self.live_map.updates)
        if step.warning:
            self.link.log.event("warning", step.warning)
        self.stages_finished += step.stages_finished
        for _name in step.stages_finished:
            if not step.finished:
                self.stage_records.finish_stage()
        for name in step.stages_begun:
            self.stage_records.begin_stage(name)
        self.finished = step.finished
        self.link.stage = self.stage_records.stage_name
        target, activity = self._held_if_depth_is_missing(step, drone)
        self._keep_true_height(drone[2], now)
        wanted_heading = heading if self.waiting_for_depth_at is not None else step.heading_radians
        turn_rate = TURN_RATE_DEGREES_PER_SECOND
        if step.activity in MOVING_ACTIVITIES:
            turn_rate = FLYING_TURN_RATE_DEGREES_PER_SECOND
        heading_to_send = self._turned_gently_toward(wanted_heading, heading, now, turn_rate)
        self.watch.send_target_on_course(target, heading_on_course_radians=heading_to_send)
        self.link.log.sample("live_navigation", activity=activity,
                             wanted_heading_degrees=round(math.degrees(step.heading_radians), 1),
                             sent_heading_degrees=round(math.degrees(heading_to_send), 1),
                             heading_degrees=round(math.degrees(heading), 1),
                             depth_pictures_used=self.depth.pictures_used,
                             height_correction_metres=round(self.link.height_correction_metres, 3),
                             drift_correction_east_metres=round(float(self.watch.start_frame.drift_correction_metres[0]), 3),
                             drift_correction_north_metres=round(float(self.watch.start_frame.drift_correction_metres[1]), 3),
                             path_squares=0 if self.navigator.path is None else len(self.navigator.path))
        self._save_map(now)

    def _finished_or_out_of_time(self):
        """True once the last goal is reached. Asked every cycle, even during a safety hold, so the
        time budget runs out on time while the drone holds still."""
        if self.finished:
            return True
        self._stop_if_out_of_time(self.link.now_seconds())
        return False

    def _stop_if_out_of_time(self, now_seconds):
        """Raises RunMustStop once the time budget for navigation is used up."""
        if self.latest_navigation_end_seconds is None or now_seconds < self.latest_navigation_end_seconds:
            return
        raise RunMustStop("The time for navigation is used up (course_run.py keeps a landing reserve inside the "
                          "rulebook's 180 s, counted from the hand-over). Landing where the drone is. "
                          + self._checkpoints_cleared())

    def _stop_if_stuck(self, drone_metres, now_seconds):
        """Raises RunMustStop if the drone has not got NO_PROGRESS_DISTANCE_METRES nearer its current
        goal for NO_PROGRESS_SECONDS. A new goal starts the count again."""
        distance_metres = float(np.linalg.norm(self.navigator.current_goal.point_metres - drone_metres[:2]))
        new_goal = self.progress_goal_number != self.navigator.goal_number
        if new_goal or distance_metres <= self.nearest_to_goal_metres - NO_PROGRESS_DISTANCE_METRES:
            self.progress_goal_number = self.navigator.goal_number
            self.nearest_to_goal_metres = distance_metres
            self.nearest_to_goal_seconds = now_seconds
            return
        if now_seconds - self.nearest_to_goal_seconds < NO_PROGRESS_SECONDS:
            return
        raise RunMustStop(f"The drone has not got {NO_PROGRESS_DISTANCE_METRES:.1f} m nearer to "
                          f"'{self.navigator.current_goal.stage_name}' for {NO_PROGRESS_SECONDS:.0f} s. Landing where "
                          f"it is, before the organizers have to stop it. " + self._checkpoints_cleared())

    def _checkpoints_cleared(self):
        """A sentence naming the checkpoints the navigator has counted so far."""
        if not self.stages_finished:
            return "No checkpoint was cleared."
        return "Checkpoints cleared: " + ", ".join(self.stages_finished) + "."

    def _fade_map(self, now_seconds):
        """Lets the live map's scores fade toward the map's first guess (live_map.LiveMap.fade)."""
        if self.last_fade_seconds is not None:
            self.live_map.fade(now_seconds - self.last_fade_seconds)
        self.last_fade_seconds = now_seconds

    def _keep_true_height(self, camera_height_metres, now_seconds):
        """Updates the height correction (height_keeper.py) once per cycle. It uses the same
        cycle time as the turning, so it is called just before _turned_gently_toward."""
        if not USE_HEIGHT_KEEPER:
            return
        seconds = 0.0 if self.last_cycle_seconds is None else min(now_seconds - self.last_cycle_seconds,
                                                                   2.0 * flight_steps.CYCLE_SECONDS)
        self.link.height_correction_metres = self.height_keeper.update(self.cruise_height_metres,
                                                                       camera_height_metres, seconds)

    def _held_if_depth_is_missing(self, step, drone):
        """The step's target, or, if depth pictures have stopped coming, the point where the drone
        is now: never move into space the camera hasn't just looked at."""
        if self.depth.seconds_since_last_picture() <= LONGEST_SECONDS_WITHOUT_DEPTH:
            self.waiting_for_depth_at = None
            return step.target_metres, step.activity
        if self.waiting_for_depth_at is None:
            self.waiting_for_depth_at = np.array([drone[0], drone[1], step.target_metres[2]])
            self.link.log.event("warning", "No depth picture for over "
                                           f"{LONGEST_SECONDS_WITHOUT_DEPTH:.1f} s: holding until depth comes back.")
        return self.waiting_for_depth_at, "holding, no depth pictures"

    def _drone_now(self):
        drone = self.watch.drone_on_course()
        heading = self.watch.drone_heading_on_course()
        if drone is None or heading is None:
            raise RunMustStop("Camera tracking has given no position or heading, so live navigation can't place "
                              "the drone on its map. Check that camera tracking is running.")
        return drone, heading

    def _turned_gently_toward(self, wanted_radians, drone_heading_radians, now_seconds,
                              turn_rate_degrees_per_second=TURN_RATE_DEGREES_PER_SECOND):
        """The heading to send this cycle: a step from the last one toward the wanted heading,
        no bigger than turn_rate_degrees_per_second allows."""
        if self.commanded_heading_radians is None:
            self.commanded_heading_radians = drone_heading_radians
            self.last_cycle_seconds = now_seconds
        seconds = min(now_seconds - self.last_cycle_seconds, 2.0 * flight_steps.CYCLE_SECONDS)
        self.last_cycle_seconds = now_seconds
        largest_step = math.radians(turn_rate_degrees_per_second) * seconds
        difference = math.atan2(math.sin(wanted_radians - self.commanded_heading_radians),
                                math.cos(wanted_radians - self.commanded_heading_radians))
        self.commanded_heading_radians += max(-largest_step, min(largest_step, difference))
        return self.commanded_heading_radians

    def _save_map(self, now_seconds, final=False):
        """Saves the live map's scores and the current path, for plotting after the flight."""
        if not final and now_seconds - self.last_saved_map_seconds < SECONDS_BETWEEN_SAVED_MAPS:
            return
        self.last_saved_map_seconds = now_seconds
        name = "final" if final else f"{self.saved_maps:04d}"
        self.saved_maps += 1
        drone = self.watch.drone_on_course()
        np.savez_compressed(os.path.join(self.maps_folder, f"live_map_{name}.npz"),
                            scores=self.live_map.scores,
                            lowest_corner_metres=self.live_map.lowest_corner_metres,
                            path_metres=np.asarray(self.navigator.path if self.navigator.path else np.zeros((0, 2))),
                            drone_metres=np.asarray(drone if drone is not None else [np.nan] * 3),
                            seconds=now_seconds)
        if final:
            summary = {"depth_pictures_used": self.depth.pictures_used,
                       "drift_correction_metres": np.round(self.watch.start_frame.drift_correction_metres, 3).tolist(),
                       "pictures_used_for_drift_correction": self.depth.wall_alignment.pictures_used,
                       "depth_pictures_without_pose": self.depth.pictures_without_pose,
                       "maps_saved": self.saved_maps}
            self.link.log.save_text_file("live_navigation_summary.json", json.dumps(summary, indent=2) + "\n")
