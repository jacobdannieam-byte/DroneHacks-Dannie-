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
    organizers' program) says where the drone is. wall_alignment.py corrects its slow drift by
    lining up the depth points with the walls we are sure of, height_keeper.py can keep the true
    height if PX4's own height drifts (for example after flying over a boulder), but it is switched off (USE_HEIGHT_KEEPER).
  - Bottom, hundreds of times a second: PX4 keeps the drone level and flies it to our target.
    We don't change it.
Between the middle and the bottom sits a reflex: every cycle, the target is put no further
ahead than the live map says is free. So a wall the camera has just seen stops the drone at
once, without waiting for a new plan. And if depth pictures stop coming, the drone holds still
(it may still turn) until they come back: it never moves into space it can't see.

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
from team_nav.live_navigation import LiveNavigator, NoWayForward
from team_nav.live_planner import goals_from_mission

# Our own choice, well under the touch check's limit (touch_detector.py: 120 degrees a second
# counts as a hit). The nose is turned toward the wanted heading at most this fast. On drone 16
# on 2026-11-08 (run 20261108_050406), a 90 degree turn at 30 degrees a second made the drone
# slide about 0.5 m from the point it was told to hold: PX4 holds its place badly while it turns.
# Turning half as fast gives it less to correct.
TURN_RATE_DEGREES_PER_SECOND = 15.0

# Our own choice. The whole live flight must end within this. The course is about 18 m; at
# 0.3 m/s with turns and look-arounds that is about 2 minutes.
LONGEST_LIVE_FLIGHT_SECONDS = 300.0

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

    def __init__(self, link, watch, course_map, mission, landing_stage_name, stop_after_stage, stage_records):
        """stage_records is an object with begin_stage(name) and finish_stage(), as CourseRun has."""
        self.link = link
        self.watch = watch
        self.stage_records = stage_records
        cruise_height_metres = mission.cruise_height_metres
        self.live_map = LiveMap(course_map, cruise_height_metres, DRONE_HALF_HEIGHT_METRES)
        goals = goals_from_mission(course_map, mission, landing_stage_name)
        self.navigator = LiveNavigator(self.live_map, goals, cruise_height_metres, stop_after_stage)
        self.depth = DepthToMap(link, watch.start_frame, self.live_map, course_map)
        self.commanded_heading_radians = None
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
            f"{goal.stage_name} at ({goal.point_metres[0]:.2f}, {goal.point_metres[1]:.2f})" for goal in goals))

    def fly(self):
        """Flies until the last goal is reached. Raises RunMustStop if there is no way forward."""
        # No look-around at the start: the drone already faces the first corridor, and on drone 16
        # every turn makes it slide (see TURN_RATE_DEGREES_PER_SECOND). The navigator still looks
        # around if it finds no path.
        self.stage_records.begin_stage(self.navigator.current_goal.stage_name)
        try:
            flight_steps.fly_until(self.link, lambda: self.finished, self._send_next_target,
                                   LONGEST_LIVE_FLIGHT_SECONDS, "live navigation to reach the landing pad", self.watch)
        except NoWayForward as reason:
            raise RunMustStop(str(reason)) from reason
        finally:
            self._save_map(self.link.now_seconds(), final=True)
        self.stage_records.finish_stage()

    def _send_next_target(self):
        now = self.link.now_seconds()
        drone, heading = self._drone_now()
        step = self.navigator.next_step(drone, heading, now)
        for _name in step.stages_finished:
            if not step.finished:
                self.stage_records.finish_stage()
        for name in step.stages_begun:
            self.stage_records.begin_stage(name)
        self.finished = step.finished
        self.link.stage = self.stage_records.stage_name
        target, activity = self._held_if_depth_is_missing(step, drone)
        self._keep_true_height(drone[2], now)
        heading_to_send = self._turned_gently_toward(step.heading_radians, heading, now)
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

    def _turned_gently_toward(self, wanted_radians, drone_heading_radians, now_seconds):
        """The heading to send this cycle: a step from the last one toward the wanted heading,
        no bigger than TURN_RATE_DEGREES_PER_SECOND allows."""
        if self.commanded_heading_radians is None:
            self.commanded_heading_radians = drone_heading_radians
            self.last_cycle_seconds = now_seconds
        seconds = min(now_seconds - self.last_cycle_seconds, 2.0 * flight_steps.CYCLE_SECONDS)
        self.last_cycle_seconds = now_seconds
        largest_step = math.radians(TURN_RATE_DEGREES_PER_SECOND) * seconds
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
