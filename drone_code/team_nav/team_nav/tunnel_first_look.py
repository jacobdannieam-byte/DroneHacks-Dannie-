"""The tunnel_first mission's LOOK step: at the tunnel height, before GO, the camera finds the
tunnel's centre and the drone corrects its line sideways.

REAL DRONE CODE, also run unchanged in the simulators. tunnel_first_run.py calls LookStep.run()
between the descent and GO; tunnel_look.py does the picture arithmetic; tunnel_look_camera.py
collects the pictures.

What it does (the team's design, 2026-11-08)
-------------------------------------------
  1. Holds still at the descent's end until the drone moves slower than STILL_SPEED and turns
     slower than STILL_ROTATION, then collects pictures until 5 agree (tunnel_look.judge_frames),
     within LOOK_DEADLINE_SECONDS.
  2. If one of the tunnel's edges is cut off by the picture's border, slides SLIDE_METRES toward
     that side and looks again, at most LARGEST_SLIDES times.
  3. Checks the edge method against the stereo depth points; more than 0.10 m apart is "unsure".
  4. Refuses a correction over LARGEST_CORRECTION_METRES (unsure). Otherwise moves sideways by at
     most CORRECTION_STEP_METRES at CORRECTION_SPEED, looks again, and repeats until what is left
     is at most GOOD_ENOUGH_METRES.
  5. GO then starts from the corrected point and flies along the tunnel's axis from the map.
It happens at the LOOK point, north of S by TEAM_NAV_LOOK_BACKUP_METRES (tunnel_first_route.py).
When unsure: with TEAM_NAV_TUNNEL_IF_UNSURE=map (the default) the drone goes back to the LOOK point
(on the map's line) and flies
Phase 1's map line, so the camera can only improve on Phase 1; with =land it lands where it is.
It never waits for depth pictures beyond the look's deadline.

The records: every look's numbers in look.jsonl, and for each look one picture (look_<n>.png) and
its band with the chosen edges in red and the support profile in green (look_<n>_band.png).

What it does not do: fly GO, or decide anything about the rest of the flight.
"""
import json
import os

import numpy as np

from team_nav import flight_steps
from team_nav.camera_pictures import colour_array_to_png, picture_message_to_png
from team_nav.tunnel_look import (LARGEST_DISAGREEMENT_METRES, band_picture_with_edges, body_left_on_course,
                                  depth_left_offset_metres, find_opening_edges, judge_frames)

IF_UNSURE_SETTING = "TEAM_NAV_TUNNEL_IF_UNSURE"
IF_UNSURE_MAP = "map"
IF_UNSURE_LAND = "land"

# The team's choices (2026-11-08).
STILL_SPEED_METRES_PER_SECOND = 0.05
STILL_ROTATION_DEGREES_PER_SECOND = 2.0
LOOK_DEADLINE_SECONDS = 3.0
SLIDE_METRES = 0.10
LARGEST_SLIDES = 3
LARGEST_CORRECTION_METRES = 0.35
CORRECTION_STEP_METRES = 0.10
CORRECTION_SPEED_METRES_PER_SECOND = 0.08
GOOD_ENOUGH_METRES = 0.05
# Our own choice: at most this many correction steps (0.35 m in 0.10 m steps needs 4).
LARGEST_CORRECTION_STEPS = 5


def if_unsure_from_environment():
    value = os.environ.get(IF_UNSURE_SETTING, IF_UNSURE_MAP).strip().lower() or IF_UNSURE_MAP
    if value not in (IF_UNSURE_MAP, IF_UNSURE_LAND):
        raise ValueError(f"{IF_UNSURE_SETTING} is '{value}'. Use '{IF_UNSURE_MAP}' (fly the map's line, the default) or "
                         f"'{IF_UNSURE_LAND}'.")
    return value


class LookUnsure(Exception):
    """The look could not be trusted; the message says why."""


class LookStep:

    def __init__(self, run, camera):
        self.run, self.link, self.watch, self.log = run, run.link, run.watch, run.log
        self.camera = camera
        self.look_number = 0
        self.records_folder = getattr(self.log, "folder", None)

    def run_look(self):
        """Returns the point (east, north) GO starts from: corrected, or the LOOK point (on the map's line,
        north of S) if unsure (with =map).
        Raises flight_steps.RunMustStop if unsure with =land."""
        s_point = np.asarray(self.run.plan.route.look_point_metres, dtype=float)
        try:
            corrected = self.look_and_correct(s_point.copy())
        except LookUnsure as unsure:
            return self.when_unsure(str(unsure), s_point)
        self.log.event("look", f"LOOK accepted: GO starts {np.linalg.norm(corrected - s_point):.2f} m from S, at "
                               f"({corrected[0]:+.3f}, {corrected[1]:+.3f}).", go_start_metres=corrected.tolist())
        return corrected

    def look_and_correct(self, point):
        slides = 0
        total_correction = np.zeros(2)
        for _ in range(LARGEST_CORRECTION_STEPS + LARGEST_SLIDES + 1):
            verdict, depth_offset = self.look_once()
            if not verdict.accepted and verdict.clipped_side and slides < LARGEST_SLIDES:
                slides += 1
                point = self.slide_toward(point, verdict.clipped_side)
                total_correction = point - np.asarray(self.run.plan.route.look_point_metres)
                continue
            self.refuse_if_unsure(verdict, depth_offset, total_correction)
            if abs(verdict.left_offset_metres) <= GOOD_ENOUGH_METRES:
                return point
            step = float(np.clip(verdict.left_offset_metres, -CORRECTION_STEP_METRES, CORRECTION_STEP_METRES))
            point = self.move_left_by(point, step)
            total_correction = point - np.asarray(self.run.plan.route.look_point_metres)
        raise LookUnsure("the correction did not settle within 0.05 m after the allowed steps")

    def refuse_if_unsure(self, verdict, depth_offset, total_correction):
        if not verdict.accepted:
            raise LookUnsure(f"the pictures did not give a steady answer ({verdict.reason})")
        if depth_offset is None:
            raise LookUnsure("the stereo depth points did not show both of the tunnel's walls")
        if abs(depth_offset - verdict.left_offset_metres) > LARGEST_DISAGREEMENT_METRES:
            raise LookUnsure(f"the edges say {verdict.left_offset_metres:+.2f} m and the depth points say "
                             f"{depth_offset:+.2f} m, more than {LARGEST_DISAGREEMENT_METRES:.2f} m apart")
        wanted_total = float(np.linalg.norm(total_correction)) + abs(verdict.left_offset_metres)
        if wanted_total > LARGEST_CORRECTION_METRES:
            raise LookUnsure(f"the correction would be {wanted_total:.2f} m in all, more than the "
                             f"{LARGEST_CORRECTION_METRES:.2f} m allowed")

    def look_once(self):
        """Waits until still, collects frames until they agree or the deadline passes. Returns the
        verdict and the stereo depth method's offset (or None)."""
        self.look_number += 1
        self.rejected_frames = 0
        looks, stamps, last_picture = [], [], None
        deadline_seconds = self.link.now_seconds() + LOOK_DEADLINE_SECONDS
        verdict = None
        while self.link.now_seconds() < deadline_seconds:
            # The height hold is updated every cycle: in Gazebo (2026-11-08) the drone sank 0.33 m during
            # a 3 s look with a fixed target, and the hold then took the change for a jump and froze.
            self.run.send_one_cycle(self.current_target(), heading_on_course_radians=self.run.locked_heading_on_course)
            new_picture = self.camera.newest_picture()
            lens = self.camera.lens()
            if new_picture is None or lens is None or not self.drone_is_still():
                continue
            if stamps and new_picture[0] <= stamps[-1]:
                continue
            last_picture = new_picture
            look = find_opening_edges(new_picture[1], *lens, points_from_camera=self.camera.latest_points_from_camera,
                                      check_depth=True)
            if look.rejected_because:
                self.note_rejected_pair(look.rejected_because)
            looks.append(look)
            stamps.append(new_picture[0])
            verdict = judge_frames(looks, stamps)
            if verdict.accepted:
                break
        if verdict is None:
            verdict = judge_frames(looks or [], stamps)
        depth_offset = None
        if verdict.accepted and self.camera.latest_points_from_camera is not None:
            depth_offset, _, _ = depth_left_offset_metres(self.camera.latest_points_from_camera, verdict.distance_metres)
        self.record(looks, verdict, depth_offset, last_picture)
        return verdict, depth_offset

    def note_rejected_pair(self, why):
        """Counts the frames whose pairs all failed the depth check; writes down the first reason."""
        self.rejected_frames = getattr(self, "rejected_frames", 0) + 1
        if getattr(self, "rejected_in_look", None) == self.look_number:
            return
        self.rejected_in_look = self.look_number
        self.log.event("look", f"Edge pairs rejected by the depth check ({why}).")

    def drone_is_still(self):
        from team_nav.tunnel_first_readings import px4_speed_metres_per_second
        rotation = self.link.rotation_degrees_per_second() or 0.0
        return px4_speed_metres_per_second(self.link) < STILL_SPEED_METRES_PER_SECOND and abs(rotation) < STILL_ROTATION_DEGREES_PER_SECOND

    def current_target(self):
        target = getattr(self, "target_point", self.run.plan.route.look_point_metres)
        return self.run.height_hold_correction_target([target[0], target[1], self.run.tunnel_height_metres()],
                                                      flight_steps.CYCLE_SECONDS)

    def move_left_by(self, point, left_metres):
        """Moves the drone left (negative: right) by left_metres at CORRECTION_SPEED, nose locked."""
        left_direction = body_left_on_course(self.run.locked_heading_on_course)
        new_point = point + left_metres * left_direction
        height = self.run.tunnel_height_metres()
        self.log.event("look", f"Correcting {abs(left_metres):.2f} m to the {'left' if left_metres > 0 else 'right'}: "
                               f"to ({new_point[0]:+.3f}, {new_point[1]:+.3f}).", left_metres=left_metres)
        self.run.follow_with_speeds([[point[0], point[1], height], [new_point[0], new_point[1], height]],
                                    [0.0, CORRECTION_SPEED_METRES_PER_SECOND], self.run.low_target_lead_metres())
        self.target_point = new_point
        return new_point

    def slide_toward(self, point, side):
        self.log.event("look", f"The {side} edge is cut off by the picture's border: sliding {SLIDE_METRES:.2f} m {side}.")
        return self.move_left_by(point, SLIDE_METRES if side == "left" else -SLIDE_METRES)

    def when_unsure(self, why, s_point):
        choice = if_unsure_from_environment()
        self.log.event("look", f"LOOK UNSURE: {why}. {IF_UNSURE_SETTING}={choice}.", reason=why, if_unsure=choice)
        if choice == IF_UNSURE_LAND:
            raise flight_steps.RunMustStop(f"The tunnel LOOK was unsure ({why}) and {IF_UNSURE_SETTING}=land. Landing.")
        here = self.watch.drone_on_course()
        if here is not None and np.linalg.norm(here[:2] - s_point) > 0.03:
            self.move_back_to(s_point, here)
        return s_point

    def move_back_to(self, s_point, here):
        height = self.run.tunnel_height_metres()
        self.run.follow_with_speeds([[here[0], here[1], height], [s_point[0], s_point[1], height]],
                                    [0.0, CORRECTION_SPEED_METRES_PER_SECOND], self.run.low_target_lead_metres())

    # ---- Records ------------------------------------------------------------------------------------

    def record(self, looks, verdict, depth_offset, last_picture):
        good = [look for look in looks if look.found_both()]
        last = looks[-1] if looks else None
        entry = {
            "look_number": self.look_number, "frames": len(looks), "frames_with_both_edges": len(good),
            "accepted": verdict.accepted, "reason": verdict.reason, "clipped_side": verdict.clipped_side,
            "frames_rejected_by_depth": getattr(self, "rejected_frames", 0),
            "depth_points_available": self.camera.latest_points_from_camera is not None,
            "uL": None if last is None else last.left_column, "uR": None if last is None else last.right_column,
            "w_pixels": None if last is None or not last.found_both() else round(last.width_pixels, 1),
            "Z_metres": None if not verdict.accepted else round(verdict.distance_metres, 3),
            "edge_left_offset_metres": None if not verdict.accepted else round(verdict.left_offset_metres, 3),
            "depth_left_offset_metres": None if depth_offset is None else round(depth_offset, 3),
            "drone_on_course": None if self.watch.drone_on_course() is None else np.round(self.watch.drone_on_course(), 3).tolist(),
        }
        self.log.event("look", f"Look {self.look_number}: {json.dumps(entry)}", **{key: value for key, value in entry.items()
                                                                                   if key != "drone_on_course"})
        self.save_files(entry, last, last_picture)

    def save_files(self, entry, last_look, last_picture):
        if self.records_folder is None:
            return
        try:
            with open(os.path.join(self.records_folder, "look.jsonl"), "a", encoding="utf-8") as lines:
                lines.write(json.dumps(entry) + "\n")
            if last_picture is None:
                return
            with open(os.path.join(self.records_folder, f"look_{self.look_number}.png"), "wb") as picture_file:
                picture_file.write(picture_message_to_png(last_picture[2]))
            with open(os.path.join(self.records_folder, f"look_{self.look_number}_band.png"), "wb") as band_file:
                band_file.write(colour_array_to_png(band_picture_with_edges(last_picture[1], last_look)))
        except (OSError, ValueError) as problem:
            self.log.event("warning", f"Could not save the look's pictures ({problem}). The flight goes on.")
