"""PRETEND / SIMULATOR ONLY. Never runs on the drone.

The bench's referee: watches the TRUE drone in the TRUE course (pretend_world.py) and decides
checkpoints, touches, the finish, the 3-minute limit, a crash, and when the organizers would cut
the motors. It knows nothing the flight program knows, and the program never sees it.

How it fits in
--------------
bench_link.py calls update() about 20 times a simulated second with the true position. When the
run is over, update() returns a sentence saying why, and bench_link.py stops the trial.
result() gives the numbers for the results table.

The rules (from the rulebook, as summarized in understand.json "rules-and-course")
---------------------------------------------------------------------------------
  - Five checkpoints, in order, each cleared once the WHOLE drone is past its line. The fourth is
    Gate B or the window; the fifth is landing and disarming with the whole drone in the pad's
    outer ring. Points 4 / 7 / 10 / 7 / 7.
  - A touch of a wall, the tunnel, a boulder or the net adds 5 s.
  - The run ends on: disarm; touching the ground off the pad; a crash; 180 s after leaving the
    launch mark; the organizers cutting the motors.
  - Finish score: 35 + 20 x (fastest / ours) + 5 through the window. We assume a fastest finish
    of 60 s (the organizers' own example). An unfinished run here only counts its checkpoint
    points; the speed part needs the other teams' times.

What it does not do: judge "flying over a wall" (rule 7.7), or the return bonus.
"""
import numpy as np

# From the rulebook: checkpoint points in order, and which line names count for each.
CHECKPOINT_POINTS = [4, 7, 10, 7]
CHECKPOINT_LINES = [["end of first corridor"], ["Gate A"], ["tunnel exit"], ["Gate B", "through the window"]]
LANDING_POINTS = 7
WINDOW_BONUS_POINTS = 5
FINISH_BASE_POINTS = 35
FINISH_SPEED_POINTS = 20
# From the rulebook, organizers' example.
ASSUMED_FASTEST_FINISH_SECONDS = 60.0
RUN_LIMIT_SECONDS = 180.0
TOUCH_PENALTY_SECONDS = 5.0

# Estimated (ARCHITECTURE.md): the drone's radius to the propeller tips, and its half height.
DRONE_RADIUS_METRES = 0.30
DRONE_HALF_HEIGHT_METRES = 0.10
# Our own choice: a propeller reaching this far into a solid thing breaks the flight (a crash).
CRASH_DEPTH_METRES = 0.10
# Our own choice: a crash while coming down this fast is a landing onto something, not a flight crash.
DESCENDING_FASTER_THAN_METRES_PER_SECOND = 0.1
# Our own choice: a touch that ends and starts again within this time is the same touch.
SAME_TOUCH_SECONDS = 1.0
# Our own choice: the drone counts as having left the launch mark (the clock starts) above this.
LIFTOFF_HEIGHT_METRES = 0.05
# Our own choice, from what the organizers did on 2026-11-08 (two motor cuts after 23-35 s of
# turning in place with under 1 m of progress): cut the motors when the drone has not moved
# NO_PROGRESS_METRES from a spot for NO_PROGRESS_SECONDS.
NO_PROGRESS_METRES = 0.3
NO_PROGRESS_SECONDS = 30.0


class BenchReferee:

    def __init__(self, world):
        self.world = world
        self.liftoff_seconds = None
        self.checkpoint_seconds = []          # Time each checkpoint was cleared, from liftoff.
        self.checkpoint_line_used = []
        # For diagnosis only, not for scoring: every checkpoint line the whole drone got past, in
        # any order. It shows "flew the course but checkpoint 1 never counted".
        self.lines_passed_in_any_order = set()
        self.touches = 0
        self.touching = False
        self.last_touching_seconds = -1e9
        self.closest_approach_metres = np.inf
        self.closest_approach_name = ""
        self.progress_anchor = None
        self.progress_anchor_seconds = None
        self.finished = False
        self.end_reason = None
        self.end_position = None
        self.end_seconds = None

    def update(self, now_seconds, position_on_course, armed, upward_speed_metres_per_second=0.0):
        """Returns None while the run goes on, or a sentence saying why it ended."""
        if self.end_reason is not None:
            return self.end_reason
        height = position_on_course[2]
        if self.liftoff_seconds is None:
            if height > LIFTOFF_HEIGHT_METRES:
                self.liftoff_seconds = now_seconds
                self.progress_anchor, self.progress_anchor_seconds = position_on_course.copy(), now_seconds
            return None
        flight_seconds = now_seconds - self.liftoff_seconds
        self._check_checkpoints(position_on_course, flight_seconds)
        reason = (self._check_touch(position_on_course, now_seconds, upward_speed_metres_per_second)
                  or self._check_ground(position_on_course, armed, flight_seconds)
                  or self._check_progress(position_on_course, now_seconds)
                  or self._check_time(flight_seconds))
        if reason is not None:
            self.end_reason = reason
            self.end_position = position_on_course.copy()
            self.end_seconds = flight_seconds
        return reason

    # ---- The rules ---------------------------------------------------------------------------------

    def _check_checkpoints(self, position, flight_seconds):
        for line_names in CHECKPOINT_LINES:
            for line_name in line_names:
                if self._whole_drone_is_past(self.world.checkpoint_lines[line_name], position):
                    self.lines_passed_in_any_order.add(line_name)
        number = len(self.checkpoint_seconds)
        if number >= len(CHECKPOINT_LINES):
            return
        for line_name in CHECKPOINT_LINES[number]:
            if self._whole_drone_is_past(self.world.checkpoint_lines[line_name], position):
                self.checkpoint_seconds.append(flight_seconds + self.touches * TOUCH_PENALTY_SECONDS)
                self.checkpoint_line_used.append(line_name)
                return

    def _whole_drone_is_past(self, line, position):
        """True if the drone's centre is at least its radius past the line, on the line's far side,
        and level with the line (not off its ends)."""
        start = np.asarray(line["start_metres"], dtype=float)
        end = np.asarray(line["end_metres"], dtype=float)
        along = (end - start) / np.linalg.norm(end - start)
        across = np.array([-along[1], along[0]])
        far_side = np.asarray(line["far_side_point_metres"], dtype=float) - start
        if far_side @ across < 0.0:
            across = -across
        offset = position[:2] - start
        along_metres = offset @ along
        level_with_line = 0.0 <= along_metres <= np.linalg.norm(end - start)
        return level_with_line and offset @ across >= DRONE_RADIUS_METRES

    def _check_touch(self, position, now_seconds, upward_speed_metres_per_second):
        if position[2] < DRONE_HALF_HEIGHT_METRES:
            return None     # On or near the floor: the referee only counts touches in flight.
        distance, name = self.world.nearest_solid_distance_metres(position, DRONE_HALF_HEIGHT_METRES)
        if distance < self.closest_approach_metres:
            self.closest_approach_metres, self.closest_approach_name = distance, name
        touching_now = distance < DRONE_RADIUS_METRES
        if touching_now and not self.touching and now_seconds - self.last_touching_seconds > SAME_TOUCH_SECONDS:
            self.touches += 1
        if touching_now:
            self.last_touching_seconds = now_seconds
        self.touching = touching_now
        if distance < DRONE_RADIUS_METRES - CRASH_DEPTH_METRES:
            if upward_speed_metres_per_second < -DESCENDING_FASTER_THAN_METRES_PER_SECOND:
                # The program chose to land here and came down onto something low (a boulder).
                return f"landed onto the {name}"
            return f"crash: propellers hit the {name}"
        return None

    def _check_ground(self, position, armed, flight_seconds):
        if armed or position[2] > LIFTOFF_HEIGHT_METRES:
            return None
        on_pad = (np.linalg.norm(position[:2] - self.world.landing_pad_centre)
                  <= self.world.landing_pad_radius_metres - DRONE_RADIUS_METRES)
        if on_pad and len(self.checkpoint_seconds) == len(CHECKPOINT_LINES):
            self.finished = True
            return "finished: landed and disarmed on the pad"
        if on_pad:
            return "landed on the pad with a checkpoint missing"
        return "landed off the pad"

    def _check_progress(self, position, now_seconds):
        if np.linalg.norm(position[:2] - self.progress_anchor[:2]) >= NO_PROGRESS_METRES:
            self.progress_anchor, self.progress_anchor_seconds = position.copy(), now_seconds
            return None
        if now_seconds - self.progress_anchor_seconds >= NO_PROGRESS_SECONDS:
            return f"organizers cut the motors: under {NO_PROGRESS_METRES} m of progress in {NO_PROGRESS_SECONDS:.0f} s"
        return None

    def _check_time(self, flight_seconds):
        if flight_seconds >= RUN_LIMIT_SECONDS:
            return f"the {RUN_LIMIT_SECONDS:.0f} s limit"
        return None

    # ---- The result --------------------------------------------------------------------------------

    def score(self):
        points = sum(CHECKPOINT_POINTS[:len(self.checkpoint_seconds)])
        if not self.finished:
            return float(points)
        finish_seconds = self.end_seconds + self.touches * TOUCH_PENALTY_SECONDS
        window = WINDOW_BONUS_POINTS if "through the window" in self.checkpoint_line_used else 0
        return FINISH_BASE_POINTS + FINISH_SPEED_POINTS * ASSUMED_FASTEST_FINISH_SECONDS / finish_seconds + window

    def result(self):
        return {
            "finished": self.finished,
            "checkpoints": len(self.checkpoint_seconds) + (1 if self.finished else 0),
            "lines_passed_any_order": len(self.lines_passed_in_any_order),
            "touches": self.touches,
            "closest_approach_metres": round(float(self.closest_approach_metres), 3),
            "closest_approach_to": self.closest_approach_name,
            "flight_seconds": None if self.end_seconds is None else round(self.end_seconds, 1),
            "score": round(self.score(), 1),
            "referee_end": self.end_reason,
            "end_east_metres": None if self.end_position is None else round(float(self.end_position[0]), 2),
            "end_north_metres": None if self.end_position is None else round(float(self.end_position[1]), 2),
            "checkpoint_seconds": [round(seconds, 1) for seconds in self.checkpoint_seconds],
        }
