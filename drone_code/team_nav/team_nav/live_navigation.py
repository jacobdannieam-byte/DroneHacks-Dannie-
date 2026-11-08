"""Decides, ten times a second, where the drone should go next and which way it should face,
using the live map. This is the drone's "brain" for getting through the course.

REAL DRONE CODE, also used unchanged in the simulator. It reads nothing and sends nothing
itself: course_run.py gives it the drone's position and heading, and sends what it decides.

How it thinks, in plain words
-----------------------------
The drone can only see forward, so it follows four rules:
  1. Look before you go. At the start, it turns a full circle in quarter turns and waits at each
     quarter, so the depth camera can fill in the live map all around it. It does the same when
     it can't find a way forward.
  2. Face where you fly. Before moving, it turns its nose (and so its camera) toward the next
     part of the path. If the path bends by more than TURN_FIRST_DEGREES, it stops and turns on
     the spot first, so it never moves sideways into space it hasn't looked at.
  3. Only move into space you have seen. It moves forward at most as far as the camera has seen
     empty space ahead (less the drone's radius), and never closer than a stopping margin to
     anything blocked. A plain wall with no pattern gives the camera no distance at all, so it
     never counts as "seen empty": the drone stops in front of it instead of flying into it.
  4. Re-plan all the time. Every REPLAN_SECONDS it plans a fresh path to the current goal from
     where it is, on the newest map. A moved wall just gives a different path. If the map file's
     walls block every path, it plans again treating the walls the camera hasn't seen as unknown,
     and goes to look (the map file may be out of date: walls moved on 2026-11-08).

The goals come from the mission (live_planner.goals_from_mission): one per checkpoint, then the
landing pad. The stage names come from the mission too, not from this code.

Positions here are on the course: (east, north, height) in metres. Headings are in radians,
measured from east toward north, so 0 faces east and pi/2 faces north.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from team_nav.live_planner import ROOM_NEEDED_METRES, find_path, has_reached

# Our own choice. How often a fresh path is planned. A search takes about 50 ms on the drone.
REPLAN_SECONDS = 0.5

# Our own choice. The drone steers toward the point this far along the path. Shorter cuts
# corners less; longer gives smoother flight.
LOOK_AHEAD_METRES = 0.5

# Our own choice. If the drone's nose points further than this from where the path goes, it
# stops and turns on the spot first. The camera sees about 78 degrees edge to edge
# (stereo_depth.angle_of_view_degrees(1024, 634.5)), so 30 degrees keeps the path in view.
TURN_FIRST_DEGREES = 30.0

# Our own choice. A turn is finished when the nose is within this of the wanted heading.
TURN_DONE_DEGREES = 10.0

# Our own choice. The drone stops this far before anything its map shows ahead, measured from
# the point where its centre would have too little room (ROOM_NEEDED_METRES).
STOPPING_MARGIN_METRES = 0.15

# Our own choice. The space right around the drone is not checked for "seen empty": the stereo
# camera can't measure nearer than about 0.37 m (stereo_depth.LARGEST_SHIFT_PIXELS).
NOT_CHECKED_NEAR_DRONE_METRES = 0.1

# Our own choice. If the drone has been turning toward one heading for this long without getting
# there, PX4 is not turning it as asked (for example a wrong sign in the heading sent). A quarter
# turn takes 3 s at live_flight.TURN_RATE_DEGREES_PER_SECOND, so 12 s is far too long.
LONGEST_TURN_SECONDS = 12.0

# Our own choice. The furthest the target is put ahead of the drone. PX4 flies toward a position
# target at about 0.95 m/s per metre of distance (docs/ARCHITECTURE.md section 9, measured in
# the simulator), so 0.35 m gives about 0.33 m/s: slow enough to stop within the margin.
LARGEST_LEAD_METRES = 0.35

# Our own choice. Below this the drone isn't really moving; it holds and re-plans.
SMALLEST_USEFUL_LEAD_METRES = 0.05

# Our own choice. During a look-around the drone waits this long at each quarter turn, so the
# depth camera (10 pictures a second) sees that direction several times.
LOOK_PAUSE_SECONDS = 1.0

# Our own choice. A full look-around is not repeated more often than this.
LEAST_SECONDS_BETWEEN_LOOK_AROUNDS = 10.0

# Our own choice. If no path has been found for this long, even after looking around, the run
# stops and the drone lands.
LONGEST_SECONDS_WITHOUT_A_PATH = 20.0

LOOKING_AROUND = "looking around"
LOOKING_STILL = "looking around, holding still to look"
TURNING = "turning to face the path"
FLYING = "flying along the path"
HOLDING = "holding, no path yet"


class NoWayForward(Exception):
    """No safe path to the goal for too long. The message says why, in a full sentence."""


@dataclass
class NavigationStep:
    target_metres: np.ndarray           # (east, north, height) to send.
    heading_radians: float              # Which way the nose should face.
    activity: str                       # One of the activity names above.
    finished: bool = False              # True once the last goal (the landing pad) is reached.
    stages_finished: list = field(default_factory=list)   # Names of stages finished this cycle.
    stages_begun: list = field(default_factory=list)


class LiveNavigator:

    def __init__(self, live_map, goals, cruise_height_metres, stop_after_stage=""):
        self.live_map = live_map
        self.goals = list(goals)
        self.cruise_height_metres = cruise_height_metres
        self.stop_after_stage = stop_after_stage
        self.goal_number = 0
        self.path = None
        self.last_plan_seconds = -math.inf
        self.seconds_without_path_since = None
        self.last_look_around_seconds = -math.inf
        self.look_headings = []
        self.look_pause_began_seconds = None
        self.hold_point = None
        self.previous_drone_point = None
        self.turning_since_seconds = None
        self.now_seconds = 0.0

    @property
    def current_goal(self):
        return self.goals[self.goal_number]

    def next_step(self, drone_metres, drone_heading_radians, now_seconds):
        """What to do this cycle. drone_metres is (east, north, height) on the course."""
        drone_metres = np.asarray(drone_metres, dtype=float)
        self.now_seconds = now_seconds
        step_events = self._check_goal_reached(drone_metres)
        if step_events is not None:
            return step_events
        if self.look_headings:
            return self._look_around(drone_metres, drone_heading_radians, now_seconds)
        self._replan_if_due(drone_metres, now_seconds)
        if self.path is None:
            return self._no_path(drone_metres, drone_heading_radians, now_seconds)
        return self._follow_path(drone_metres, drone_heading_radians)

    def start_look_around(self, drone_metres, drone_heading_radians, now_seconds):
        """Four quarter turns, starting with the one to the left, ending facing the start."""
        self.look_headings = [drone_heading_radians + quarter * math.pi / 2.0 for quarter in (1, 2, 3, 4)]
        self.look_pause_began_seconds = None
        self.last_look_around_seconds = now_seconds
        self.hold_point = np.asarray(drone_metres, dtype=float).copy()

    # ---- Goals and stages ----------------------------------------------------------------------

    def _check_goal_reached(self, drone_metres):
        """If the drone has reached the current goal, moves on to the next. Returns a
        NavigationStep only when that ends the navigation."""
        previous = self.previous_drone_point
        self.previous_drone_point = drone_metres.copy()
        if not has_reached(self.current_goal, previous, drone_metres):
            self._pending_events = ([], [])
            return None
        finished_name = self.current_goal.stage_name
        last_goal = self.goal_number == len(self.goals) - 1
        if last_goal or finished_name == self.stop_after_stage:
            return NavigationStep(self._hold_target(drone_metres), self._heading_toward_path_or(drone_metres, 0.0),
                                  HOLDING, finished=True, stages_finished=[finished_name])
        self.goal_number += 1
        self.path = None
        self.last_plan_seconds = -math.inf
        self._pending_events = ([finished_name], [self.current_goal.stage_name])
        return None

    # ---- Looking around ------------------------------------------------------------------------

    def _look_around(self, drone_metres, drone_heading_radians, now_seconds):
        wanted = self.look_headings[0]
        if _angle_between(drone_heading_radians, wanted) > math.radians(TURN_DONE_DEGREES):
            self.look_pause_began_seconds = None
            return self._step(self._hold_target(drone_metres), wanted, LOOKING_AROUND)
        if self.look_pause_began_seconds is None:
            self.look_pause_began_seconds = now_seconds
        if now_seconds - self.look_pause_began_seconds >= LOOK_PAUSE_SECONDS:
            self.look_headings.pop(0)
            self.look_pause_began_seconds = None
            self.last_plan_seconds = -math.inf
        return self._step(self._hold_target(drone_metres), wanted, LOOKING_STILL)

    # ---- Planning --------------------------------------------------------------------------------

    def _replan_if_due(self, drone_metres, now_seconds):
        if now_seconds - self.last_plan_seconds < REPLAN_SECONDS:
            return
        self.last_plan_seconds = now_seconds
        self.path = find_path(self.live_map, drone_metres, self.current_goal.point_metres)
        if self.path is None:
            room_without_map_walls = self.live_map.room_metres(trust_map_walls=False)
            self.path = find_path(self.live_map, drone_metres, self.current_goal.point_metres, room_without_map_walls)

    def _no_path(self, drone_metres, drone_heading_radians, now_seconds):
        if self.seconds_without_path_since is None:
            self.seconds_without_path_since = now_seconds
        waited = now_seconds - self.seconds_without_path_since
        if waited > LONGEST_SECONDS_WITHOUT_A_PATH:
            raise NoWayForward(
                f"No safe path to '{self.current_goal.stage_name}' for {waited:.0f} seconds, even after looking "
                f"around. The live map shows too little room everywhere toward the goal: either the way is really "
                f"blocked, or the depth camera is seeing obstacles that aren't there.")
        if now_seconds - self.last_look_around_seconds > LEAST_SECONDS_BETWEEN_LOOK_AROUNDS:
            self.start_look_around(drone_metres, drone_heading_radians, now_seconds)
            return self._look_around(drone_metres, drone_heading_radians, now_seconds)
        return self._step(self._hold_target(drone_metres), drone_heading_radians, HOLDING)

    # ---- Following the path ---------------------------------------------------------------------

    def _follow_path(self, drone_metres, drone_heading_radians):
        self.seconds_without_path_since = None
        aim_point = _point_ahead_on_path(self.path, drone_metres, LOOK_AHEAD_METRES)
        offset = aim_point - drone_metres[:2]
        distance_to_aim = float(np.linalg.norm(offset))
        if distance_to_aim < 1e-3:
            return self._step(self._hold_target(drone_metres), drone_heading_radians, FLYING)
        wanted_heading = math.atan2(offset[1], offset[0])
        if _angle_between(drone_heading_radians, wanted_heading) > math.radians(TURN_FIRST_DEGREES):
            return self._step(self._hold_target(drone_metres), wanted_heading, TURNING)
        direction = offset / distance_to_aim
        free_metres = self.live_map.free_distance_metres(drone_metres, direction, ROOM_NEEDED_METRES, 2.0)
        seen_metres = self.live_map.seen_distance_metres(drone_metres, direction, 2.0, NOT_CHECKED_NEAR_DRONE_METRES)
        lead = min(LARGEST_LEAD_METRES, free_metres - STOPPING_MARGIN_METRES, seen_metres - ROOM_NEEDED_METRES,
                   distance_to_aim)
        if lead < SMALLEST_USEFUL_LEAD_METRES:
            # Something is just ahead that the plan didn't know about, or the camera hasn't seen
            # the space ahead as empty yet. Hold, look, and plan again at once.
            self.last_plan_seconds = -math.inf
            return self._step(self._hold_target(drone_metres), wanted_heading, HOLDING)
        self.hold_point = None
        target = np.array([drone_metres[0] + direction[0] * lead, drone_metres[1] + direction[1] * lead,
                           self.cruise_height_metres])
        return self._step(target, wanted_heading, FLYING)

    # ---- Small helpers ---------------------------------------------------------------------------

    def _hold_target(self, drone_metres):
        """Where to stay while turning or holding: the point where holding began, so the drone
        doesn't creep."""
        if self.hold_point is None:
            self.hold_point = np.asarray(drone_metres, dtype=float).copy()
        return np.array([self.hold_point[0], self.hold_point[1], self.cruise_height_metres])

    def _heading_toward_path_or(self, drone_metres, fallback_radians):
        if not self.path:
            return fallback_radians
        aim = _point_ahead_on_path(self.path, drone_metres, LOOK_AHEAD_METRES)
        return math.atan2(aim[1] - drone_metres[1], aim[0] - drone_metres[0])

    def _check_turn_progress(self, activity, wanted_heading, now_seconds):
        """Raises NoWayForward if the drone has been turning for LONGEST_TURN_SECONDS without
        reaching the heading."""
        if activity not in (TURNING, LOOKING_AROUND):
            self.turning_since_seconds = None
            return
        if self.turning_since_seconds is None:
            self.turning_since_seconds = now_seconds
        if now_seconds - self.turning_since_seconds > LONGEST_TURN_SECONDS:
            raise NoWayForward(
                f"The drone was asked to turn to {math.degrees(wanted_heading):.0f} degrees but has not got there in "
                f"{LONGEST_TURN_SECONDS:.0f} seconds. PX4 is not turning it as asked; check the heading sent in "
                f"target_sender.py against flight_interface.")

    def _step(self, target, heading, activity):
        self._check_turn_progress(activity, heading, self.now_seconds)
        finished, begun = getattr(self, "_pending_events", ([], []))
        self._pending_events = ([], [])
        return NavigationStep(target, heading, activity, stages_finished=list(finished), stages_begun=list(begun))


def _point_ahead_on_path(path, drone_metres, look_ahead_metres):
    """The first path point at least look_ahead_metres from the drone, after the path point
    nearest the drone. The last point if none is that far."""
    points = np.asarray(path)
    distances = np.linalg.norm(points - drone_metres[:2], axis=1)
    nearest = int(np.argmin(distances))
    for index in range(nearest, len(points)):
        if distances[index] >= look_ahead_metres:
            return points[index]
    return points[-1]


def _angle_between(first_radians, second_radians):
    """The size of the turn from one heading to the other, 0 to pi."""
    return abs(math.atan2(math.sin(second_radians - first_radians), math.cos(second_radians - first_radians)))
