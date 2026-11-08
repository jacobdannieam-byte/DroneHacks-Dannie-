"""Decides, ten times a second, where the drone should go next and which way it should face,
using the live map. This is the drone's "brain" for getting through the course.

REAL DRONE CODE, also used unchanged in the simulator. It reads nothing and sends nothing
itself: course_run.py gives it the drone's position and heading, and sends what it decides.

How it thinks, in plain words
-----------------------------
The first corridor follows the direction from the launch mark to the checkpoint. It makes small
sideways corrections with the nose facing down the corridor. If depth can't confirm room ahead,
it waits for a clear view and eventually lands. A single blocked spot in run 104453 made the
general planner try to turn almost sideways here, where the drone has little room to slide.

After the first corridor, the navigator replans toward the next checkpoint. If it needs
to turn, it looks for room to hold and rotate. It can first move toward a roomier point
on the visible part of the path. With no usable path it waits for new depth instead of
scanning all the way around. Translation still requires observed clear space.

The goals come from the mission (live_planner.goals_from_mission): one per checkpoint, then the
landing pad. The stage names come from the mission too, not from this code.

Positions here are on the course: (east, north, height) in metres. Headings are in radians,
measured from east toward north, so 0 faces east and pi/2 faces north.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from team_nav.drone_dimensions import DRONE_RADIUS_METRES
from team_nav.live_planner import ROOM_NEEDED_METRES, corridor_exit_reached, find_path, has_reached

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

# Our own choice. Keep sideways corrections small enough to stay in the forward camera's view
# even while the nose is still finishing its turn to face down the first corridor.
CORRIDOR_STEERING_LIMIT_DEGREES = 15.0

# Our own choices for Q1. Keep targets close and require the exit to stay clear over several
# navigation cycles so one bad position report cannot finish the stage.
CORRIDOR_LEAD_METRES = 0.20
CORRIDOR_EXIT_CONFIRM_SECONDS = 0.20

# Our own choice. The drone stops this far before anything its map shows ahead, measured from
# the point where its centre would have too little room (ROOM_NEEDED_METRES).
STOPPING_MARGIN_METRES = 0.15

# Our own choice. The space right around the drone is not checked for "seen empty": the stereo
# camera can't measure nearer than about 0.37 m (stereo_depth.LARGEST_SHIFT_PIXELS).
NOT_CHECKED_NEAR_DRONE_METRES = 0.1

# Our own choice. If the drone has been turning toward one heading for this long without getting
# there, PX4 is not turning it as asked (for example a wrong sign in the heading sent). A half
# turn takes 12 s at live_flight.TURN_RATE_DEGREES_PER_SECOND (15 degrees a second), and the drone
# lags the sent heading by a second or two, so 25 s is far too long.
LONGEST_TURN_SECONDS = 25.0

# Our own choice. A drone that slid during the turn must return this close to its hold point
# before it can move on.
SETTLED_DISTANCE_METRES = 0.10

# Our own choice. Give the drone this long to regain the hold point before ending the run.
LONGEST_SETTLING_SECONDS = 5.0

# Our own choice. The furthest the target is put ahead of the drone. PX4 flies toward a position
# target at about 0.95 m/s per metre of distance (docs/ARCHITECTURE.md section 9, measured in
# the simulator), so 0.35 m gives about 0.33 m/s: slow enough to stop within the margin.
LARGEST_LEAD_METRES = 0.35

# Our own choice. Below this the drone isn't really moving; it holds and re-plans.
SMALLEST_USEFUL_LEAD_METRES = 0.05

# Drone 16 slid about half a metre in run 20261108_050406. Use that observation as the
# minimum allowance for a sharp turn. A slower yaw command alone does not establish less slide.
OBSERVED_TURN_SLIDE_METRES = 0.50
TURN_ROOM_METRES = ROOM_NEEDED_METRES + OBSERVED_TURN_SLIDE_METRES

# Our own choice. Give depth time to clear a temporary obstruction before landing.
LONGEST_SECONDS_WITHOUT_A_PATH = 20.0

APPROACHING_TURN = "moving to a clearer point before turning"
WAITING_FOR_TURN_VIEW = "holding, waiting for depth after turning"
TURNING = "turning to face the path"
FLYING = "flying along the path"
HOLDING = "holding, no path yet"
SETTLING = "holding, moving back to the hold point after a turn"
CORRIDOR_BLOCKED = "holding, first corridor blocked or unseen"
CORRIDOR_CONFIRMING_EXIT = "holding, confirming progress at the first corridor exit"


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

    def __init__(self, live_map, goals, cruise_height_metres, stop_after_stage="", *, full_course_practice=False):
        self.live_map = live_map
        self.goals = list(goals)
        self.cruise_height_metres = cruise_height_metres
        self.stop_after_stage = stop_after_stage
        self.full_course_practice = full_course_practice
        self.goal_number = 0
        self.path = None
        self.last_plan_seconds = -math.inf
        self.seconds_without_path_since = None
        self.turn_heading_radians = None
        self.map_updates_when_turn_settled = None
        self.hold_point = None
        self.previous_drone_point = None
        self.corridor_exit_seen_at_seconds = None
        self.turning_since_seconds = None
        self.settling_since_seconds = None
        self.now_seconds = 0.0

    @property
    def current_goal(self):
        return self.goals[self.goal_number]

    def next_step(self, drone_metres, drone_heading_radians, now_seconds, depth_map_updates=None):
        """What to do this cycle. drone_metres is (east, north, height) on the course."""
        drone_metres = np.asarray(drone_metres, dtype=float)
        self.now_seconds = now_seconds
        if self.turn_heading_radians is not None:
            return self._continue_turn(drone_metres, drone_heading_radians, depth_map_updates)
        step_events = self._check_goal_reached(drone_metres)
        if step_events is not None:
            return step_events
        if self.current_goal.straight_heading_radians is not None:
            return self._follow_straight_corridor(drone_metres, drone_heading_radians)
        self._replan_if_due(drone_metres, now_seconds)
        if self.path is None:
            return self._hold_for_path(drone_metres, drone_heading_radians, "no route to the checkpoint")
        return self._follow_path(drone_metres, drone_heading_radians)

    # ---- Goals and stages ----------------------------------------------------------------------

    def _check_goal_reached(self, drone_metres):
        """If the drone has reached the current goal, moves on to the next. Returns a
        NavigationStep only when that ends the navigation."""
        previous = self.previous_drone_point
        self.previous_drone_point = drone_metres.copy()
        self._pending_events = ([], [])
        corridor_heading = self.current_goal.straight_heading_radians
        if corridor_heading is not None:
            # The sketch's Q1 line is an estimate. During full-course practice, reaching
            # it tells us to look for the next leg. The turn still needs its own clearance.
            practice_uses_centre = self.full_course_practice or self.stop_after_stage == self.current_goal.stage_name
            progress_past_line_metres = 0.0 if practice_uses_centre else DRONE_RADIUS_METRES
            square = self.live_map.square_of(drone_metres)
            if (square is None or self.live_map.mapped_room_metres[square] < ROOM_NEEDED_METRES
                    or not corridor_exit_reached(self.current_goal, drone_metres, progress_past_line_metres)):
                if self.corridor_exit_seen_at_seconds is not None:
                    self.hold_point = None
                self.corridor_exit_seen_at_seconds = None
                return None
            if self.corridor_exit_seen_at_seconds is None:
                self.corridor_exit_seen_at_seconds = self.now_seconds
                self.hold_point = None
            if self.now_seconds - self.corridor_exit_seen_at_seconds < CORRIDOR_EXIT_CONFIRM_SECONDS:
                return self._step(self._hold_target(drone_metres), corridor_heading, CORRIDOR_CONFIRMING_EXIT)
        elif not has_reached(self.current_goal, previous, drone_metres):
            return None
        finished_name = self.current_goal.stage_name
        last_goal = self.goal_number == len(self.goals) - 1
        if last_goal or finished_name == self.stop_after_stage:
            heading = self.current_goal.straight_heading_radians
            if heading is None:
                heading = self._heading_toward_path_or(drone_metres, 0.0)
            return NavigationStep(self._hold_target(drone_metres), heading,
                                  HOLDING, finished=True, stages_finished=[finished_name])
        self.goal_number += 1
        self.path = None
        self.last_plan_seconds = -math.inf
        self.seconds_without_path_since = None
        self._pending_events = ([finished_name], [self.current_goal.stage_name])
        return None

    def _continue_turn(self, drone_metres, drone_heading_radians, depth_map_updates):
        heading = self.turn_heading_radians
        square = self.live_map.square_of(drone_metres)
        if square is None or self._turn_room_metres()[square] < ROOM_NEEDED_METRES:
            raise NoWayForward("The drone no longer has enough clearance at the turn. Landing here.")
        if _angle_between(drone_heading_radians, heading) > math.radians(TURN_DONE_DEGREES):
            self.map_updates_when_turn_settled = None
            return self._step(self._hold_target(drone_metres), heading, TURNING)
        if not self._settled(drone_metres):
            return self._step(self._hold_target(drone_metres), heading, SETTLING)
        if self.map_updates_when_turn_settled is None:
            self.map_updates_when_turn_settled = depth_map_updates
        if depth_map_updates == self.map_updates_when_turn_settled and depth_map_updates is not None:
            return self._hold_for_path(drone_metres, heading, "no new depth after the turn", WAITING_FOR_TURN_VIEW)
        self.turn_heading_radians = None
        self.map_updates_when_turn_settled = None
        self.last_plan_seconds = -math.inf
        return self._step(self._hold_target(drone_metres), heading, WAITING_FOR_TURN_VIEW)

    # ---- Planning --------------------------------------------------------------------------------

    def _replan_if_due(self, drone_metres, now_seconds):
        if now_seconds - self.last_plan_seconds < REPLAN_SECONDS:
            return
        self.last_plan_seconds = now_seconds
        room_metres = self._turn_room_metres()
        self.path = find_path(self.live_map, drone_metres, self.current_goal.point_metres, room_metres)
        if self.path is None and not self.full_course_practice:
            room_without_map_walls = self.live_map.room_metres(trust_map_walls=False)
            self.path = find_path(self.live_map, drone_metres, self.current_goal.point_metres, room_without_map_walls)

    def _hold_for_path(self, drone_metres, heading, reason, activity=HOLDING):
        if self.seconds_without_path_since is None:
            self.seconds_without_path_since = self.now_seconds
        waited = self.now_seconds - self.seconds_without_path_since
        if waited > LONGEST_SECONDS_WITHOUT_A_PATH:
            raise NoWayForward(f"Cannot advance toward '{self.current_goal.stage_name}': {reason}. Landing here.")
        return self._step(self._hold_target(drone_metres), heading, activity)

    def _turn_room_metres(self):
        return self.live_map.room_metres(keep_mapped_walls=self.full_course_practice)

    def _approach_or_turn(self, drone_metres, drone_heading_radians, wanted_heading):
        room_metres = self._turn_room_metres()
        room_metres[self.live_map.near_a_boulder()] = 0.0
        square = self.live_map.square_of(drone_metres)
        if square is not None and room_metres[square] >= TURN_ROOM_METRES:
            self.seconds_without_path_since = None
            self.turn_heading_radians = wanted_heading
            self.hold_point = None
            return self._step(self._hold_target(drone_metres), wanted_heading, TURNING)

        # Only approach a turn along a short, visible section of the route. A clear pocket
        # beyond an obstacle or to the side of the camera cannot justify moving toward it.
        for point in self.path:
            offset_metres = point - drone_metres[:2]
            distance_metres = float(np.linalg.norm(offset_metres))
            if not SMALLEST_USEFUL_LEAD_METRES <= distance_metres <= LARGEST_LEAD_METRES:
                continue
            approach_heading = math.atan2(offset_metres[1], offset_metres[0])
            if _angle_between(approach_heading, drone_heading_radians) > math.radians(TURN_FIRST_DEGREES):
                continue
            if room_metres[self.live_map.square_of(point)] < TURN_ROOM_METRES:
                continue
            direction = offset_metres / distance_metres
            lead_metres = self._safe_lead_metres(drone_metres, direction, distance_metres, room_metres)
            if lead_metres >= distance_metres:
                self.seconds_without_path_since = None
                return self._advance(drone_metres, direction, lead_metres, drone_heading_radians, APPROACHING_TURN)
        return self._hold_for_path(drone_metres, drone_heading_radians, "no clear place to turn")

    # ---- Following the path ---------------------------------------------------------------------

    def _follow_straight_corridor(self, drone_metres, drone_heading_radians):
        heading = self.current_goal.straight_heading_radians
        room_metres = self.live_map.room_metres(keep_mapped_walls=True)
        room_metres[self.live_map.near_a_boulder()] = 0.0
        heading_error = _angle_between(drone_heading_radians, heading)
        if heading_error > math.radians(TURN_FIRST_DEGREES):
            square = self.live_map.square_of(drone_metres)
            if square is None or room_metres[square] < TURN_ROOM_METRES:
                return self._hold_for_path(drone_metres, drone_heading_radians,
                                           "no room to turn toward the first corridor")
        if heading_error > math.radians(TURN_DONE_DEGREES):
            return self._step(self._hold_target(drone_metres), heading, TURNING)
        if not self._settled(drone_metres):
            return self._step(self._hold_target(drone_metres), heading, SETTLING)

        offset_metres = self.current_goal.point_metres - drone_metres[:2]
        forward_metres = offset_metres @ np.array([math.cos(heading), math.sin(heading)])
        bearing = math.atan2(offset_metres[1], offset_metres[0])
        steering_limit = math.radians(CORRIDOR_STEERING_LIMIT_DEGREES)
        steering = math.remainder(bearing - heading, math.tau)
        travel_heading = heading + max(-steering_limit, min(steering_limit, steering))
        direction = np.array([math.cos(travel_heading), math.sin(travel_heading)])
        lead_metres = self._safe_lead_metres(drone_metres, direction,
                                            min(forward_metres, CORRIDOR_LEAD_METRES), room_metres)
        if lead_metres < SMALLEST_USEFUL_LEAD_METRES:
            if self.seconds_without_path_since is None:
                self.seconds_without_path_since = self.now_seconds
            if self.now_seconds - self.seconds_without_path_since > LONGEST_SECONDS_WITHOUT_A_PATH:
                raise NoWayForward("The map and depth could not confirm enough room ahead in the first corridor "
                                   f"for {LONGEST_SECONDS_WITHOUT_A_PATH:.0f} seconds. Landing here.")
            return self._step(self._hold_target(drone_metres), heading, CORRIDOR_BLOCKED)

        self.seconds_without_path_since = None
        return self._advance(drone_metres, direction, lead_metres, heading)

    def _follow_path(self, drone_metres, drone_heading_radians):
        aim_point = _point_ahead_on_path(self.path, drone_metres, LOOK_AHEAD_METRES)
        offset = aim_point - drone_metres[:2]
        distance_to_aim = float(np.linalg.norm(offset))
        if distance_to_aim < 1e-3:
            return self._hold_for_path(drone_metres, drone_heading_radians, "the path ends before the checkpoint")
        wanted_heading = math.atan2(offset[1], offset[0])
        if _angle_between(drone_heading_radians, wanted_heading) > math.radians(TURN_FIRST_DEGREES):
            return self._approach_or_turn(drone_metres, drone_heading_radians, wanted_heading)
        if not self._settled(drone_metres):
            return self._step(self._hold_target(drone_metres), wanted_heading, SETTLING)
        direction = offset / distance_to_aim
        lead = self._safe_lead_metres(drone_metres, direction, distance_to_aim)
        if lead < SMALLEST_USEFUL_LEAD_METRES:
            # Something is just ahead that the plan didn't know about, or the camera hasn't seen
            # the space ahead as empty yet. Hold, look, and plan again at once.
            self.last_plan_seconds = -math.inf
            return self._hold_for_path(drone_metres, drone_heading_radians, "the route ahead is blocked or unseen")
        self.seconds_without_path_since = None
        return self._advance(drone_metres, direction, lead, wanted_heading)

    def _safe_lead_metres(self, drone_metres, direction, distance_to_aim_metres, room_metres=None):
        if room_metres is None and self.full_course_practice:
            room_metres = self._turn_room_metres()
        free_metres = self.live_map.free_distance_metres(drone_metres, direction, ROOM_NEEDED_METRES, 2.0, room_metres)
        seen_metres = self.live_map.seen_distance_metres(drone_metres, direction, 2.0, NOT_CHECKED_NEAR_DRONE_METRES)
        return min(LARGEST_LEAD_METRES, free_metres - STOPPING_MARGIN_METRES, seen_metres - ROOM_NEEDED_METRES,
                   distance_to_aim_metres)

    def _advance(self, drone_metres, direction, lead_metres, wanted_heading, activity=FLYING):
        self.hold_point = None
        target = np.append(drone_metres[:2] + direction * lead_metres, self.cruise_height_metres)
        return self._step(target, wanted_heading, activity)

    # ---- Small helpers ---------------------------------------------------------------------------

    def _settled(self, drone_metres):
        """Wait for the drone to regain the hold point before starting forward again."""
        if self.hold_point is None:
            self.settling_since_seconds = None
            return True
        if np.linalg.norm(drone_metres[:2] - self.hold_point[:2]) <= SETTLED_DISTANCE_METRES:
            self.settling_since_seconds = None
            return True
        if self.settling_since_seconds is None:
            self.settling_since_seconds = self.now_seconds
        if self.now_seconds - self.settling_since_seconds >= LONGEST_SETTLING_SECONDS:
            raise NoWayForward("The drone could not return to its hold point after turning. Landing here.")
        return False

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
        if activity != TURNING:
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
