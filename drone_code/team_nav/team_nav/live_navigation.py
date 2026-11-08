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
The first checkpoint counts once the whole drone is past its line. On the sketch map of
2026-11-08 the top wall is only 0.58 m beyond the line, less than the drone's 0.60 m width, so
the map alone never allows it. The map is only a sketch, so once the drone's centre is near or
past the line (in competition), the wall ahead is judged by the live map, where depth can show
that the real wall is further away, and the drone creeps on until only CREEP_GAP_TO_WALL_METRES
is left between its body and that wall. If even then the whole drone is not past the line, it
counts once the centre is past the line (or at most LARGEST_SHORTFALL_AT_LINE_METRES short of
it) and the drone can go no deeper. A loud warning then says the referee may not count it.

After the first corridor, the navigator replans toward the next checkpoint. If it needs to
turn, it turns calmly: it stops first, and starts the turn only when the drone is nearly still,
the same new direction has come out of planning for TURN_AGREEMENT_SECONDS (so at least two
plans in a row agree), and there is room to turn. It can first move toward a roomier point on
the visible part of the path. With no usable path it waits for new depth instead of scanning
all the way around. Translation still requires observed clear space.

full_course_practice (practice mode, whole course) differs from competition in two ways. It
keeps the map's walls even where the camera sees through them (when turning, when choosing how
far to move, and when planning, where it also never retries without the map's walls). And it
counts the first checkpoint as soon as the drone's centre is past the line. Competition trusts
the live map, which can clear a wall the map file put in the wrong place.

The goals come from the mission (live_planner.goals_from_mission): one per checkpoint, then the
landing pad. The stage names come from the mission too, not from this code.

Positions here are on the course: (east, north, height) in metres. Headings are in radians,
measured from east toward north, so 0 faces east and pi/2 faces north.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from team_nav.drone_dimensions import DRONE_RADIUS_METRES
from team_nav.live_planner import ROOM_NEEDED_METRES, corridor_exit_reached, find_path, has_reached, metres_past_line

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

# Our own choices. A turn on the spot starts only when the wanted direction has stayed within
# TURN_AGREEMENT_DEGREES of the first one asked for, for TURN_AGREEMENT_SECONDS: with a new plan
# every REPLAN_SECONDS, at least two plans in a row must agree. In the 10:44 run of 2026-11-08 one
# plan flipped the wanted heading 60 degrees the wrong way, and the drone turned for nothing.
TURN_AGREEMENT_DEGREES = 20.0
TURN_AGREEMENT_SECONDS = 0.5

# Our own choice, from measurements on drone 16 on 2026-11-08: a turn started while the drone was
# still moving made it slide 0.28 m, a turn from rest only 0.08 to 0.12 m. So a turn starts only
# below this speed. The speed is measured over SPEED_MEASURING_SECONDS: the hover wander of 1 to
# 3 cm (measured) then reads under 0.06 m/s, below this limit.
LARGEST_SPEED_TO_START_A_TURN_METRES_PER_SECOND = 0.08
SPEED_MEASURING_SECONDS = 0.5

# Our own choice. Keep sideways corrections small enough to stay in the forward camera's view
# even while the nose is still finishing its turn to face down the first corridor.
CORRIDOR_STEERING_LIMIT_DEGREES = 15.0

# Our own choices for Q1. Keep targets close and require the exit to stay clear over several
# navigation cycles so one bad position report cannot finish the stage.
CORRIDOR_LEAD_METRES = 0.20
CORRIDOR_EXIT_CONFIRM_SECONDS = 0.20

# Our own choice. Counting the first checkpoint WITHOUT the whole drone past the line ("it can go
# no deeper") must hold this long, because a count can't be undone. In the noise bench a single
# noisy depth point just ahead stopped the creep for 0.2 s and the checkpoint counted 0.18 to
# 0.27 m past the line instead of 0.29 m or more (seeds 9, 11 and 12, 2026-11-08). About ten
# depth pictures arrive in this time, and their rays clear such a point.
CREEP_END_CONFIRM_SECONDS = 1.0

# Our own choice. The first checkpoint's line is our own reading of the rulebook, placed on a
# sketch map. When the drone can go no deeper, its centre may be up to this far short of the line
# and still count (with the loud warning), and from there the live map judges the wall ahead.
# Without it, a drone stopped 2 cm short of the line by the sketch's wall waited until the
# no-progress rule landed it (code review, 2026-11-08).
LARGEST_SHORTFALL_AT_LINE_METRES = 0.10

# Our own choice. The drone stops this far before anything its map shows ahead, measured from
# the point where its centre would have too little room (ROOM_NEEDED_METRES).
STOPPING_MARGIN_METRES = 0.15
# The team's choice on 2026-11-08. Past the first checkpoint's line (in competition) the drone
# creeps toward the wall ahead until only this gap is left between its body and the wall the
# live map shows, with no stopping margin. It moves slowly there, because each target is only a
# few centimetres ahead; PX4 overshoots a position target by at most 4 cm (measured on drone 16).
# On the sketch map the wall is 0.58 m past the line, and the whole drone past the line needs
# 0.30 m plus its 0.30 m radius, so it needs the real wall at least 0.30 + 0.30 + 0.03 = 0.63 m
# past the line, plus up to one 5 cm live-map square, because the map only knows which square
# the wall's face is in. The normal clearance (ROOM_NEEDED_METRES plus STOPPING_MARGIN_METRES) would
# need 0.80 m.
CREEP_GAP_TO_WALL_METRES = 0.03
CREEP_ROOM_NEEDED_METRES = DRONE_RADIUS_METRES + CREEP_GAP_TO_WALL_METRES

# Our own choice. A drone too close to a wall to turn (for example after creeping deep past the
# first line) backs straight out, the way it came, in steps this long until it has turn room.
# The way it came was seen empty on the way in, and moving straight back needs no turn.
BACK_AWAY_STEP_METRES = 0.10

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

# Our own choice. The first corridor moves on with a lead as small as this. It is less than the
# half square (0.025 m) in which free and seen distances are measured, so the creep past the
# first line can use the last half square. With SMALLEST_USEFUL_LEAD_METRES it stopped 0.13 and
# 0.15 m past the line in the 3D rendered sim (runs 20261108_224004 and _224239), where the live
# map allowed about 0.22 m.
CORRIDOR_SMALLEST_LEAD_METRES = 0.01

# Measured on drone 16 on 2026-11-08 (10:23 and 10:44 runs): a turn of about 80 degrees from rest,
# at live_flight.TURN_RATE_DEGREES_PER_SECOND, made the drone slide 0.08 to 0.12 m. Rounded up to
# 0.15 m for a quarter turn. Turns now start only from rest
# (LARGEST_SPEED_TO_START_A_TURN_METRES_PER_SECOND). waypoint_route.py uses TURN_ROOM_METRES too.
# It was 0.50 m, from an older run (20261108_050406) that turned at 30 degrees a second; with that,
# the drone could not turn at the end of the first corridor on the sketch map (0.55 m of room).
OBSERVED_TURN_SLIDE_METRES = 0.15
TURN_ROOM_METRES = ROOM_NEEDED_METRES + OBSERVED_TURN_SLIDE_METRES

# Our own choice. Give depth time to clear a temporary obstruction before landing.
LONGEST_SECONDS_WITHOUT_A_PATH = 20.0

APPROACHING_TURN = "moving to a clearer point before turning"
STOPPING_TO_TURN = "holding, waiting to be still and sure of the new direction before turning"
WAITING_FOR_TURN_VIEW = "holding, waiting for depth after turning"
TURNING = "turning to face the path"
FLYING = "flying along the path"
HOLDING = "holding, no path yet"
SETTLING = "holding, moving back to the hold point after a turn"
CORRIDOR_BLOCKED = "holding, first corridor blocked or unseen"
CORRIDOR_CONFIRMING_EXIT = "holding, confirming progress at the first corridor exit"

# The activities in which the drone moves. live_flight.py turns the nose more slowly in these.
MOVING_ACTIVITIES = (FLYING, APPROACHING_TURN)


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
    warning: str = ""                   # A warning to write in the records, or "".


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
        # The turn first asked for, and since when (see TURN_AGREEMENT_SECONDS).
        self.proposed_turn_heading_radians = None
        self.proposed_turn_since_seconds = None
        # The drone's speed, measured from its positions SPEED_MEASURING_SECONDS apart. Unknown
        # (infinite) until the first measurement, so no turn starts before it.
        self.speed_metres_per_second = math.inf
        self.speed_measured_from = None         # (position, seconds) the next measurement starts from.
        self._pending_warning = ""               # Given with the next step, then cleared.
        self.corridor_exit_warning = ""          # Given when the first checkpoint counts (see below).

    @property
    def current_goal(self):
        return self.goals[self.goal_number]

    def next_step(self, drone_metres, drone_heading_radians, now_seconds, depth_map_updates=None):
        """What to do this cycle. drone_metres is (east, north, height) on the course."""
        drone_metres = np.asarray(drone_metres, dtype=float)
        self.now_seconds = now_seconds
        self._measure_speed(drone_metres)
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
            if not self._corridor_exit_counts(drone_metres):
                if self.corridor_exit_seen_at_seconds is not None:
                    self.hold_point = None
                self.corridor_exit_seen_at_seconds = None
                return None
            if self.corridor_exit_seen_at_seconds is None:
                self.corridor_exit_seen_at_seconds = self.now_seconds
                self.hold_point = None
            confirm_seconds = CREEP_END_CONFIRM_SECONDS if self.corridor_exit_warning else CORRIDOR_EXIT_CONFIRM_SECONDS
            if self.now_seconds - self.corridor_exit_seen_at_seconds < confirm_seconds:
                return self._step(self._hold_target(drone_metres), corridor_heading, CORRIDOR_CONFIRMING_EXIT)
            self._pending_warning = self.corridor_exit_warning
        elif not has_reached(self.current_goal, previous, drone_metres):
            return None
        finished_name = self.current_goal.stage_name
        last_goal = self.goal_number == len(self.goals) - 1
        if last_goal or finished_name == self.stop_after_stage:
            heading = self.current_goal.straight_heading_radians
            if heading is None:
                heading = self._heading_toward_path_or(drone_metres, 0.0)
            warning, self._pending_warning = self._pending_warning, ""
            return NavigationStep(self._hold_target(drone_metres), heading,
                                  HOLDING, finished=True, stages_finished=[finished_name], warning=warning)
        self.goal_number += 1
        self.path = None
        self.last_plan_seconds = -math.inf
        self.seconds_without_path_since = None
        self._pending_events = ([finished_name], [self.current_goal.stage_name])
        return None

    def _corridor_exit_counts(self, drone_metres):
        """True when the drone may count the first checkpoint: its whole body is past the line, or
        (see the file's header) its centre is near or past the line and the drone can go no deeper. In
        that second case it also prepares a loud warning in corridor_exit_warning, given out when
        the checkpoint really counts."""
        goal = self.current_goal
        if not corridor_exit_reached(goal, drone_metres, -LARGEST_SHORTFALL_AT_LINE_METRES):
            return False
        # There is deliberately no check here that the drone's own square has enough room. PX4 can
        # overshoot a creep target by a few centimetres into a square with less room than the creep
        # needs; the flight will not move it from there, so such a check made it wait until the
        # no-path rule landed it. The next stage backs away from the wall before it turns
        # (_back_away_for_turn_room).
        # The sketch's line is an estimate. During practice, reaching it with the drone's centre
        # tells us to look for the next leg. The turn still needs its own clearance.
        practice_uses_centre = self.full_course_practice or self.stop_after_stage == goal.stage_name
        self.corridor_exit_warning = ""
        if practice_uses_centre:
            return corridor_exit_reached(goal, drone_metres, 0.0)
        if corridor_exit_reached(goal, drone_metres, DRONE_RADIUS_METRES):
            return True
        # It counts only when the flight itself can go no deeper: the same lead and the same
        # smallest lead as in _follow_straight_corridor. A second, different test of "room ahead"
        # here made the drone hold where the flight would not move it, until the no-path rule
        # landed it (noise bench and 3D rendered sim, 2026-11-08). If depth confirms more room
        # during the CREEP_END_CONFIRM_SECONDS hold, the lead grows and the creep goes on.
        room_metres = self._corridor_room_metres(drone_metres)
        _direction, lead_metres = self._corridor_direction_and_lead(drone_metres, room_metres)
        if lead_metres >= CORRIDOR_SMALLEST_LEAD_METRES:
            return False                  # It can still go deeper.
        self.corridor_exit_warning = (
            f"CHECKPOINT '{goal.stage_name.upper()}' COUNTED WITHOUT THE WHOLE DRONE PAST THE LINE "
            f"(centre {metres_past_line(goal, drone_metres):.2f} m past; the whole drone needs {DRONE_RADIUS_METRES:.2f} m). "
            f"The live map leaves no room to go deeper without coming too close to a wall. "
            f"THE REFEREE MAY NOT COUNT THIS CHECKPOINT. Check where the real line and wall are.")
        return True

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
        # Boulders do not stop a turn. The planner already lets a path pass near or over a boulder
        # (at a higher cost, live_planner.py), and refusing to turn there left the drone hovering
        # by the boulder until the no-progress rule landed it on top (noise bench, 2026-11-08).
        room_metres = self._turn_room_metres()
        square = self.live_map.square_of(drone_metres)
        if square is not None and room_metres[square] >= TURN_ROOM_METRES:
            if not self._calm_enough_to_turn(wanted_heading):
                return self._step(self._hold_target(drone_metres), drone_heading_radians, STOPPING_TO_TURN)
            self.seconds_without_path_since = None
            self.turn_heading_radians = wanted_heading
            self.proposed_turn_heading_radians = None
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
        backing_away = self._back_away_for_turn_room(drone_metres, drone_heading_radians, room_metres)
        if backing_away is not None:
            return backing_away
        return self._hold_for_path(drone_metres, drone_heading_radians, "no clear place to turn")

    def _back_away_for_turn_room(self, drone_metres, drone_heading_radians, room_metres):
        """A step straight back (BACK_AWAY_STEP_METRES), if the room there is larger than here.
        Returns None if backing away would not help."""
        here = self.live_map.square_of(drone_metres)
        backward = -np.array([math.cos(drone_heading_radians), math.sin(drone_heading_radians)])
        behind = self.live_map.square_of(drone_metres[:2] + backward * BACK_AWAY_STEP_METRES)
        if here is None or behind is None or room_metres[behind] <= room_metres[here]:
            return None
        self.seconds_without_path_since = None
        return self._advance(drone_metres, backward, BACK_AWAY_STEP_METRES, drone_heading_radians, APPROACHING_TURN)

    # ---- Following the path ---------------------------------------------------------------------

    def _follow_straight_corridor(self, drone_metres, drone_heading_radians):
        heading = self.current_goal.straight_heading_radians
        room_metres = self._corridor_room_metres(drone_metres)
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

        direction, lead_metres = self._corridor_direction_and_lead(drone_metres, room_metres)
        if lead_metres < CORRIDOR_SMALLEST_LEAD_METRES:
            if self.seconds_without_path_since is None:
                self.seconds_without_path_since = self.now_seconds
            if self.now_seconds - self.seconds_without_path_since > LONGEST_SECONDS_WITHOUT_A_PATH:
                raise NoWayForward("The map and depth could not confirm enough room ahead in the first corridor "
                                   f"for {LONGEST_SECONDS_WITHOUT_A_PATH:.0f} seconds. Landing here.")
            return self._step(self._hold_target(drone_metres), heading, CORRIDOR_BLOCKED)

        self.seconds_without_path_since = None
        return self._advance(drone_metres, direction, lead_metres, heading)

    def _corridor_room_metres(self, drone_metres):
        """The room the first corridor is flown with. Boulders count as walls. Until the drone's
        centre is within LARGEST_SHORTFALL_AT_LINE_METRES of the line, the map's walls always
        count. After that (in competition) the
        live map decides, so depth can show that the sketch put the wall beyond the line too
        close (see the file's header). The drone then only moves straight on, down the corridor."""
        near_or_past_line = corridor_exit_reached(self.current_goal, drone_metres, -LARGEST_SHORTFALL_AT_LINE_METRES)
        trust_live_map = near_or_past_line and not self.full_course_practice
        room_metres = self.live_map.room_metres(keep_mapped_walls=not trust_live_map)
        room_metres[self.live_map.near_a_boulder()] = 0.0
        return room_metres

    def _corridor_direction_and_lead(self, drone_metres, room_metres):
        """Which way to move in the first corridor (down it, steering at most
        CORRIDOR_STEERING_LIMIT_DEGREES toward the goal), and how far ahead the target may go."""
        heading = self.current_goal.straight_heading_radians
        offset_metres = self.current_goal.point_metres - drone_metres[:2]
        forward_metres = offset_metres @ np.array([math.cos(heading), math.sin(heading)])
        bearing = math.atan2(offset_metres[1], offset_metres[0])
        steering_limit = math.radians(CORRIDOR_STEERING_LIMIT_DEGREES)
        steering = math.remainder(bearing - heading, math.tau)
        travel_heading = heading + max(-steering_limit, min(steering_limit, steering))
        direction = np.array([math.cos(travel_heading), math.sin(travel_heading)])
        room_needed_metres, stopping_margin_metres = self._corridor_clearance_metres(drone_metres)
        lead_metres = self._safe_lead_metres(drone_metres, direction, min(forward_metres, CORRIDOR_LEAD_METRES),
                                            room_metres, stopping_margin_metres, room_needed_metres)
        return direction, lead_metres

    def _corridor_clearance_metres(self, drone_metres):
        """(room needed, stopping margin) for the first corridor: the creep's small clearance once
        the drone is near or past the first line (competition), else the normal one."""
        near_or_past_line = corridor_exit_reached(self.current_goal, drone_metres, -LARGEST_SHORTFALL_AT_LINE_METRES)
        if near_or_past_line and not self.full_course_practice:
            return CREEP_ROOM_NEEDED_METRES, 0.0
        return ROOM_NEEDED_METRES, STOPPING_MARGIN_METRES

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

    def _safe_lead_metres(self, drone_metres, direction, distance_to_aim_metres, room_metres=None,
                          stopping_margin_metres=STOPPING_MARGIN_METRES, room_needed_metres=ROOM_NEEDED_METRES):
        if room_metres is None and self.full_course_practice:
            room_metres = self._turn_room_metres()
        free_metres = self.live_map.free_distance_metres(drone_metres, direction, room_needed_metres, 2.0, room_metres)
        seen_metres = self.live_map.seen_distance_metres(drone_metres, direction, 2.0, NOT_CHECKED_NEAR_DRONE_METRES)
        return min(LARGEST_LEAD_METRES, free_metres - stopping_margin_metres, seen_metres - room_needed_metres,
                   distance_to_aim_metres)

    def _advance(self, drone_metres, direction, lead_metres, wanted_heading, activity=FLYING):
        self.hold_point = None
        self.proposed_turn_heading_radians = None
        target = np.append(drone_metres[:2] + direction * lead_metres, self.cruise_height_metres)
        return self._step(target, wanted_heading, activity)

    # ---- Small helpers ---------------------------------------------------------------------------

    def _measure_speed(self, drone_metres):
        """Updates speed_metres_per_second from the distance moved over SPEED_MEASURING_SECONDS."""
        if self.speed_measured_from is None:
            self.speed_measured_from = (drone_metres[:2].copy(), self.now_seconds)
            return
        start_metres, start_seconds = self.speed_measured_from
        seconds = self.now_seconds - start_seconds
        if seconds < SPEED_MEASURING_SECONDS:
            return
        self.speed_metres_per_second = float(np.linalg.norm(drone_metres[:2] - start_metres)) / seconds
        self.speed_measured_from = (drone_metres[:2].copy(), self.now_seconds)

    def _calm_enough_to_turn(self, wanted_heading):
        """True when the same new direction has been asked for long enough (TURN_AGREEMENT_SECONDS)
        and the drone is nearly still. A direction that changes by more than
        TURN_AGREEMENT_DEGREES starts the wait again."""
        proposed = self.proposed_turn_heading_radians
        if proposed is None or _angle_between(proposed, wanted_heading) > math.radians(TURN_AGREEMENT_DEGREES):
            self.proposed_turn_heading_radians = wanted_heading
            self.proposed_turn_since_seconds = self.now_seconds
            return False
        if self.now_seconds - self.proposed_turn_since_seconds < TURN_AGREEMENT_SECONDS:
            return False
        return self.speed_metres_per_second < LARGEST_SPEED_TO_START_A_TURN_METRES_PER_SECOND

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
        warning, self._pending_warning = self._pending_warning, ""
        return NavigationStep(target, heading, activity, stages_finished=list(finished), stages_begun=list(begun),
                              warning=warning)


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
