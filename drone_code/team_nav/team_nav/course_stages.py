"""The course stages: what the drone does between takeoff and landing.

How it fits: once the drone has climbed to cruise height, mission.py calls Course.step twenty times a
second. Course answers with a target position, a short reason for its choice, and whether the drone
should now land. landing.py then does the landing. This file never talks to ROS and never reads a
clock: every call carries the time, so the tests can run it on a laptop.

The stages, in order:

    follow_line -> gate_a -> tunnel_align -> tunnel_cross -> tunnel_exit -> chicane -> find_pad

After find_pad, the mission lands on the pad.

What it deliberately does not do: plan a route from the map, keep the drone inside the course
boundaries, take off, or land. It also never moves the drone up or down.

Words used here:
* A "setpoint" is the position and heading we ask the drone to fly to. We keep it a short way ahead
  of the drone (the "carrot"), so the drone flies smoothly at a gentle speed.
* Positions are in the camera tracking frame, called ENU: x points East, y North, z Up, in metres.
* "Heading" is the direction the nose points, in radians, anticlockwise from East.
* "Forward" and "left" are relative to the nose.

Safety rules (each one is a test in test/test_course_stages.py):
* An input that is missing, too old, or stamped in the future is invalid (mission_inputs.py).
* The drone only moves into space the depth camera has seen to be free: the carrot never goes
  further ahead than the free distance minus the stopping distance. Unknown space means hold.
  Turning on the spot needs no clearance.
* If camera tracking is not reported healthy (vision_ok), the drone holds and stops advancing. It
  does not even turn.
* A hold that is waiting for an input lands after longest_wait_for_input_seconds. A stage that runs
  past its time limit hovers for hover_after_time_limit_seconds, then lands, unless the stage's own
  finishing test passes during that hover.
* Every decision comes with a short reason, which mission.py logs whenever it changes.
"""
import math
from dataclasses import dataclass

from .mission_inputs import is_finite_number, read_inputs

# Our own choice: when squaring up to the barrier, never aim more than this far round at once.
SQUARE_UP_LARGEST_TURN_DEGREES = 10.0
# Our own choice: when turning in the chicane, aim this far round from the nose. The aim moves with
# the nose, so the drone keeps turning at its turn rate until the way ahead opens.
CHICANE_TURN_AIM_DEGREES = 30.0
# Our own choice: one side must be this much more open than the other to count as more open.
CLEARLY_MORE_OPEN_METRES = 0.2

STAGES = ("follow_line", "gate_a", "tunnel_align", "tunnel_cross", "tunnel_exit", "chicane", "find_pad")
COURSE_COMPLETE = "course complete: centred over the pad"


@dataclass
class StageAction:
    """One stage's choice for this step. Movement is either speeds relative to the nose, or a goal
    position to fly to. waiting_for_input marks a hold that lands if it goes on too long."""
    reason: str
    forward_speed_metres_per_second: float = 0.0
    left_speed_metres_per_second: float = 0.0
    goal_position: tuple | None = None
    target_heading_radians: float | None = None
    waiting_for_input: bool = False
    next_stage: str | None = None
    land: bool = False
    # How far the drone may go on space the depth camera saw free a moment ago, while it sees
    # nothing now (only the tunnel uses this, for a dark stretch).
    remembered_free_metres: float = 0.0

    def moves(self):
        return (self.forward_speed_metres_per_second != 0.0 or self.left_speed_metres_per_second != 0.0
                or self.goal_position is not None)


@dataclass
class StageDecision:
    stage: str
    reason: str
    setpoint: tuple                   # (x, y, z, heading)
    land: bool


def wrap_angle(angle_radians):
    """The same angle, between -pi and +pi."""
    return (angle_radians + math.pi) % (2 * math.pi) - math.pi


def limit(value, largest_magnitude):
    return max(-largest_magnitude, min(largest_magnitude, value))


def distance_travelled_along(position, start, heading_radians):
    return ((position[0] - start[0]) * math.cos(heading_radians)
            + (position[1] - start[1]) * math.sin(heading_radians))


def body_to_world(position, heading_radians, forward_metres, left_metres):
    cos_heading, sin_heading = math.cos(heading_radians), math.sin(heading_radians)
    return (position[0] + cos_heading * forward_metres - sin_heading * left_metres,
            position[1] + sin_heading * forward_metres + cos_heading * left_metres)


def world_to_body(position, heading_radians, point):
    east, north = point[0] - position[0], point[1] - position[1]
    cos_heading, sin_heading = math.cos(heading_radians), math.sin(heading_radians)
    return cos_heading * east + sin_heading * north, -sin_heading * east + cos_heading * north


def free_distance_seen(distance_metres, judged_out_to_metres):
    """How far ahead a clearance shows free space: up to the obstacle, or, when clear, up to where
    the depth picture was judged."""
    if math.isinf(distance_metres):
        return judged_out_to_metres if is_finite_number(judged_out_to_metres) else 0.0
    return distance_metres


class Course:
    def __init__(self, settings):
        self.settings = settings
        self.stage = None
        self.stage_start_seconds = None
        self.setpoint = None                      # [x, y, z, heading]
        self.time_limit_hover_since_seconds = None
        self.waiting_since_seconds = None
        self.carrot_was_moving = False
        self.last_counted_stamp = {}
        self.new_clearances_this_step = False
        self.line_end_sightings = 0
        self.pad_sightings = 0
        self.pad_position = None                  # the pad on the floor (x, y): a world-position goal
        self.free_space_seen_until = None         # (x, y, cos, sin): the furthest point seen free ahead
        self.stage_memory = {}
        self.room_ahead_metres = 0.0

    # ------------------------------------------------------------------ calls from mission.py
    def start(self, now_seconds, setpoint):
        """After the takeoff. setpoint = (x, y, z, heading) at cruise height."""
        self.setpoint = [float(value) for value in setpoint]
        self.enter_stage("follow_line", now_seconds)

    def drop_world_goals(self):
        """Forget every remembered position in the world. Used after the camera tracking jumps,
        when they no longer mean anything."""
        self.pad_position = None
        self.free_space_seen_until = None
        self.stage_memory = {}

    def step(self, now_seconds, seconds_since_last_step, position, heading_radians, inputs):
        view = read_inputs(inputs, now_seconds, self.settings)
        self.count_new_reports(position, heading_radians, view)
        deciding_stage = self.stage
        action = self.choose_action(now_seconds, position, heading_radians, view)
        action = self.refuse_moves_into_unseen_space(action, view)
        action = self.land_after_waiting_too_long(now_seconds, action)
        self.move_setpoint(position, heading_radians, seconds_since_last_step, action)
        if action.next_stage:
            self.enter_stage(action.next_stage, now_seconds)
        return StageDecision(deciding_stage, action.reason, tuple(self.setpoint), action.land)

    # ------------------------------------------------------------------ bookkeeping
    def enter_stage(self, stage, now_seconds):
        self.stage = stage
        self.stage_start_seconds = now_seconds
        self.stage_memory = {}
        self.time_limit_hover_since_seconds = None
        self.waiting_since_seconds = None

    def count_new_reports(self, position, heading_radians, view):
        """Count each detector report once, however many steps it is seen for."""
        inputs = view.inputs
        if inputs.line_stamp_seconds is not None and inputs.line_stamp_seconds != self.last_counted_stamp.get("line"):
            self.last_counted_stamp["line"] = inputs.line_stamp_seconds
            if view.line_ok and inputs.line_end_seen:
                self.line_end_sightings += 1
            else:
                self.line_end_sightings = 0
        if inputs.pad_stamp_seconds is not None and inputs.pad_stamp_seconds != self.last_counted_stamp.get("pad"):
            self.last_counted_stamp["pad"] = inputs.pad_stamp_seconds
            self.remember_pad(position, heading_radians, view)
        stamp = inputs.clearances_stamp_seconds
        self.new_clearances_this_step = view.clearances is not None and stamp != self.last_counted_stamp.get("clear")
        if self.new_clearances_this_step:
            self.last_counted_stamp["clear"] = stamp
            self.remember_free_space(position, heading_radians, view.clearances)

    def remember_pad(self, position, heading_radians, view):
        if not view.pad_ok:
            self.pad_sightings = 0
            return
        forward, left = view.inputs.pad_offset_forward_left_metres
        seen_at = body_to_world(position, heading_radians, forward, left)
        if self.pad_position is None:
            self.pad_position = seen_at
        else:
            fraction = self.settings.pad_smoothing_fraction
            self.pad_position = (fraction * seen_at[0] + (1 - fraction) * self.pad_position[0],
                                 fraction * seen_at[1] + (1 - fraction) * self.pad_position[1])
        self.pad_sightings += 1

    def remember_free_space(self, position, heading_radians, clearances):
        if clearances.ahead_metres is None:
            return
        free = free_distance_seen(clearances.ahead_metres, clearances.judged_out_to_metres)
        end = body_to_world(position, heading_radians, free, 0.0)
        self.free_space_seen_until = (end[0], end[1], math.cos(heading_radians), math.sin(heading_radians))

    def remembered_room(self, position):
        if self.free_space_seen_until is None:
            return 0.0
        end_x, end_y, cos_heading, sin_heading = self.free_space_seen_until
        ahead = (end_x - position[0]) * cos_heading + (end_y - position[1]) * sin_heading
        return max(0.0, ahead - self.settings.stopping_distance_metres)

    # ------------------------------------------------------------------ the rules every stage obeys
    def refuse_moves_into_unseen_space(self, action, view):
        """Turn a move into a hold unless the depth camera has seen room ahead."""
        self.room_ahead_metres = 0.0
        if not action.moves() or action.land:
            return action
        room, why_not = self.room_ahead(view, action)
        if room > 0.0:
            self.room_ahead_metres = room
            return action
        return StageAction(why_not, target_heading_radians=action.target_heading_radians, waiting_for_input=True)

    def room_ahead(self, view, action):
        """(metres the carrot may lead, "") or (0, the reason it may not move)."""
        settings = self.settings
        clearances = view.clearances
        if clearances is not None and clearances.ahead_metres is not None:
            free = free_distance_seen(clearances.ahead_metres, clearances.judged_out_to_metres)
            room = free - settings.stopping_distance_metres
            if room > settings.smallest_useful_room_metres:
                return room, ""
            return 0.0, "obstacle ahead, holding"
        if action.remembered_free_metres > settings.smallest_useful_room_metres:
            return action.remembered_free_metres, ""
        if clearances is not None:
            return 0.0, "way ahead unknown, holding"
        return 0.0, f"depth clearances {view.clearances_condition}, holding"

    def land_after_waiting_too_long(self, now_seconds, action):
        if not action.waiting_for_input or action.land:
            self.waiting_since_seconds = None
            return action
        if self.waiting_since_seconds is None:
            self.waiting_since_seconds = now_seconds
        waited = now_seconds - self.waiting_since_seconds
        if waited < self.settings.longest_wait_for_input_seconds:
            return action
        what = action.reason.removesuffix(", holding")
        return StageAction(f"{what} for {self.settings.longest_wait_for_input_seconds:g} s, landing", land=True)

    def move_setpoint(self, position, heading_radians, seconds_since_last_step, action):
        if action.land or not action.moves():
            if self.carrot_was_moving:            # stop where the drone is, not where the carrot was
                self.setpoint[0], self.setpoint[1] = float(position[0]), float(position[1])
            self.carrot_was_moving = False
        else:
            self.advance_carrot(position, heading_radians, seconds_since_last_step, action)
            self.carrot_was_moving = True
        if action.target_heading_radians is not None and not action.land:
            self.turn_toward(action.target_heading_radians, seconds_since_last_step)

    def advance_carrot(self, position, heading_radians, seconds_since_last_step, action):
        settings = self.settings
        carrot = (self.setpoint[0], self.setpoint[1])
        if action.goal_position is not None:
            east, north = action.goal_position[0] - carrot[0], action.goal_position[1] - carrot[1]
            distance = math.hypot(east, north)
            step = settings.slow_speed_metres_per_second * seconds_since_last_step
            fraction = min(1.0, step / distance) if distance > 1e-9 else 0.0
            carrot = (carrot[0] + east * fraction, carrot[1] + north * fraction)
        else:
            moved = body_to_world((0.0, 0.0), heading_radians,
                                  action.forward_speed_metres_per_second * seconds_since_last_step,
                                  action.left_speed_metres_per_second * seconds_since_last_step)
            carrot = (carrot[0] + moved[0], carrot[1] + moved[1])
        # Keep the carrot a bounded way from the drone: forward no further than the room seen free,
        # sideways and backward only a little, because the forward camera cannot see there.
        forward, left = world_to_body(position, heading_radians, carrot)
        forward = max(-settings.sideways_carrot_lead_metres,
                      min(forward, settings.carrot_lead_metres, self.room_ahead_metres))
        left = limit(left, settings.sideways_carrot_lead_metres)
        # PX4 flies to a position target as fast as 12 m/s, so the carrot's whole distance from the
        # drone, not just its forward part, stays within the carrot lead.
        distance = math.hypot(forward, left)
        if distance > settings.carrot_lead_metres:
            forward, left = forward * settings.carrot_lead_metres / distance, left * settings.carrot_lead_metres / distance
        self.setpoint[0], self.setpoint[1] = body_to_world(position, heading_radians, forward, left)

    def turn_toward(self, target_heading_radians, seconds_since_last_step):
        largest_turn = math.radians(self.settings.turn_rate_degrees_per_second) * seconds_since_last_step
        error = wrap_angle(target_heading_radians - self.setpoint[3])
        self.setpoint[3] = wrap_angle(self.setpoint[3] + limit(error, largest_turn))

    def choose_action(self, now_seconds, position, heading_radians, view):
        settings = self.settings
        if not view.vision_ok:
            return StageAction(f"camera tracking {view.vision_condition}, holding", waiting_for_input=True)
        stage_function = getattr(self, "stage_" + self.stage)
        if self.time_limit_hover_since_seconds is not None:
            return self.hover_after_time_limit(now_seconds, position, heading_radians, view, stage_function)
        time_limit = getattr(settings, f"{self.stage}_time_limit_seconds")
        if now_seconds - self.stage_start_seconds > time_limit:
            self.time_limit_hover_since_seconds = now_seconds
            return StageAction(f"{self.stage} ran out of time after {time_limit:g} s, hovering")
        return stage_function(now_seconds, position, heading_radians, view)

    def hover_after_time_limit(self, now_seconds, position, heading_radians, view, stage_function):
        """Hold still. The stage still runs, but only its finishing test is used."""
        action = stage_function(now_seconds, position, heading_radians, view)
        if action.next_stage or action.land:
            return StageAction(f"{action.reason} (while hovering after the time limit)",
                               next_stage=action.next_stage, land=action.land)
        hover_seconds = self.settings.hover_after_time_limit_seconds
        if now_seconds - self.time_limit_hover_since_seconds >= hover_seconds:
            return StageAction(f"{self.stage} ran out of time; hovered {hover_seconds:g} s, landing", land=True)
        return StageAction(f"{self.stage} ran out of time, hovering")

    # ------------------------------------------------------------------ the stages
    def stage_follow_line(self, now_seconds, position, heading_radians, view):
        settings = self.settings
        if not view.line_ok:
            return StageAction(f"line {view.line_condition}, holding", waiting_for_input=True)
        if self.line_end_sightings >= settings.sightings_to_confirm:
            return StageAction("line end seen", next_stage="gate_a")
        angle = view.inputs.line_angle_left_radians
        heading_along_line = heading_radians + angle
        if abs(angle) > math.radians(settings.turn_on_the_spot_degrees):
            return StageAction("turning onto the line", target_heading_radians=heading_along_line)
        sideways = limit(settings.line_follow_gain_per_second * view.inputs.line_offset_left_metres,
                         settings.sideways_speed_limit_metres_per_second)
        return StageAction("following the line", forward_speed_metres_per_second=settings.cruise_speed_metres_per_second,
                           left_speed_metres_per_second=sideways, target_heading_radians=heading_along_line)

    def stage_gate_a(self, now_seconds, position, heading_radians, view):
        settings, memory = self.settings, self.stage_memory
        if "start" not in memory:                 # where the line ended, and which way it pointed
            memory["start"] = (position[0], position[1])
            memory["heading"] = heading_radians
        if distance_travelled_along(position, memory["start"], memory["heading"]) >= settings.gate_a_pass_distance_metres:
            return StageAction("through gate A", next_stage="tunnel_align")
        return StageAction("crossing gate A", forward_speed_metres_per_second=settings.slow_speed_metres_per_second,
                           left_speed_metres_per_second=self.centring_speed(view.clearances),
                           target_heading_radians=memory["heading"])

    def centring_speed(self, clearances):
        """Sideways speed toward the middle of an opening, from the left and right gaps."""
        settings = self.settings
        if clearances is None:
            return 0.0
        left, right = clearances.left_metres, clearances.right_metres
        if is_finite_number(left) and is_finite_number(right):
            return limit(settings.gap_centring_gain_per_second * (left - right),
                         settings.sideways_speed_limit_metres_per_second)
        if is_finite_number(right) and right < settings.barrier_side_distance_metres:
            return settings.sideways_speed_limit_metres_per_second / 2
        if is_finite_number(left) and left < settings.barrier_side_distance_metres:
            return -settings.sideways_speed_limit_metres_per_second / 2
        return 0.0

    def stage_tunnel_align(self, now_seconds, position, heading_radians, view):
        settings, memory = self.settings, self.stage_memory
        memory.setdefault("heading", heading_radians)
        memory.setdefault("steady_pictures", 0)
        if "aligned_at_seconds" in memory:
            return self.hover_at_tunnel_entrance(now_seconds)
        clearances = view.clearances
        if clearances is None:
            return StageAction(f"depth clearances {view.clearances_condition}, holding", waiting_for_input=True)
        ahead, left, right = clearances.ahead_metres, clearances.left_metres, clearances.right_metres
        facing_barrier = is_finite_number(ahead) and ahead <= settings.tunnel_standoff_metres
        opening_framed = (is_finite_number(left) and is_finite_number(right)
                          and max(left, right) <= settings.barrier_side_distance_metres)
        if not (facing_barrier or opening_framed):
            return StageAction("approaching the barrier", forward_speed_metres_per_second=settings.slow_speed_metres_per_second,
                               target_heading_radians=memory["heading"])
        if is_finite_number(left) and is_finite_number(right) and abs(left - right) > settings.square_up_tolerance_metres:
            memory["steady_pictures"] = 0
            # Turn toward the nearer side until both sides read the same: then the nose is square.
            turn = limit(settings.square_up_gain_radians_per_metre * (right - left),
                         math.radians(SQUARE_UP_LARGEST_TURN_DEGREES))
            return StageAction("squaring up to the barrier", target_heading_radians=heading_radians + turn)
        if self.new_clearances_this_step:
            memory["steady_pictures"] += 1
        if memory["steady_pictures"] < settings.sightings_to_confirm:
            return StageAction("at the tunnel entrance, settling")
        if settings.tunnel_enabled:
            return StageAction("lined up with the tunnel entrance", next_stage="tunnel_cross")
        memory["aligned_at_seconds"] = now_seconds
        return StageAction("tunnel disabled: hovering at the tunnel entrance")

    def hover_at_tunnel_entrance(self, now_seconds):
        waited = now_seconds - self.stage_memory["aligned_at_seconds"]
        if waited >= self.settings.hover_at_tunnel_entrance_seconds:
            return StageAction("tunnel disabled: landing at the tunnel entrance", land=True)
        return StageAction("tunnel disabled: hovering at the tunnel entrance")

    def stage_tunnel_cross(self, now_seconds, position, heading_radians, view):
        settings, memory = self.settings, self.stage_memory
        if "start" not in memory:
            memory["start"] = (position[0], position[1])
            memory["heading"] = heading_radians
            memory["went_dark"] = False
        if view.height_above_floor_metres is None:
            return StageAction(f"laser height {view.height_condition}, holding", waiting_for_input=True)
        travelled = distance_travelled_along(position, memory["start"], memory["heading"])
        ahead = view.clearances.ahead_metres if view.clearances is not None else None
        memory["went_dark"] = memory["went_dark"] or ahead is None
        if travelled >= settings.tunnel_travel_limit_metres:
            return StageAction("tunnel length crossed", next_stage="tunnel_exit")
        depth_is_back = ahead is not None and ahead >= settings.tunnel_exit_clear_metres
        if memory["went_dark"] and depth_is_back and travelled >= settings.tunnel_minimum_travel_metres:
            return StageAction("depth back after the dark stretch", next_stage="tunnel_exit")
        return StageAction("crossing the tunnel", forward_speed_metres_per_second=settings.slow_speed_metres_per_second,
                           target_heading_radians=memory["heading"], remembered_free_metres=self.remembered_room(position))

    def stage_tunnel_exit(self, now_seconds, position, heading_radians, view):
        memory = self.stage_memory                # camera tracking is checked before every stage
        if view.clearances is None or view.clearances.ahead_metres is None:
            memory.pop("depth_back_since", None)
            return StageAction("out of the tunnel, waiting for depth")
        memory.setdefault("depth_back_since", now_seconds)
        if now_seconds - memory["depth_back_since"] >= self.settings.tunnel_exit_settle_seconds:
            return StageAction("camera tracking and depth back after the tunnel", next_stage="chicane")
        return StageAction("out of the tunnel, settling")

    def stage_chicane(self, now_seconds, position, heading_radians, view):
        settings, memory = self.settings, self.stage_memory
        if self.pad_sightings >= settings.sightings_to_confirm:
            return StageAction("pad in view", next_stage="find_pad")
        prefix = "no window detector yet, gate B: " if settings.window_enabled else "gate B: "
        clearances = view.clearances
        if clearances is None:
            return StageAction(f"depth clearances {view.clearances_condition}, holding", waiting_for_input=True)
        ahead = clearances.ahead_metres
        needed = settings.chicane_go_again_distance_metres if memory.get("turning") else settings.chicane_turn_distance_metres
        if ahead is not None and ahead >= needed:
            memory["turning"] = False
            return StageAction(prefix + "open ahead", forward_speed_metres_per_second=settings.cruise_speed_metres_per_second)
        memory["turning"] = True
        turn = self.more_open_side(clearances, memory.get("turn", settings.chicane_default_turn))
        memory["turn"] = turn
        return StageAction(prefix + f"blocked ahead, turning {'left' if turn > 0 else 'right'}",
                           target_heading_radians=heading_radians + turn * math.radians(CHICANE_TURN_AIM_DEGREES))

    @staticmethod
    def more_open_side(clearances, default_turn):
        """+1 for left, -1 for right. Unknown counts as least open, seen clear as most open."""
        def openness(distance):
            if distance is None:
                return -1.0
            return 1e9 if math.isinf(distance) else distance
        left, right = openness(clearances.left_metres), openness(clearances.right_metres)
        if left > right + CLEARLY_MORE_OPEN_METRES:
            return 1
        if right > left + CLEARLY_MORE_OPEN_METRES:
            return -1
        return default_turn

    def stage_find_pad(self, now_seconds, position, heading_radians, view):
        settings, memory = self.settings, self.stage_memory
        if self.pad_position is None:
            return StageAction("pad position unknown, holding", waiting_for_input=True)
        east, north = self.pad_position[0] - position[0], self.pad_position[1] - position[1]
        distance = math.hypot(east, north)
        if distance <= settings.pad_centre_tolerance_metres:
            memory.setdefault("over_pad_since", now_seconds)
            if now_seconds - memory["over_pad_since"] >= settings.pad_settle_seconds:
                return StageAction(COURSE_COMPLETE, land=True)
            return StageAction("over the pad, settling")
        memory.pop("over_pad_since", None)
        bearing = math.atan2(north, east)
        far = distance > settings.stop_facing_pad_metres
        if far and abs(wrap_angle(bearing - heading_radians)) > math.radians(settings.turn_on_the_spot_degrees):
            return StageAction("turning to the pad", target_heading_radians=bearing)
        return StageAction("flying to the pad", goal_position=self.pad_position,
                           target_heading_radians=bearing if far else None)
