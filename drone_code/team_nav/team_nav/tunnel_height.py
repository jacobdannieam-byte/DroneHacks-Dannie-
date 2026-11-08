"""The tunnel_first mission's height safeguards: the height gate before the turn, the height watch
whenever the drone is high, where to come down when the watch stops the run, and the height report.

REAL DRONE CODE, also used in the simulator and the tests. It is plain arithmetic, with no ROS or
PX4. tunnel_first_run.py feeds it readings.

Why (the team's analysis, 2026-11-08)
-------------------------------------
Drone 16's PX4 gets no height from camera tracking (EKF2_EV_CTRL = 9: horizontal position and
heading only). Over a wall PX4 stops using the bottom laser, and its height then runs on the motion
sensor and the air pressure sensor, which drift. PX4 takes the laser back only below about 2.1 m,
slower than 0.7 m/s sideways, after its consistency check passes, and taking it back doesn't undo the
error built up meanwhile. At 2.4 m the drone can be only about 0.28 m lower than it thinks before it
clips a 1.87 m wall. So we watch PX4's height against camera tracking's ourselves.

The disagreement
----------------
    disagreement = (PX4's height - camera tracking's height) - b0
both heights measured up from the launch mark. b0 is that same difference, averaged while the drone
hovers steady at 0.70 m above the launch mark just after takeoff, with PX4 on the laser: the two
heights' zero points differ a little, and b0 removes that.

The height gate (at 0.90 m at S, before the turn)
-------------------------------------------------
Passes once, for GATE_STEADY_SECONDS in a row: PX4 is using the laser for height (cs_rng_hgt), its
laser consistency check passes (cs_rng_kin_consistent), its height has not been reset
(z_reset_counter unchanged), and |disagreement| <= GATE_LARGEST_DISAGREEMENT_METRES. With
TEAM_NAV_TUNNEL_GATE=relaxed only |disagreement| <= RELAXED_LARGEST_DISAGREEMENT_METRES counts. Fails
if that has not happened within GATE_LONGEST_SECONDS; it then says which condition failed most.

The height watch (whenever camera tracking puts the drone above WATCH_ABOVE_METRES)
-------------------------------------------------------------------------------
Warns at |disagreement| > WARN_METRES for WARN_SECONDS. Stops the run at > STOP_METRES for
STOP_SECONDS, at any change of PX4's height reset counter, or if PX4 says its vertical position or
speed is not valid.

Coming down after the watch stops the run
-----------------------------------------
Over open floor: straight down. Over a wall (or the tunnel): keep the height, carry on along the
planned path to the first point within DESCENT_SEARCH_METRES whose floor is open, and come down
there; if none is ahead, the nearest open floor within that distance in any direction. Never down
over a wall, never up.

What it does not do: send anything.
"""
import math

import numpy as np

from team_nav.high_path_rules import POSITION_DRIFT_METRES, laser_reach_metres
from team_nav.course_map import distance_to_line_piece_metres
from team_nav.high_path_rules import distance_to_tunnel_metres

# The team's choices (2026-11-08).
GATE_STEADY_SECONDS = 2.0
GATE_LONGEST_SECONDS = 4.0
GATE_LARGEST_DISAGREEMENT_METRES = 0.10
RELAXED_LARGEST_DISAGREEMENT_METRES = 0.15
GATE_SETTING = "TEAM_NAV_TUNNEL_GATE"
GATE_STRICT = "strict"
GATE_RELAXED = "relaxed"
WATCH_ABOVE_METRES = 1.5
WARN_METRES = 0.15
WARN_SECONDS = 0.3
STOP_METRES = 0.20
STOP_SECONDS = 0.2
DESCENT_SEARCH_METRES = 1.5
# Our own choice: points along the path are tried this far apart.
DESCENT_SEARCH_STEP_METRES = 0.05


def disagreement_metres(px4_height_metres, camera_height_metres, baseline_metres):
    if px4_height_metres is None or camera_height_metres is None or baseline_metres is None:
        return None
    return (px4_height_metres - camera_height_metres) - baseline_metres


class HeightGate:
    """Feed it one reading per cycle; .passed or .failed_because becomes set."""

    def __init__(self, start_seconds, mode=GATE_STRICT, z_reset_counter_at_start=None):
        self.start_seconds = start_seconds
        self.mode = mode
        self.z_reset_counter = z_reset_counter_at_start
        self.good_since_seconds = None
        self.passed = False
        self.failed_because = None
        self.fail_counts = {}

    def update(self, now_seconds, uses_laser, laser_consistent, z_reset_counter, disagreement):
        """Returns True once passed. Sets failed_because once GATE_LONGEST_SECONDS has gone by."""
        problems = self._problems(uses_laser, laser_consistent, z_reset_counter, disagreement)
        for problem in problems:
            self.fail_counts[problem] = self.fail_counts.get(problem, 0) + 1
        if problems:
            self.good_since_seconds = None
        elif self.good_since_seconds is None:
            self.good_since_seconds = now_seconds
        if self.good_since_seconds is not None and now_seconds - self.good_since_seconds >= GATE_STEADY_SECONDS:
            self.passed = True
            return True
        if now_seconds - self.start_seconds >= GATE_LONGEST_SECONDS:
            worst = max(self.fail_counts, key=self.fail_counts.get) if self.fail_counts else "never steady for 2 s"
            self.failed_because = worst
        return False

    def _problems(self, uses_laser, laser_consistent, z_reset_counter, disagreement):
        problems = []
        limit = GATE_LARGEST_DISAGREEMENT_METRES if self.mode == GATE_STRICT else RELAXED_LARGEST_DISAGREEMENT_METRES
        if disagreement is None or abs(disagreement) > limit:
            problems.append(f"height disagreement over {limit:.2f} m")
        if self.mode == GATE_RELAXED:
            return problems
        if not uses_laser:
            problems.append("PX4 not using the laser for height (cs_rng_hgt)")
        if not laser_consistent:
            problems.append("PX4's laser consistency check not passing (cs_rng_kin_consistent)")
        if z_reset_counter is not None and self.z_reset_counter is not None and z_reset_counter != self.z_reset_counter:
            problems.append("PX4 reset its height (z_reset_counter changed)")
            self.z_reset_counter = z_reset_counter
        if self.z_reset_counter is None:
            self.z_reset_counter = z_reset_counter
        return problems


# The height watch's answers.
WATCH_OK = "ok"
WATCH_WARN = "warn"
WATCH_STOP = "stop"


class HeightWatch:

    def __init__(self, z_reset_counter_at_start=None):
        self.z_reset_counter = z_reset_counter_at_start
        self.warn_since_seconds = None
        self.stop_since_seconds = None
        self.reason = ""

    def update(self, now_seconds, camera_height_metres, disagreement, z_reset_counter, z_valid, v_z_valid):
        """Returns WATCH_OK, WATCH_WARN or WATCH_STOP; .reason says why for the last two."""
        reset = self._reset_happened(z_reset_counter)
        if camera_height_metres is None or camera_height_metres <= WATCH_ABOVE_METRES:
            self.warn_since_seconds = self.stop_since_seconds = None
            return WATCH_OK
        if reset:
            self.reason = f"PX4 reset its height estimate above {WATCH_ABOVE_METRES:.1f} m (z_reset_counter changed)"
            return WATCH_STOP
        if z_valid is False or v_z_valid is False:
            self.reason = "PX4 says its vertical position or speed is not valid"
            return WATCH_STOP
        size = None if disagreement is None else abs(disagreement)
        self.stop_since_seconds = _since(self.stop_since_seconds, size is not None and size > STOP_METRES, now_seconds)
        self.warn_since_seconds = _since(self.warn_since_seconds, size is not None and size > WARN_METRES, now_seconds)
        if self.stop_since_seconds is not None and now_seconds - self.stop_since_seconds >= STOP_SECONDS:
            self.reason = (f"PX4's height and camera tracking's disagree by {disagreement:+.2f} m (after the launch "
                           f"baseline), more than {STOP_METRES:.2f} m for {STOP_SECONDS:.1f} s")
            return WATCH_STOP
        if self.warn_since_seconds is not None and now_seconds - self.warn_since_seconds >= WARN_SECONDS:
            self.reason = f"height disagreement {disagreement:+.2f} m, over {WARN_METRES:.2f} m"
            return WATCH_WARN
        return WATCH_OK

    def _reset_happened(self, z_reset_counter):
        if z_reset_counter is None:
            return False
        if self.z_reset_counter is None:
            self.z_reset_counter = z_reset_counter
            return False
        changed = z_reset_counter != self.z_reset_counter
        self.z_reset_counter = z_reset_counter
        return changed


def _since(since_seconds, holds, now_seconds):
    if not holds:
        return None
    return now_seconds if since_seconds is None else since_seconds


def wall_or_tunnel_under(course_map, point_metres, height_metres):
    """The name of a wall or the tunnel the bottom laser could see from here (with drift), or None.
    Low obstacles (boulders) don't count: coming down over one is allowed."""
    point = np.asarray(point_metres, dtype=float)[np.newaxis, :2]
    for wall in course_map.walls:
        reach = laser_reach_metres(height_metres, wall.top_height_metres) + POSITION_DRIFT_METRES
        distance = distance_to_line_piece_metres(point, wall.start_metres, wall.end_metres)[0] - wall.thickness_metres / 2.0
        if distance <= reach:
            return wall.name
    for tube in course_map.tubes:
        reach = laser_reach_metres(height_metres, tube.outside_height_metres) + POSITION_DRIFT_METRES
        if distance_to_tunnel_metres(point, tube)[0] <= reach:
            return tube.name
    return None


def choose_descent_point(course_map, drone_metres, path_ahead_metres):
    """Where to come down: (east, north), or None if there is no open floor within DESCENT_SEARCH_METRES.
    path_ahead_metres is the planned path's (east, north) points from the drone onwards."""
    drone = np.asarray(drone_metres, dtype=float)
    height = float(drone[2])
    if wall_or_tunnel_under(course_map, drone[:2], height) is None:
        return drone[:2].copy()
    for point in _points_along(drone[:2], path_ahead_metres):
        if wall_or_tunnel_under(course_map, point, height) is None:
            return point
    return _nearest_open_floor(course_map, drone[:2], height)


def _points_along(start, path_ahead_metres):
    """Points every DESCENT_SEARCH_STEP_METRES along the path from start, up to DESCENT_SEARCH_METRES."""
    points, travelled = [], 0.0
    previous = np.asarray(start, dtype=float)
    for corner in path_ahead_metres:
        corner = np.asarray(corner, dtype=float)[:2]
        leg = float(np.linalg.norm(corner - previous))
        steps = max(1, int(math.ceil(leg / DESCENT_SEARCH_STEP_METRES)))
        for step in range(1, steps + 1):
            point = previous + (corner - previous) * step / steps
            if travelled + leg * step / steps > DESCENT_SEARCH_METRES:
                return points
            points.append(point)
        travelled += leg
        previous = corner
    return points


def _nearest_open_floor(course_map, centre, height):
    best, best_distance = None, math.inf
    for east in np.arange(-DESCENT_SEARCH_METRES, DESCENT_SEARCH_METRES + 1e-9, 0.1):
        for north in np.arange(-DESCENT_SEARCH_METRES, DESCENT_SEARCH_METRES + 1e-9, 0.1):
            distance = math.hypot(east, north)
            if distance > DESCENT_SEARCH_METRES or distance >= best_distance:
                continue
            point = centre + np.array([east, north])
            inside_net = (np.all(point >= np.asarray(course_map.lowest_corner_metres) + 0.35)
                          and np.all(point <= np.asarray(course_map.highest_corner_metres) - 0.35))
            if inside_net and wall_or_tunnel_under(course_map, point, height) is None:
                best, best_distance = point, distance
    return best


class HeightRecord:
    """Collects what the after-run height report needs."""

    def __init__(self):
        self.laser_changes = []          # (seconds, stage, camera height, now using the laser)
        self.last_uses_laser = None
        self.largest = (0.0, None, None)  # (|disagreement|, seconds, stage)
        self.resets = []                 # (seconds, stage, what, counter)
        self.counters = {}
        self.gate_result = "not reached"

    def update(self, seconds, stage, camera_height, uses_laser, disagreement, counters):
        if uses_laser is not None and uses_laser != self.last_uses_laser:
            if self.last_uses_laser is not None:
                self.laser_changes.append((seconds, stage, camera_height, uses_laser))
            self.last_uses_laser = uses_laser
        if disagreement is not None and abs(disagreement) > self.largest[0]:
            self.largest = (abs(disagreement), seconds, stage, disagreement)
        for name, value in counters.items():
            if value is None:
                continue
            before = self.counters.get(name)
            if before is not None and value != before:
                self.resets.append((seconds, stage, name, value))
            self.counters[name] = value

    def paragraph(self):
        changes = "; ".join(f"{'back' if uses else 'dropped'} at {seconds:.1f} s ({stage}, {height:.2f} m)"
                            for seconds, stage, height, uses in self.laser_changes) or "never changed"
        if self.largest[1] is None:
            largest = "no disagreement measured"
        else:
            largest = f"{self.largest[3]:+.2f} m at {self.largest[1]:.1f} s ({self.largest[2]})"
        resets = "; ".join(f"{name} to {value} at {seconds:.1f} s ({stage})" for seconds, stage, name, value in self.resets) or "none"
        return (f"HEIGHT REPORT: PX4's use of the bottom laser {changes}. Largest height disagreement (PX4 minus "
                f"camera tracking, after the launch baseline): {largest}. Height gate: {self.gate_result}. "
                f"Resets: {resets}.")
