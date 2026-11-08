"""Decides, once per control cycle, whether the run may carry on, must hold still, must stop and
land, or has already been taken away from our program.

REAL DRONE CODE, also used unchanged in the simulator. It is plain arithmetic, with no ROS or
PX4, so every rule can be tested on its own (test/test_run_safety.py). flight_steps.py gathers
the drone's reports into a DroneCondition ten times a second, asks RunSafety.check what to do,
and does it.

The checks run in every stage where the drone is in the air: the climb, the route, the settle
above the pad, and the landing's descent until touchdown. During the descent the drone is
already landing, so flight_steps.py only acts on a height problem there: the rest of the
descent finds the floor with the laser's raw reading.

The rules, in the order they are checked
----------------------------------------
Rule 5 (PX4 trusts camera tracking) is checked right after rule 2. Rule 4 (the height) is checked
before rule 3 (camera tracking), so it keeps running while the drone holds still waiting for
camera tracking.
1. Control is no longer ours. "Offboard" is the PX4 mode in which the drone follows our
   targets. If it has ended (the teammate flipped the mode switch, PX4 failed safe, or the
   motors were switched off), our targets do nothing, so we stop sending onward targets, write
   down why, and finish. "Disarmed" means the motors are not allowed to spin; that also ends it.
2. A touch. touch_detector.py flags a sudden tilt or spin. The team decided on 2026-11-07 that
   any touch the drone can notice stops the run, so the drone lands where it is.
3. Camera tracking is late or says it is lost. Camera tracking is the program that works out
   the drone's position from its cameras; it is the drone's only sideways position. If its last
   report is older than CAMERA_TRACKING_LONGEST_SILENCE_SECONDS, or it reports that it has lost
   track, the drone holds still (it keeps its last target and the route does not move on).
   If that lasts longer than LONGEST_HOLD_SECONDS, the run stops and the drone lands.
4. PX4's height has gone wrong. PX4 is the flight controller's software, and drone 16's PX4
   uses the bottom laser as its main height source. Camera tracking's height is always compared
   with PX4's (height_check.py's CameraHeightCheck, with a loose 30 cm limit, because camera
   height drifts), while camera tracking is reporting normally. On top of that, one of two
   forms of a closer check runs:
     - With the laser's raw reading (/fmu/out/distance_sensor, which the simulator has),
       height_check.py compares PX4's height with it over clear floor. If they disagree, PX4 has
       probably stopped using the laser.
     - Without it (the real drone does not publish that topic, checked on 2026-11-07),
       LaserUseCheck below watches PX4's own report of whether it uses the laser for height
       (estimator_status_flags). If PX4 reports that the laser is faulty for
       LASER_UNUSED_LONGEST_SECONDS while the drone is above AIRBORNE_HEIGHT_METRES, the check
       fires. If PX4 reports only that it has stopped using the laser, the flight goes on
       (the team decided this on 2026-11-08): the records get a warning, and the camera height
       comparison above uses a tighter limit from then on.
   When a check fires, the drone lands where it is, and the landing does not trust PX4's height.

   Why a stopped laser no longer stops the run: on 2026-11-08 PX4 on drone 16 stopped using the
   laser in mid-flight because of its own "range kinematic consistency" check, while the laser
   still worked. PX4's height then still agreed with camera tracking's within about 8 cm, but the
   old rule landed the drone halfway back from the out-and-back flight. PX4 did not start to use
   the laser again before the end of that flight.

5. PX4 no longer trusts camera tracking. PX4 works out the drone's position and heading itself,
   from camera tracking's reports. On drone 16 those reports often arrive 200 to 400 ms late
   instead of the expected 60 ms (measured 2026-11-08). PX4 then stops using them, for position
   or for heading. Without the camera, PX4 falls back to the compass for heading. Indoors the
   compass is disturbed and points 135 to 180 degrees away from the camera's heading. PX4's whole
   idea of "forward" then swings round, and it pushes the drone the wrong way: the 06:15 flight
   slid 0.9 m and hit a wall. CameraTrustCheck below watches for this. If PX4's heading and camera
   tracking's heading disagree by more than HEADING_GAP_LIMIT_DEGREES for
   HEADING_GAP_LONGEST_SECONDS, or PX4 has not used camera tracking's position for
   CAMERA_POSITION_UNUSED_LONGEST_SECONDS, the drone lands where it is. Holding still would not
   help: the hold point itself is in the wrong place. The landing sends PX4 only a downward speed
   with no sideways part, which stays right even when PX4's idea of direction is wrong. This rule
   runs from the moment our program has control, even on the floor (CameraTrustCheck says why).

Separately, check_target asks whether a target is inside the course, before it is sent. A target
outside the course (beyond the safety net, or above the wall tops) is refused, which stops the
run: something has gone badly wrong with the route or the frames, and flying on would be worse.

What it does not do: send anything, land, or read any reports. flight_steps.py does those.
"""
from dataclasses import dataclass

import numpy as np

from team_nav.drone_dimensions import DRONE_HALF_HEIGHT_METRES, DRONE_RADIUS_METRES
from team_nav.height_check import (CAMERA_HEIGHT_GAP_LIMIT_METRES, CAMERA_HEIGHT_GAP_LIMIT_WITHOUT_LASER_METRES,
                                   CameraHeightCheck, HeightCheck)
from team_nav.touch_detector import TouchDetector

# Measured on drone 16 on 2026-11-08 (the out-and-back flight's recorder, 24330 reports).
# Camera tracking reports about 30 times a second on average, but the reports arrive in bursts:
# 782 gaps were longer than 0.15 s, 1 in 100 gaps was longer than 0.29 s, and the longest was
# 0.35 s. With the old limit of 0.2 s the drone held still dozens of times in each flight for
# no real problem. 0.5 s is above every gap that we saw, and our checks run every 0.1 s, so a
# real silence is still found within 0.6 s.
CAMERA_TRACKING_LONGEST_SILENCE_SECONDS = 0.5

# Our own choice. A short gap in camera tracking is worth waiting out. After this long the drone
# has been flying on PX4's guess alone for too long, and landing is safer than waiting.
LONGEST_HOLD_SECONDS = 5.0

# Our own choice. The route follower keeps its target at most 0.65 m ahead of the drone
# (route_follower.py). A target much further away means the frames or the route have gone wrong,
# and PX4 would fly there fast (about 0.77 m/s per metre of distance, section 9).
FARTHEST_TARGET_FROM_DRONE_METRES = 1.2

# Our own choice. Close to the floor PX4 may legitimately use the air pressure sensor instead of
# the laser: on 2026-11-07 drone 16, sitting on the floor, was not using the laser for height.
# So the "no raw laser" check only counts above this height, measured by camera tracking.
AIRBORNE_HEIGHT_METRES = 0.3

# Our own choice, the same as height_check.py's HEIGHT_GAP_LONGEST_SECONDS. PX4 reports its
# estimator flags about twice a second on drone 16, so this is two reports in a row.
LASER_UNUSED_LONGEST_SECONDS = 1.0

# Measured on drone 16 on 2026-11-08 (four flights, 34000 reports): while PX4 used camera
# tracking for heading, its heading and the camera's differed by at most 2 to 11 degrees in 95 of
# 100 reports. While PX4 used the compass, they differed by 135 to 180 degrees. 20 degrees is well
# above the first and well below the second.
HEADING_GAP_LIMIT_DEGREES = 20.0

# Our own choice. A gap this short can be a single late report; the swing to the compass lasts
# much longer. At 30 degrees a second of turning (live_flight.py turns at 15), even a late report
# moves the gap by only 9 degrees in 0.3 s.
HEADING_GAP_LONGEST_SECONDS = 0.3

# Our own choice. Without camera tracking's position, PX4 guesses from its motion sensors alone,
# and on 2026-11-08 the drone slid 0.3 to 0.5 m within 2 s of that. PX4 reports its estimator
# flags about twice a second, so this is two reports in a row.
CAMERA_POSITION_UNUSED_LONGEST_SECONDS = 1.0

# The two laser problems that LaserUseCheck reports.
LASER_FAULTY = "PX4 reports that the bottom laser is faulty"
LASER_STOPPED = "PX4 reports that it has stopped using the bottom laser for height"

# What the flight program must do next.
CARRY_ON = "carry on"
HOLD = "hold still"
LAND_HERE = "stop the run and land here"
CONTROL_LOST = "control is no longer ours"


class TargetRefused(Exception):
    """A target failed check_target. The message says why, in full sentences."""


@dataclass
class DroneCondition:
    """Everything the checks need, gathered at one moment. A value is None when its report has
    not arrived."""
    now_seconds: float
    control_is_ours: bool
    is_armed: bool
    camera_tracking_age_seconds: float
    camera_tracking_says_lost: bool
    position_on_course_metres: object        # (east, north, height), or None.
    tilt_degrees: object
    rotation_degrees_per_second: object
    px4_height_risen_metres: object          # How far PX4 thinks the drone has risen since the launch mark.
    laser_height_risen_metres: object        # The same, from the bottom laser's raw reading.
    raw_laser_available: bool = True         # False on the real drone, which doesn't publish the raw reading.
    px4_uses_laser_for_height: object = None # PX4's own flag (cs_rng_hgt), or None if not reported recently.
    px4_says_laser_is_faulty: object = None  # PX4's own flag (cs_rng_fault), or None if not reported recently.
    # How much higher than wanted the flight program asks PX4 to fly, to make up for PX4's height
    # going wrong (height_keeper.py). The height checks expect PX4 to be off by this much.
    height_correction_metres: float = 0.0
    px4_uses_camera_position: object = None  # PX4's own flag (cs_ev_pos), or None if not reported recently.
    # How far PX4's heading has turned away from camera tracking's since the launch mark, in
    # degrees, or None if either is missing. 0 while PX4 follows the camera's heading.
    px4_heading_gap_degrees: object = None


@dataclass
class SafetyDecision:
    action: str                              # CARRY_ON, HOLD, LAND_HERE or CONTROL_LOST.
    reason: str = ""                         # A full sentence, empty for CARRY_ON.
    land_using_laser: bool = False           # True if PX4's height can't be trusted for the landing.
    warning: str = ""                        # A sentence for the records, given once. The flight goes on.


class RunSafety:

    def __init__(self, course_map, other_possible_boulders=()):
        """course_map is the drone's map. other_possible_boulders are boulders from other maps of
        the same course, which the bottom laser may see (height_check.py explains why)."""
        self.course_map = course_map
        self.touch_detector = TouchDetector()
        self.height_check = HeightCheck(course_map, other_possible_boulders)
        self.laser_use_check = LaserUseCheck()
        self.camera_height_check = CameraHeightCheck()
        self.camera_trust_check = CameraTrustCheck()
        # When the current hold began, or None if the drone isn't holding.
        self.hold_began_at_seconds = None

    def check(self, condition):
        """Returns the SafetyDecision for this moment. Call it once per control cycle."""
        if not condition.control_is_ours or not condition.is_armed:
            return SafetyDecision(CONTROL_LOST, _why_control_is_lost(condition))

        touch = self._check_for_touch(condition)
        if touch is not None:
            return SafetyDecision(LAND_HERE, touch)

        distrust = self.camera_trust_check.check(condition)
        if distrust is not None:
            return SafetyDecision(LAND_HERE, distrust)

        # The height check runs before the camera tracking check, so it keeps running while the
        # drone holds still for camera tracking. A wrong height is worse than a short hold: if both
        # happen, the drone lands.
        height_problem = self._check_height(condition)
        if height_problem is not None:
            return SafetyDecision(LAND_HERE, height_problem, land_using_laser=True)
        warning = self.laser_use_check.take_new_warning()

        tracking_problem = _camera_tracking_problem(condition)
        if tracking_problem is not None:
            decision = self._hold_or_give_up(condition.now_seconds, tracking_problem)
            decision.warning = warning
            return decision
        self.hold_began_at_seconds = None
        return SafetyDecision(CARRY_ON, warning=warning)

    def check_target(self, target_on_course_metres, drone_on_course_metres=None):
        """Raises TargetRefused if the target is outside the course or too far from the drone.
        Positions are (east, north, height) on the course map."""
        target = np.asarray(target_on_course_metres, dtype=float)
        if not np.all(np.isfinite(target)):
            raise TargetRefused(f"The target {target} is not a real number. This is a bug in the flight program.")
        self._refuse_if_outside_the_course(target)
        if drone_on_course_metres is not None:
            _refuse_if_far_from_the_drone(target, np.asarray(drone_on_course_metres, dtype=float))

    # ---- The individual rules -------------------------------------------------------------------

    def _check_for_touch(self, condition):
        if condition.tilt_degrees is None or condition.rotation_degrees_per_second is None:
            return None
        return self.touch_detector.check(condition.now_seconds, condition.tilt_degrees, condition.rotation_degrees_per_second)

    def _hold_or_give_up(self, now_seconds, tracking_problem):
        if self.hold_began_at_seconds is None:
            self.hold_began_at_seconds = now_seconds
        seconds_holding = now_seconds - self.hold_began_at_seconds
        if seconds_holding <= LONGEST_HOLD_SECONDS:
            return SafetyDecision(HOLD, tracking_problem)
        return SafetyDecision(
            LAND_HERE,
            f"{tracking_problem} The drone has held still for {seconds_holding:.1f} seconds waiting for it, longer "
            f"than the {LONGEST_HOLD_SECONDS:.0f} seconds we allow, so it lands where it is.",
            land_using_laser=True,
        )

    def _check_height(self, condition):
        # Both comparisons run every cycle, so each keeps its own timer up to date.
        camera_problem = self._compare_with_camera_height(condition)
        laser_problem = self._compare_with_the_laser(condition)
        if camera_problem is not None:
            return camera_problem
        return laser_problem

    def _compare_with_the_laser(self, condition):
        """The raw-laser comparison where the raw reading exists, else PX4's own laser flags."""
        if not condition.raw_laser_available:
            return self.laser_use_check.check(condition)
        needed = (condition.position_on_course_metres, condition.px4_height_risen_metres, condition.laser_height_risen_metres)
        for value in needed:
            if value is None:
                return None
        return self.height_check.check(condition.now_seconds, condition.position_on_course_metres,
                                       _px4_height_less_correction(condition), condition.laser_height_risen_metres)

    def _compare_with_camera_height(self, condition):
        """Only while camera tracking reports normally: a late or lost camera height proves nothing.
        After PX4 has stopped using the laser, the tighter limit applies."""
        tracking_is_normal = _camera_tracking_problem(condition) is None
        have_both = condition.position_on_course_metres is not None and condition.px4_height_risen_metres is not None
        if not (tracking_is_normal and have_both):
            self.camera_height_check.gap_began_at_seconds = None
            return None
        limit_metres = CAMERA_HEIGHT_GAP_LIMIT_METRES
        if self.laser_use_check.laser_has_stopped:
            limit_metres = CAMERA_HEIGHT_GAP_LIMIT_WITHOUT_LASER_METRES
        return self.camera_height_check.check(condition.now_seconds, _px4_height_less_correction(condition),
                                              condition.position_on_course_metres[2], limit_metres)

    def _refuse_if_outside_the_course(self, target):
        # The whole drone, not just its centre, must stay inside the safety net.
        lowest_allowed = self.course_map.lowest_corner_metres + DRONE_RADIUS_METRES
        highest_allowed = self.course_map.highest_corner_metres - DRONE_RADIUS_METRES
        inside_sideways = np.all(target[:2] >= lowest_allowed) and np.all(target[:2] <= highest_allowed)
        if not inside_sideways:
            raise TargetRefused(
                f"The target ({target[0]:+.2f} m east, {target[1]:+.2f} m north) would put part of the drone outside "
                f"the course's safety net. The route or the start heading must be wrong, so the run stops."
            )
        # From the rulebook: the drone must stay below the wall tops.
        highest_height_metres = self.course_map.wall_top_height_metres - DRONE_HALF_HEIGHT_METRES
        if not 0.0 <= target[2] <= highest_height_metres:
            raise TargetRefused(
                f"The target height {target[2]:.2f} m is outside the allowed 0 to {highest_height_metres:.2f} m "
                f"(the drone's top must stay below the {self.course_map.wall_top_height_metres:.2f} m wall tops). "
                f"The route must be wrong, so the run stops."
            )


def _px4_height_less_correction(condition):
    """PX4's height, less the height correction we asked for on purpose. If PX4's height has
    drifted by exactly the correction, this equals the true height."""
    return condition.px4_height_risen_metres - condition.height_correction_metres


class LaserUseCheck:
    """The height check for when the laser's raw reading is not available. It watches PX4's own
    laser flags while the drone is above AIRBORNE_HEIGHT_METRES, and each problem must last
    LASER_UNUSED_LONGEST_SECONDS to count:
      - PX4 reports that the laser is faulty: the check fires, and the run stops.
      - PX4 reports that it has stopped using the laser for height: the flight goes on. The
        check sets laser_has_stopped, which tightens the camera height comparison, and gives
        one warning for the records (take_new_warning).

    "Stopped" means PX4 was using the laser at some moment while airborne, and then wasn't. If
    PX4 never uses the laser at all after takeoff, this check stays quiet, and the flight program's
    records show it (px4_uses_laser_for_height in drone_condition.csv). laser_has_stopped stays
    set for the rest of the run, because on 2026-11-08 PX4 did not start to use the laser again."""

    def __init__(self):
        self.laser_was_used_while_airborne = False
        self.problem_began_at_seconds = None
        self.laser_has_stopped = False
        self.new_warning = ""

    def check(self, condition):
        """Returns None while all is well, or a sentence saying why the run must stop."""
        position = condition.position_on_course_metres
        if position is None or position[2] < AIRBORNE_HEIGHT_METRES:
            self.problem_began_at_seconds = None
            return None
        if condition.px4_uses_laser_for_height:
            self.laser_was_used_while_airborne = True
        problem = self._problem_now(condition)
        if problem is None:
            self.problem_began_at_seconds = None
            return None
        if self.problem_began_at_seconds is None:
            self.problem_began_at_seconds = condition.now_seconds
        if condition.now_seconds - self.problem_began_at_seconds < LASER_UNUSED_LONGEST_SECONDS:
            return None
        if problem == LASER_STOPPED:
            self._note_that_the_laser_has_stopped(position)
            return None
        return (f"{problem} for {LASER_UNUSED_LONGEST_SECONDS:.0f} second, {position[2]:.2f} m above the floor. "
                f"Its height now comes from the air pressure sensor, which drifts (docs/ARCHITECTURE.md, "
                f"section 7), so its height can't be trusted.")

    def take_new_warning(self):
        """The warning sentence, once, or an empty string."""
        warning = self.new_warning
        self.new_warning = ""
        return warning

    def _note_that_the_laser_has_stopped(self, position):
        if self.laser_has_stopped:
            return
        self.laser_has_stopped = True
        self.new_warning = (
            f"{LASER_STOPPED} for {LASER_UNUSED_LONGEST_SECONDS:.0f} second, {position[2]:.2f} m above the floor. "
            f"Its height now comes from the air pressure sensor, which drifts. The flight goes on, but from now on "
            f"the drone lands if PX4's height and camera tracking's height differ by more than "
            f"{CAMERA_HEIGHT_GAP_LIMIT_WITHOUT_LASER_METRES:.2f} m.")

    def _problem_now(self, condition):
        if condition.px4_says_laser_is_faulty:
            return LASER_FAULTY
        stopped_using = condition.px4_uses_laser_for_height is False and self.laser_was_used_while_airborne
        if stopped_using:
            return LASER_STOPPED
        return None


class CameraTrustCheck:
    """Rule 5: notices when PX4 stops trusting camera tracking. Each problem must last its own
    time limit to count.

    It runs from the moment our program has control, even with the drone still on the floor: in
    the 05:42 and 06:15 flights of 2026-11-08 the swing to the compass (about 165 degrees) came
    right after arming and in the first 0.3 m of the climb, and the 06:15 slide began there. A
    landing that starts on the floor only waits for the motors to stop, which is safe."""

    def __init__(self):
        self.heading_gap_began_at_seconds = None
        self.position_unused_began_at_seconds = None

    def check(self, condition):
        """Returns None while all is well, or a sentence saying why the run must stop."""
        heading_problem = self._check_heading(condition)
        if heading_problem is not None:
            return heading_problem
        return self._check_position_use(condition)

    def _check_heading(self, condition):
        gap = condition.px4_heading_gap_degrees
        if gap is None or abs(gap) <= HEADING_GAP_LIMIT_DEGREES:
            self.heading_gap_began_at_seconds = None
            return None
        if self.heading_gap_began_at_seconds is None:
            self.heading_gap_began_at_seconds = condition.now_seconds
        if condition.now_seconds - self.heading_gap_began_at_seconds < HEADING_GAP_LONGEST_SECONDS:
            return None
        return (f"PX4's heading differs from camera tracking's by {abs(gap):.0f} degrees, more than the "
                f"{HEADING_GAP_LIMIT_DEGREES:.0f} we allow. PX4 has probably switched to the compass, which is "
                f"disturbed indoors, so it will push the drone the wrong way. The drone lands where it is.")

    def _check_position_use(self, condition):
        if condition.px4_uses_camera_position is not False:
            self.position_unused_began_at_seconds = None
            return None
        if self.position_unused_began_at_seconds is None:
            self.position_unused_began_at_seconds = condition.now_seconds
        seconds_unused = condition.now_seconds - self.position_unused_began_at_seconds
        if seconds_unused < CAMERA_POSITION_UNUSED_LONGEST_SECONDS:
            return None
        return (f"PX4 has not used camera tracking's position for {seconds_unused:.1f} s, so it is guessing the "
                f"drone's position from its motion sensors alone, and the drone drifts. Camera tracking's reports "
                f"are probably arriving too late for PX4. The drone lands where it is.")


def _refuse_if_far_from_the_drone(target, drone):
    sideways_distance_metres = float(np.linalg.norm(target[:2] - drone[:2]))
    if sideways_distance_metres > FARTHEST_TARGET_FROM_DRONE_METRES:
        raise TargetRefused(
            f"The next target is {sideways_distance_metres:.2f} m from the drone, more than the "
            f"{FARTHEST_TARGET_FROM_DRONE_METRES:.1f} m we allow, so PX4 would fly there too fast. The drone has "
            f"probably been pushed off course or camera tracking has jumped, so the run stops."
        )


def _why_control_is_lost(condition):
    if not condition.is_armed:
        return "The motors have been switched off (the drone is disarmed), so our targets can do nothing."
    return ("PX4 has left Offboard mode, so it no longer follows our targets. A teammate may have flipped the mode "
            "switch, or PX4 failed safe. Our program stops sending targets.")


def _camera_tracking_problem(condition):
    if condition.camera_tracking_says_lost:
        return "Camera tracking reports that it has lost track, so the drone's position is unknown. Holding still."
    if condition.camera_tracking_age_seconds > CAMERA_TRACKING_LONGEST_SILENCE_SECONDS:
        return (f"Camera tracking's last position report is {condition.camera_tracking_age_seconds:.2f} s old, more than "
                f"the {CAMERA_TRACKING_LONGEST_SILENCE_SECONDS:.1f} s we allow. Holding still.")
    return None
