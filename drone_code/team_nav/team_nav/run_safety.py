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
   the drone's position from its cameras; it is the drone's only sideways position. A report's
   age is measured from the moment its camera pictures were taken, not from when it arrived,
   because on drone 16 reports arrive on time but carry old pictures. If the latest report is
   older than CAMERA_REPORT_OLDEST_AGE_SECONDS, or camera tracking reports that it has lost
   track, the drone holds still at once (it keeps its last target and the route does not move
   on). It carries on after the reports have been usable again (not older than that limit) for
   FRESH_REPORTS_BEFORE_RESUMING_SECONDS. The run stops and
   the drone lands only if the reports stay too old (or lost) for CAMERA_PROBLEM_LONGEST_SECONDS
   without a break.
4. PX4's height has gone wrong. PX4 is the flight controller's software, and drone 16's PX4
   uses the bottom laser as its main height source. Camera tracking's height is always compared
   with PX4's (height_check.py's CameraHeightCheck, with a loose 30 cm limit, because camera
   height drifts, and widened while the drone climbs or descends, because camera tracking's
   height arrives late), while camera tracking is reporting normally. On top of that, one of two
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
   from camera tracking's reports. When a report is too late, PX4 stops using it for a moment,
   for position or for heading. Before 2026-11-08 PX4 then fell back to the compass for heading,
   which indoors points 135 to 180 degrees away, and pushed the drone the wrong way (the 06:15
   flight slid 0.9 m and hit a wall). The compass is now off (EKF2_MAG_TYPE 5, changed on
   2026-11-08), but PX4 can still drop camera tracking for a moment. CameraTrustCheck watches
   for this. These are cheap to wait out, so the reaction comes in two steps:
     - At once, on one reading: if PX4's heading and camera tracking's heading disagree by more
       than HEADING_GAP_LIMIT_DEGREES, or PX4 says it is not using camera tracking's position,
       the drone holds still. For a heading gap the target's heading is PX4's own current
       heading, so PX4 starts no turn.
     - Only if it lasts: a heading gap for HEADING_GAP_LONGEST_SECONDS, or the position unused
       for CAMERA_POSITION_UNUSED_LONGEST_SECONDS, lands the drone where it is. The landing
       sends PX4 only a downward speed, which stays right even when PX4's idea of direction is
       wrong.
   Why not land at once: in the 10:20 turn test on 2026-11-08 the old 0.3 s heading rule landed
   the drone 0.4 s into a heading episode that PX4 ended by itself about 1 s later. This rule
   runs from the moment our program has control, even on the floor (CameraTrustCheck says why).

Separately, check_target looks at each target before it is sent. A target outside the course
(beyond the safety net, or above the wall tops) is refused, which stops the run: something has
gone badly wrong with the route or the frames, and flying on would be worse. A target farther
than FARTHEST_TARGET_FROM_DRONE_METRES from the drone is pulled in to that distance and sent;
the run stops only if targets stay that far for FAR_TARGET_LONGEST_SECONDS.

The rule of thumb behind these choices: cheap actions (hold still, keep the last target, start
no turn) may happen on one reading. Expensive ones (land, end the run) need the problem to last
(persistence.py). True emergencies (a touch, control lost) stay immediate.

What it does not do: send anything, land, or read any reports. flight_steps.py does those.
"""
from dataclasses import dataclass

import numpy as np

from team_nav.drone_dimensions import DRONE_HALF_HEIGHT_METRES, DRONE_RADIUS_METRES
from team_nav.height_check import (CAMERA_HEIGHT_GAP_LIMIT_METRES, CAMERA_HEIGHT_GAP_LIMIT_WITHOUT_LASER_METRES,
                                   CameraHeightCheck, HeightCheck)
from team_nav.persistence import ConditionHeldFor
from team_nav.touch_detector import TouchDetector

# Measured on drone 16 on 2026-11-08: a camera report's age (now minus its picture time) was
# 60 ms in the middle and 75 ms for 95 in 100 reports in clean periods; in bad periods, which
# last 1.3 to 4.6 minutes, reports were up to 0.43 s old. From PX4's settings on drone 16
# (EKF2_DELAY_MAX 350 ms, plus the time PX4 needs to pass the report on): PX4 may stop using a
# report that is more than about 0.45 s old. So up to that age the position is still usable.
CAMERA_REPORT_OLDEST_AGE_SECONDS = 0.45
# The same limit under its old name, because course_run.py imports it.
CAMERA_TRACKING_LONGEST_SILENCE_SECONDS = CAMERA_REPORT_OLDEST_AGE_SECONDS

# Our own choice. About 15 reports in a row of usable age; long enough for PX4 to be using the
# camera again. A stricter "fresh" age (0.15 s) was tried first, but replaying drone 16's bad
# periods (most reports 0.15 to 0.43 s old) gave holds of 25 to 66 s.
FRESH_REPORTS_BEFORE_RESUMING_SECONDS = 0.5
# Our own choice. Late reports in bad periods were at most 0.43 s old, so a hold normally ends
# within a second. Three seconds of reports that are all too old means camera tracking has
# really stopped, and landing is safer than waiting.
CAMERA_PROBLEM_LONGEST_SECONDS = 3.0

# Our own choice. The route follower keeps its target at most 0.65 m ahead of the drone
# (route_follower.py). A target much further away means the frames or the route have gone wrong,
# and PX4 would fly there fast (about 0.77 m/s per metre of distance, section 9). Such a target is
# pulled in to this distance instead.
FARTHEST_TARGET_FROM_DRONE_METRES = 1.2
# Our own choice: three cycles at 10 per second. One far target can come from a single camera
# tracking jump (10:44 on 2026-11-08: about 1.0 m in one 0.1 s report); targets that stay far
# mean the drone really is off course.
FAR_TARGET_LONGEST_SECONDS = 0.3

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

# Our own choice. Measured on drone 16 on 2026-11-08: PX4 went back to camera tracking's heading
# 0.6 to 1.1 s after dropping it. The drone holds still meanwhile, so waiting costs little. A gap
# that lasts 3 s is not one of those moments, and the hold point itself may then be wrong.
HEADING_GAP_LONGEST_SECONDS = 3.0

# Our own choice. Measured on drone 16 on 2026-11-08: PX4 stopped using camera tracking's position
# for 0.13 to 0.46 s at a time, and without it the drone slid 0.3 to 0.5 m within 2 s. PX4 reports
# its estimator flags only once or twice a second, so one stale "not used" report could look like
# a whole second. 2 s is at least two reports, and the drone holds still meanwhile.
CAMERA_POSITION_UNUSED_LONGEST_SECONDS = 2.0

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
    # How old camera tracking's latest report is, counted from when its camera pictures were taken.
    camera_report_age_seconds: float
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
    # PX4's vertical speed, upward positive, or None. Camera tracking's height lags PX4's while the
    # drone climbs or descends, so the height comparison allows for it (height_check.py).
    px4_upward_speed_metres_per_second: object = None


@dataclass
class SafetyDecision:
    action: str                              # CARRY_ON, HOLD, LAND_HERE or CONTROL_LOST.
    reason: str = ""                         # A full sentence, empty for CARRY_ON.
    land_using_laser: bool = False           # True if PX4's height can't be trusted for the landing.
    warning: str = ""                        # A sentence for the records, given once. The flight goes on.
    # For a HOLD: True if the held target should point the nose where PX4 thinks it points now,
    # so that PX4 starts no turn while its heading and camera tracking's disagree.
    hold_with_px4_heading: bool = False


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
        self.camera_problem_lasted = ConditionHeldFor(CAMERA_PROBLEM_LONGEST_SECONDS)
        self.camera_reports_fresh_for_long_enough = ConditionHeldFor(FRESH_REPORTS_BEFORE_RESUMING_SECONDS)
        self.holding_for_camera_tracking = False
        self.target_far_from_drone = ConditionHeldFor(FAR_TARGET_LONGEST_SECONDS)

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

        camera_problem = _camera_tracking_problem(condition)
        if self.camera_problem_lasted.update(camera_problem is not None, condition.now_seconds):
            return SafetyDecision(LAND_HERE, f"{camera_problem} This has lasted {CAMERA_PROBLEM_LONGEST_SECONDS:.0f} "
                                             f"seconds without a break, so the drone lands where it is.",
                                  land_using_laser=True)
        decision = self._hold_or_carry_on(condition, camera_problem, warning)
        if decision.action == HOLD:
            # No onward target is checked during a hold, so a far target before the hold and one
            # after it are not "in a row".
            self.target_far_from_drone.reset()
        return decision

    def check_target(self, target_on_course_metres, drone_on_course_metres, now_seconds):
        """Returns the target to send, as (east, north, height) on the course map. A target too far
        from the drone is pulled in to FARTHEST_TARGET_FROM_DRONE_METRES. Raises TargetRefused if
        the target is outside the course, or if targets have been too far for
        FAR_TARGET_LONGEST_SECONDS. drone_on_course_metres may be None (no distance check)."""
        target = np.asarray(target_on_course_metres, dtype=float)
        if not np.all(np.isfinite(target)):
            raise TargetRefused(f"The target {target} is not a real number. This is a bug in the flight program.")
        if drone_on_course_metres is not None:
            target = self._pulled_in_if_far(target, np.asarray(drone_on_course_metres, dtype=float), now_seconds)
        self._refuse_if_outside_the_course(target)
        return target

    # ---- The individual rules -------------------------------------------------------------------

    def _check_for_touch(self, condition):
        if condition.tilt_degrees is None or condition.rotation_degrees_per_second is None:
            return None
        return self.touch_detector.check(condition.now_seconds, condition.tilt_degrees, condition.rotation_degrees_per_second)

    def _hold_or_carry_on(self, condition, camera_problem, warning):
        """The cheap reactions, on one reading: hold still while anything makes the drone's
        position or heading doubtful."""
        # Asked first, because it must see every cycle to time the fresh reports.
        camera_hold_reason = self._camera_hold_reason(condition, camera_problem)
        if _heading_gap_is_too_big(condition):
            return SafetyDecision(HOLD, _heading_gap_hold_reason(condition), warning=warning, hold_with_px4_heading=True)
        if condition.px4_uses_camera_position is False:
            return SafetyDecision(HOLD, "PX4 says it is not using camera tracking's position at the moment, so it is "
                                        "guessing the drone's position. Holding still.", warning=warning)
        if camera_hold_reason is not None:
            return SafetyDecision(HOLD, camera_hold_reason, warning=warning)
        return SafetyDecision(CARRY_ON, warning=warning)

    def _camera_hold_reason(self, condition, camera_problem):
        """A hold starts on one late or lost report, and ends only after
        FRESH_REPORTS_BEFORE_RESUMING_SECONDS of usable reports. Returns None when not holding."""
        report_is_usable = camera_problem is None
        usable_for_long_enough = self.camera_reports_fresh_for_long_enough.update(report_is_usable, condition.now_seconds)
        if camera_problem is not None:
            self.holding_for_camera_tracking = True
            return camera_problem + " Holding still."
        if self.holding_for_camera_tracking and not usable_for_long_enough:
            return (f"Camera tracking's latest report is {condition.camera_report_age_seconds:.2f} s old. Holding still "
                    f"until its reports have been younger than {CAMERA_REPORT_OLDEST_AGE_SECONDS:.2f} s for "
                    f"{FRESH_REPORTS_BEFORE_RESUMING_SECONDS:.1f} s.")
        self.holding_for_camera_tracking = False
        return None

    def _pulled_in_if_far(self, target, drone, now_seconds):
        """The target, or, if it is too far from the drone sideways, the point on the way to it at
        FARTHEST_TARGET_FROM_DRONE_METRES. The height is kept."""
        sideways_offset = target[:2] - drone[:2]
        sideways_distance_metres = float(np.linalg.norm(sideways_offset))
        is_far = sideways_distance_metres > FARTHEST_TARGET_FROM_DRONE_METRES
        if self.target_far_from_drone.update(is_far, now_seconds):
            raise TargetRefused(
                f"The targets have been more than {FARTHEST_TARGET_FROM_DRONE_METRES:.1f} m from the drone for "
                f"{FAR_TARGET_LONGEST_SECONDS:.1f} s (now {sideways_distance_metres:.2f} m), so PX4 would fly there "
                f"too fast. The drone has probably been pushed off course or camera tracking has jumped, so the run "
                f"stops."
            )
        if not is_far:
            return target
        pulled_in = target.copy()
        pulled_in[:2] = drone[:2] + sideways_offset * (FARTHEST_TARGET_FROM_DRONE_METRES / sideways_distance_metres)
        return pulled_in

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
            self.camera_height_check.reset()
            return None
        limit_metres = CAMERA_HEIGHT_GAP_LIMIT_METRES
        if self.laser_use_check.laser_has_stopped:
            limit_metres = CAMERA_HEIGHT_GAP_LIMIT_WITHOUT_LASER_METRES
        return self.camera_height_check.check(condition.now_seconds, _px4_height_less_correction(condition),
                                              condition.position_on_course_metres[2], limit_metres,
                                              condition.px4_upward_speed_metres_per_second)

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
    """Rule 5: notices when PX4 has stopped trusting camera tracking for long enough to land.
    (RunSafety holds the drone still on the first reading; this class only decides the landing.)

    It runs from the moment our program has control, even with the drone still on the floor: in
    the 05:42 and 06:15 flights of 2026-11-08 the swing to the compass (about 165 degrees) came
    right after arming and in the first 0.3 m of the climb, and the 06:15 slide began there. A
    landing that starts on the floor only waits for the motors to stop, which is safe."""

    def __init__(self):
        self.heading_gap_lasted = ConditionHeldFor(HEADING_GAP_LONGEST_SECONDS)
        self.position_unused_lasted = ConditionHeldFor(CAMERA_POSITION_UNUSED_LONGEST_SECONDS)

    def check(self, condition):
        """Returns None while all is well, or a sentence saying why the run must stop."""
        now = condition.now_seconds
        heading_gap_has_lasted = self.heading_gap_lasted.update(_heading_gap_is_too_big(condition), now)
        position_unused_has_lasted = self.position_unused_lasted.update(condition.px4_uses_camera_position is False, now)
        if heading_gap_has_lasted:
            return (f"PX4's heading has differed from camera tracking's by more than {HEADING_GAP_LIMIT_DEGREES:.0f} "
                    f"degrees for {HEADING_GAP_LONGEST_SECONDS:.0f} seconds (now "
                    f"{abs(condition.px4_heading_gap_degrees):.0f}). PX4 may be using the compass, which is disturbed "
                    f"indoors, or no longer knows which way the drone points, so it may push the drone the wrong "
                    f"way. The drone lands where it is.")
        if position_unused_has_lasted:
            return (f"PX4 has not used camera tracking's position for {CAMERA_POSITION_UNUSED_LONGEST_SECONDS:.0f} "
                    f"seconds, so it is guessing the drone's position from its motion sensors alone, and the drone "
                    f"drifts. Camera tracking's reports are probably arriving too late for PX4. The drone lands "
                    f"where it is.")
        return None


def _heading_gap_is_too_big(condition):
    gap = condition.px4_heading_gap_degrees
    return gap is not None and abs(gap) > HEADING_GAP_LIMIT_DEGREES


def _heading_gap_hold_reason(condition):
    return (f"PX4's heading differs from camera tracking's by {abs(condition.px4_heading_gap_degrees):.0f} degrees, "
            f"more than the {HEADING_GAP_LIMIT_DEGREES:.0f} we allow. PX4 has probably stopped using camera "
            f"tracking's heading for a moment. Holding still, with the nose where PX4 thinks it points.")


def _why_control_is_lost(condition):
    if not condition.is_armed:
        return "The motors have been switched off (the drone is disarmed), so our targets can do nothing."
    return ("PX4 has left Offboard mode, so it no longer follows our targets. A teammate may have flipped the mode "
            "switch, or PX4 failed safe. Our program stops sending targets.")


def _camera_tracking_problem(condition):
    """A sentence if camera tracking's latest report is too old to use or says it is lost, else None."""
    if condition.camera_tracking_says_lost:
        return "Camera tracking reports that it has lost track, so the drone's position is unknown."
    if condition.camera_report_age_seconds > CAMERA_REPORT_OLDEST_AGE_SECONDS:
        return (f"Camera tracking's latest position report is {condition.camera_report_age_seconds:.2f} s old (from "
                f"its pictures' time), more than the {CAMERA_REPORT_OLDEST_AGE_SECONDS:.2f} s we allow.")
    return None
