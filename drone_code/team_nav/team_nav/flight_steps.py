"""The steps every real flight is made of: wait on the ground until the drone is ready, wait for
the teammate's start switch, climb, hold still, and land with the push-down step.

REAL DRONE CODE, also used unchanged in the simulator (through
simulator/test_flights/real_program_test.py, which only plays the teammate's part). course_run.py
builds the whole course flight from these steps; practice flights can build shorter ones.

The file has three parts:
  1. DroneLink: the connection to the drone. It reads the drone's reports (drone_reports.py, plus
     camera tracking's own reports), sends targets (target_sender.py), records the drone's
     condition and every target ten times a second (run_log.py), and keeps a steady 10 Hz rhythm.
  2. SafetyWatch: runs run_safety.py's checks once per cycle and turns each decision into an
     action: hold still, or raise RunMustStop or ControlIsNoLongerOurs for the program to handle.
     Given a watch, the steps run it in every airborne stage: the climb, holds, the route (in
     course_run.py) and a landing's descent until touchdown.
  3. The steps themselves, as plain functions that take a DroneLink.

Words used here
---------------
  - "Arming" gives the motors permission to spin. A teammate arms on the remote control; our
    code never does.
  - "Offboard" is the PX4 mode in which the drone follows our targets. The organizers'
    flight_interface asks PX4 for it once, when a teammate flips the mode switch to Position
    after arming (docs/ARCHITECTURE.md, section 8). We call that "control is ours".
  - "Camera tracking" (cuVSLAM) works out the drone's position from its cameras. It reports
    metres forward, left and up along the way the drone pointed when tracking started.
  - The "launch frame" uses the same directions, measured from where the drone sat on the launch
    mark just before the flight. Steps take positions in the launch frame, so they don't depend
    on where camera tracking happened to start.
  - PX4's z points DOWN, so climbing makes it smaller.

Targets never stop
------------------
Every step sends a target every cycle, because flight_interface only hands control to a program
that is already sending targets, and a speed target expires after 1 second. The only time nothing
is sent is while waiting to be ready on the ground, before we know where the drone is.

When a report is missing
------------------------
The flight needs only the organizers' summary, PX4's position and status, and camera tracking.
The laser's raw reading, PX4's tilt report and PX4's estimator flags make safety checks possible,
but the real drone may not publish them: on 2026-11-07 drone 16 had no raw laser reading. Once
the drone is ready, announce_which_checks_can_run says loudly which checks run in a weaker form,
and the program never waits for those reports.

What this file does not do: plan or follow the route (course_run.py and route_follower.py), or
decide what is safe (run_safety.py). It never arms the drone or changes PX4's mode.
"""
import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy

from team_nav.run_safety import CARRY_ON, CONTROL_LOST, HOLD, LAND_HERE, DroneCondition, TargetRefused
from team_nav.target_sender import POSE_TARGET_TOPIC, SPEED_TARGET_TOPIC, TargetSender

# ---- Rhythm ---------------------------------------------------------------------------------------

# Our own choice. flight_interface forgets a speed target after 1 second and passes targets to PX4
# 10 times a second, so 10 per second keeps up with it with plenty of margin.
CYCLES_PER_SECOND = 10.0
CYCLE_SECONDS = 1.0 / CYCLES_PER_SECOND

# Our own choice: how often a waiting step reminds the people what it is waiting for.
SECONDS_BETWEEN_REMINDERS = 5.0

# ---- Camera tracking's own reports ----------------------------------------------------------------

# From the organizers' code (flight_interface.py's default odometry_topic) and their cuVSLAM
# program (onboard/slam/cuvslam_lib_node.py).
CAMERA_TRACKING_POSITION_TOPIC = "/visual_slam/tracking/odometry"
CAMERA_TRACKING_STATUS_TOPIC = "/visual_slam/status"
# From the message definition (simulator/ros_packages/isaac_ros_visual_slam_interfaces, read from
# drone 16): vo_state 2 means camera tracking has failed.
CAMERA_TRACKING_LOST_STATE = 2

# ---- Being ready on the ground --------------------------------------------------------------------

# Our own choice. Every report the flight needs must be newer than this before we start. The
# organizers' summary arrives 5 times a second, the laser about 5 times a second on drone 16
# (section 7), and the rest faster.
READY_REPORT_LONGEST_AGE_SECONDS = 1.0
# From the simulator flights (simulator_test_flight.py waits this long): PX4's height estimate
# settles for a few seconds after it first becomes available.
READY_STEADY_SECONDS = 5.0
# Reports the flight cannot do without. "state" is the organizers' summary; the names are
# drone_reports.py's.
REPORTS_NEEDED_BEFORE_START = ["state", "px4_position", "px4_status"]
# Reports that make a safety check possible, but that the real drone may not publish. On
# 2026-11-07 drone 16 had no /fmu/out/distance_sensor (the laser's raw reading). Without one of
# these, its check runs in a weaker form or not at all, and the program says so loudly at the start.
#   laser             the laser's raw reading: the height check's comparison, the laser landing.
#   px4_odometry      PX4's tilt and turn rate: the touch detector. If it is missing, the touch
#                     detector uses px4_attitude (tilt) and motion_sensors (the raw gyro's turn
#                     rate) instead; drone 16 sends both.
#   estimator_flags   whether PX4 uses the laser: the height check without a raw reading.
OPTIONAL_REPORTS = ["laser", "px4_odometry", "px4_attitude", "motion_sensors", "estimator_flags"]
# Our own choice. PX4 sends its estimator flags about twice a second on drone 16, so a flag older
# than this is stale and not used.
ESTIMATOR_FLAGS_LONGEST_AGE_SECONDS = 2.0
# Our own choice. PX4 sends its position about 100 times a second on drone 16, so an older one
# means PX4's height can't be used for a landing.
PX4_POSITION_LONGEST_AGE_SECONDS = 0.5

# ---- Climbing and holding -------------------------------------------------------------------------

# Our own choices, the same as the simulator's test flights, where they worked.
HEIGHT_TOLERANCE_METRES = 0.10
SECONDS_TO_SETTLE = 2.0
LONGEST_CLIMB_SECONDS = 20.0

# ---- Landing (docs/ARCHITECTURE.md, section 7) ---------------------------------------------------

MEDIUM_DESCENT_METRES_PER_SECOND = 0.5
GENTLE_DESCENT_METRES_PER_SECOND = 0.35
PUSH_DOWN_METRES_PER_SECOND = 1.0
HEIGHT_TO_START_GENTLE_DESCENT_METRES = 0.30
# Touchdown: the height is this close to the launch mark's, and the drone is not moving down, for
# this long. Section 7's design, tested in the simulator.
TOUCHDOWN_HEIGHT_TOLERANCE_METRES = 0.03
TOUCHDOWN_LARGEST_DOWNWARD_SPEED_METRES_PER_SECOND = 0.05
TOUCHDOWN_SECONDS_REQUIRED = 0.3
# Our own choice, from section 7's "what if": if the drone stops moving down for this long during
# the gentle descent, it is sitting on something, or the height reading is stuck or missing, so
# it counts as touchdown whatever the height says. This only counts once the gentle descent has
# run long enough for the drone to be moving down.
STALLED_DESCENT_SECONDS = 1.0
GENTLE_DESCENT_BEFORE_A_STALL_COUNTS_SECONDS = 2.0
# Our own choice. When the landing has no height reading at all, it descends at the gentle speed
# until PX4 itself says it has landed, and never pushes down, because a push-down in the air would
# hit the floor at 1 m/s. In the simulator, without the push-down, PX4 switched the motors off about
# 9.7 s after touchdown (section 7).
# The three ways a landing can find the floor, best first.
LASER_RAW_READING = "the bottom laser's raw reading"
PX4_DISTANCE_TO_GROUND = "PX4's distance to the ground"
NO_HEIGHT_READING = "no height reading: gentle descent until PX4 says landed"

# Our own choice. 20 s of descent is at least 7 m, far more than we ever fly, so running out of
# time means a reading is wrong. We then carry on to the push-down step, which is harmless on the
# floor and only 1 m/s in the air.
LONGEST_DESCENT_STAGE_SECONDS = 20.0


class RunMustStop(Exception):
    """Something happened that ends the run; the drone must land where it is. The message says
    what, in full sentences. land_using_laser is True when PX4's height can't be trusted."""

    def __init__(self, reason, land_using_laser=False):
        super().__init__(reason)
        self.land_using_laser = land_using_laser


class LaunchPoseRefused(Exception):
    """The drone's pose on the launch mark is not acceptable, so the flight must not start. The
    message says why, in full sentences."""


class ControlIsNoLongerOurs(Exception):
    """PX4 has left Offboard mode or the motors are off, so our targets can do nothing."""


@dataclass
class LaunchReadings:
    """What the drone's reports said while it sat on the launch mark, just before the flight."""
    tracked_position_metres: np.ndarray      # Camera tracking's (forward, left, up).
    px4_z_metres: float                      # PX4's height as z, which points down.
    px4_distance_to_ground_metres: float     # PX4's laser-based distance to the floor.
    raw_laser_metres: float                  # The bottom laser's raw reading.


# =================================================================================================
# 1. The connection to the drone
# =================================================================================================

class DroneLink:
    """Everything a flight step needs to read the drone, send it targets and record the flight."""

    def __init__(self, node, run_log, ignore_raw_laser=False):
        """ignore_raw_laser makes the program behave as if the laser's raw reading did not
        exist, as on the real drone, to test that form of the checks in the simulator."""
        # Imported here, not at the top, so that the tests, which use a pretend drone, can import
        # this file on a computer without PX4's message definitions.
        from team_nav.drone_reports import DroneReports

        self.node = node
        self.log = run_log
        self.reports = DroneReports(node)
        self.targets = TargetSender(node, on_target_sent=self._record_target)
        self.launch = None                   # LaunchReadings, once remember_launch_mark has run.
        self.ignore_raw_laser = ignore_raw_laser
        # Decided once the drone is ready on the ground (announce_which_checks_can_run).
        self.raw_laser_available = False
        self.stage = "starting"              # Written next to every sample, to find stages later.
        self.next_cycle_seconds = time.monotonic()
        self._listen_to_camera_tracking()
        self.log.event("topics", "Topics in use: " + ", ".join(self.topic_names()), topics=self.topic_names())

    def _listen_to_camera_tracking(self):
        """drone_reports.py only has camera tracking's position from the organizers' summary,
        which arrives 5 times a second. To notice a 0.5 s silence we need its own reports."""
        from nav_msgs.msg import Odometry
        self.tracked_position_metres = None
        self.tracked_position_arrival_seconds = None
        self.camera_tracking_state = None
        self.node.create_subscription(Odometry, CAMERA_TRACKING_POSITION_TOPIC, self._on_tracked_position, 10)
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        except ImportError:
            # cuVSLAM's status message isn't installed. We can still notice silences.
            self.log.event("warning", f"Camera tracking's status message type isn't installed, so "
                                      f"{CAMERA_TRACKING_STATUS_TOPIC} is not read. Silences are still noticed.")
            return
        self.node.create_subscription(VisualSlamStatus, CAMERA_TRACKING_STATUS_TOPIC, self._on_tracking_status, 10)

    def _on_tracked_position(self, message):
        position = message.pose.pose.position
        self.tracked_position_metres = np.array([position.x, position.y, position.z], dtype=float)
        self.tracked_position_arrival_seconds = time.monotonic()

    def _on_tracking_status(self, message):
        self.camera_tracking_state = int(message.vo_state)

    def topic_names(self):
        names = sorted(self.reports.topic_names.values())
        return names + [CAMERA_TRACKING_POSITION_TOPIC, CAMERA_TRACKING_STATUS_TOPIC, POSE_TARGET_TOPIC, SPEED_TARGET_TOPIC]

    # ---- Rhythm ---------------------------------------------------------------------------------

    def now_seconds(self):
        return time.monotonic()

    def wait_one_cycle(self):
        """Handles every report that arrives until the next tenth of a second."""
        self.next_cycle_seconds += CYCLE_SECONDS
        if self.next_cycle_seconds < time.monotonic():
            # We fell behind, for example while planning. Start a fresh rhythm rather than rushing.
            self.next_cycle_seconds = time.monotonic() + CYCLE_SECONDS
        rclpy.spin_once(self.node, timeout_sec=0.0)
        while True:
            seconds_left = self.next_cycle_seconds - time.monotonic()
            if seconds_left <= 0.0:
                return
            rclpy.spin_once(self.node, timeout_sec=seconds_left)

    # ---- Camera tracking --------------------------------------------------------------------------

    def tracked_position(self):
        """Camera tracking's latest (forward, left, up) in metres, or None."""
        return None if self.tracked_position_metres is None else self.tracked_position_metres.copy()

    def camera_tracking_age_seconds(self):
        if self.tracked_position_arrival_seconds is None:
            return float("inf")
        return time.monotonic() - self.tracked_position_arrival_seconds

    def camera_tracking_says_lost(self):
        return self.camera_tracking_state == CAMERA_TRACKING_LOST_STATE

    # ---- Positions from the launch mark ---------------------------------------------------------

    def position_from_launch(self):
        """The drone's (forward, left, up) in metres from where it sat on the launch mark."""
        tracked = self.tracked_position()
        if tracked is None or self.launch is None:
            return None
        return tracked - self.launch.tracked_position_metres

    def send_position_from_launch(self, forward_left_up_metres):
        """Sends a position target given in the launch frame.

        The height needs a correction. flight_interface turns our target's "up" straight into
        PX4's height (from the organizers' code, frames.setpoint_position_enu_to_ned: PX4 z is
        minus our up, with no shift). But PX4's height and camera tracking's height have
        different zero points. Measured on drone 16 on 2026-11-08: on the launch mark, camera
        tracking said up 0.00 m and PX4 said z = -0.15 m. A target of 0.80 m up then stopped the
        drone 0.65 m above the floor, and the climb timed out. So we give the height as PX4 will
        read it: PX4's launch height plus the height we want. Forward and left need no
        correction, because PX4 takes them from camera tracking (they agreed within 2 cm)."""
        target = self.launch.tracked_position_metres + np.asarray(forward_left_up_metres, dtype=float)
        px4_launch_height_up_metres = -self.launch.px4_z_metres
        target[2] = px4_launch_height_up_metres + float(forward_left_up_metres[2])
        self.targets.send_position(target)

    # ---- Heights -------------------------------------------------------------------------------

    def px4_height_risen_metres(self):
        """How far PX4 thinks the drone has risen since the launch mark, or None."""
        if not self.reports.px4_height_estimate_is_working() or self.launch is None:
            return None
        return self.launch.px4_z_metres - self.reports.px4_z_metres()

    def px4_height_above_floor_metres(self):
        """PX4's laser-based distance to the ground, less its reading on the launch mark, or None.
        This is the height the normal landing uses."""
        distance_metres = self.reports.px4_distance_to_ground_metres()
        if distance_metres is None or self.launch is None:
            return None
        return distance_metres - self.launch.px4_distance_to_ground_metres

    def laser_height_risen_metres(self):
        """How far the bottom laser has risen since the launch mark, from its raw reading, or None.
        It stays right even when PX4 has stopped trusting the laser."""
        if self.launch is None or not self.raw_laser_available:
            return None
        return self.reports.laser_height_risen_since_launch_metres(self.launch.raw_laser_metres)

    def px4_distance_to_ground_is_usable(self):
        """True while PX4 reports a working height estimate, recently enough to land with."""
        fresh = self.reports.seconds_since("px4_position") <= PX4_POSITION_LONGEST_AGE_SECONDS
        return fresh and self.reports.px4_height_estimate_is_working()

    def px4_laser_flags(self):
        """(PX4 uses the laser for height, PX4 says the laser is faulty), each None if PX4's
        estimator flags are missing or stale."""
        if self.reports.seconds_since("estimator_flags") > ESTIMATOR_FLAGS_LONGEST_AGE_SECONDS:
            return None, None
        flags = self.reports.latest["estimator_flags"]
        return bool(flags.cs_rng_hgt), bool(flags.cs_rng_fault)

    def downward_speed_metres_per_second(self):
        if not self.reports.has("px4_position"):
            return None
        return self.reports.px4_downward_speed_metres_per_second()

    def tilt_degrees(self):
        return self.reports.tilt_degrees() if self.reports.has_attitude() else None

    def rotation_degrees_per_second(self):
        return self.reports.rotation_degrees_per_second() if self.reports.has_turn_rate() else None

    # ---- The organizers' summary -------------------------------------------------------------------

    def control_is_ours(self):
        return self.reports.control_is_ours()

    def is_armed(self):
        return self.reports.is_armed()

    def is_landed(self):
        state = self.reports.state()
        return state is not None and state["landed"] is True

    # ---- Records ---------------------------------------------------------------------------------

    def record_drone_condition(self):
        """Adds one row to drone_condition.csv. Called once per cycle by every step."""
        self.log.sample("drone_condition", stage=self.stage, **_tracking_columns(self),
                        **_px4_columns(self.reports), **_height_columns(self), **_status_columns(self.reports),
                        **_report_age_columns(self))

    def _record_target(self, sent_target):
        self.log.sample("targets", stage=self.stage, kind=sent_target.kind,
                        forward_or_speed=sent_target.forward_left_up[0], left_or_speed=sent_target.forward_left_up[1],
                        up_or_speed=sent_target.forward_left_up[2])


def _tracking_columns(link):
    tracked = link.tracked_position()
    from_launch = link.position_from_launch()
    return {
        "tracked_forward_metres": None if tracked is None else tracked[0],
        "tracked_left_metres": None if tracked is None else tracked[1],
        "tracked_up_metres": None if tracked is None else tracked[2],
        "from_launch_forward_metres": None if from_launch is None else from_launch[0],
        "from_launch_left_metres": None if from_launch is None else from_launch[1],
        "from_launch_up_metres": None if from_launch is None else from_launch[2],
        "camera_tracking_lost": link.camera_tracking_says_lost(),
    }


def _px4_columns(reports):
    position = reports.latest.get("px4_position")
    names = ["x", "y", "z", "vx", "vy", "vz", "dist_bottom", "xy_valid", "z_valid"]
    columns = {}
    for name in names:
        columns[f"px4_{name}"] = None if position is None else getattr(position, name)
    return columns


def _height_columns(link):
    return {
        "px4_height_risen_metres": link.px4_height_risen_metres(),
        "px4_height_above_floor_metres": link.px4_height_above_floor_metres(),
        "raw_laser_metres": link.reports.raw_laser_reading_metres(),
        "laser_height_risen_metres": link.laser_height_risen_metres(),
        "px4_uses_laser_for_height": link.reports.px4_uses_laser_for_height(),
        "tilt_degrees": link.tilt_degrees(),
        "rotation_degrees_per_second": link.rotation_degrees_per_second(),
    }


def _status_columns(reports):
    state = reports.state() or {}
    status = reports.latest.get("px4_status")
    land_detected = reports.latest.get("px4_land_detected")
    return {
        "offboard": state.get("offboard_active"),
        "armed": state.get("armed"),
        "landed": state.get("landed"),
        "nav_state": None if status is None else int(status.nav_state),
        "px4_land_detected": None if land_detected is None else bool(land_detected.landed),
        "battery_volts": reports.battery_volts(),
    }


def _report_age_columns(link):
    columns = {"camera_tracking_age_seconds": _rounded_age(link.camera_tracking_age_seconds())}
    for report_name in ["state", "px4_position", "px4_status", "px4_odometry", "px4_attitude", "motion_sensors", "laser",
                        "estimator_flags", "battery"]:
        columns[f"{report_name}_age_seconds"] = _rounded_age(link.reports.seconds_since(report_name))
    return columns


def _rounded_age(seconds):
    return None if math.isinf(seconds) else round(seconds, 3)


# =================================================================================================
# 2. The safety checks, once per cycle
# =================================================================================================

class SafetyWatch:
    """Runs run_safety.py's checks on the drone's condition and acts on the decision.

    start_frame (start_frame.py) converts between the course map and the launch frame. It is
    needed because the height check must know where the drone is on the course."""

    def __init__(self, link, run_safety, start_frame):
        self.link = link
        self.run_safety = run_safety
        self.start_frame = start_frame
        self.is_holding = False
        self.height_problem_seen = False
        # The decision of check_during_landing's last cycle, so each new one is written down once.
        self.last_landing_action = CARRY_ON

    def drone_on_course(self):
        """(east, north, height) on the course map, or None."""
        from_launch = self.link.position_from_launch()
        return None if from_launch is None else self.start_frame.course_from_start_frame(from_launch)

    def send_target_on_course(self, target_on_course_metres, check_distance_from_drone=True):
        """Checks the target is inside the course, then sends it. A refused target stops the run."""
        drone = self.drone_on_course() if check_distance_from_drone else None
        try:
            self.run_safety.check_target(target_on_course_metres, drone)
        except TargetRefused as refusal:
            raise RunMustStop(str(refusal)) from refusal
        self.link.send_position_from_launch(self.start_frame.start_frame_from_course(target_on_course_metres))

    def check(self):
        """Runs the checks. Returns True if the drone must hold still this cycle. Raises
        RunMustStop or ControlIsNoLongerOurs when the run must end."""
        condition = self._gather_condition()
        decision = self.run_safety.check(condition)
        self._record(condition, decision)
        if decision.action == CONTROL_LOST:
            raise ControlIsNoLongerOurs(decision.reason)
        if decision.action == LAND_HERE:
            self.height_problem_seen = self.height_problem_seen or decision.land_using_laser
            raise RunMustStop(decision.reason, land_using_laser=decision.land_using_laser)
        self._announce_hold_changes(decision)
        return decision.action == HOLD

    def check_during_landing(self):
        """Runs the same checks once during a landing's descent, before touchdown, and records
        them. The drone is already landing, so a touch or a camera tracking problem changes
        nothing; each new kind of decision is written down once. Returns True once the checks
        have found PX4's height untrustworthy, at any time in this run. Raises
        ControlIsNoLongerOurs if control has gone."""
        condition = self._gather_condition()
        decision = self.run_safety.check(condition)
        self._record(condition, decision)
        if decision.action == CONTROL_LOST:
            raise ControlIsNoLongerOurs(decision.reason)
        if decision.action != CARRY_ON and decision.action != self.last_landing_action:
            self.link.log.event("landing_check", f"During the landing: {decision.reason} The landing carries on.")
        self.last_landing_action = decision.action
        self.height_problem_seen = self.height_problem_seen or decision.land_using_laser
        return self.height_problem_seen

    def px4_height_is_in_doubt(self):
        """True if a landing should find the floor with the laser's raw reading, not PX4's height."""
        if self.height_problem_seen:
            return True
        if not self.link.reports.px4_height_estimate_is_working():
            return True
        # PX4 says it has stopped using the laser for height (docs/ARCHITECTURE.md, section 7).
        # Only counted while airborne: on the floor PX4 may use the air pressure sensor instead.
        uses_laser, laser_is_faulty = self.link.px4_laser_flags()
        if laser_is_faulty:
            return True
        return uses_laser is False and self.run_safety.laser_use_check.laser_was_used_while_airborne

    def _gather_condition(self):
        link = self.link
        uses_laser, laser_is_faulty = link.px4_laser_flags()
        return DroneCondition(
            now_seconds=link.now_seconds(),
            control_is_ours=link.control_is_ours(),
            is_armed=link.is_armed(),
            camera_tracking_age_seconds=link.camera_tracking_age_seconds(),
            camera_tracking_says_lost=link.camera_tracking_says_lost(),
            position_on_course_metres=self.drone_on_course(),
            tilt_degrees=link.tilt_degrees(),
            rotation_degrees_per_second=link.rotation_degrees_per_second(),
            px4_height_risen_metres=link.px4_height_risen_metres(),
            laser_height_risen_metres=link.laser_height_risen_metres(),
            raw_laser_available=link.raw_laser_available,
            px4_uses_laser_for_height=uses_laser,
            px4_says_laser_is_faulty=laser_is_faulty,
        )

    def _record(self, condition, decision):
        on_course = condition.position_on_course_metres
        laser_sees_clear_floor = None
        if on_course is not None:
            laser_sees_clear_floor = self.run_safety.height_check.laser_sees_clear_floor(on_course)
        self.link.log.sample(
            "safety_checks", stage=self.link.stage,
            course_east_metres=None if on_course is None else on_course[0],
            course_north_metres=None if on_course is None else on_course[1],
            course_height_metres=None if on_course is None else on_course[2],
            px4_height_risen_metres=condition.px4_height_risen_metres,
            laser_height_risen_metres=condition.laser_height_risen_metres,
            laser_sees_clear_floor=laser_sees_clear_floor, raw_laser_available=condition.raw_laser_available,
            px4_uses_laser_for_height=condition.px4_uses_laser_for_height,
            px4_says_laser_is_faulty=condition.px4_says_laser_is_faulty,
            tilt_degrees=condition.tilt_degrees, rotation_degrees_per_second=condition.rotation_degrees_per_second,
            camera_tracking_age_seconds=_rounded_age(condition.camera_tracking_age_seconds),
            decision=decision.action, reason=decision.reason,
        )

    def _announce_hold_changes(self, decision):
        holding_now = decision.action == HOLD
        if holding_now and not self.is_holding:
            self.link.log.event("hold", decision.reason)
        if self.is_holding and not holding_now:
            self.link.log.event("hold", "Camera tracking is reporting normally again. Carrying on.")
        self.is_holding = holding_now


# =================================================================================================
# 3. The steps
# =================================================================================================

def fly_until(link, is_finished, send_target, longest_seconds, what_we_are_waiting_for, watch=None):
    """Every cycle: run the safety checks (if a watch is given), send a target, record the
    drone's condition, and wait for the next cycle, until is_finished() is true.

    If the safety checks say hold, the last target is sent again instead of send_target, so the
    drone stays where it was told to be. Raises RunMustStop if it takes longer than longest_seconds."""
    start_seconds = link.now_seconds()
    while not is_finished():
        if link.now_seconds() - start_seconds > longest_seconds:
            raise RunMustStop(f"Waited {longest_seconds:.0f} seconds for {what_we_are_waiting_for}, and it did not happen.")
        must_hold = watch.check() if watch is not None else False
        if must_hold:
            link.targets.resend_last()
        else:
            send_target()
        link.record_drone_condition()
        link.wait_one_cycle()


def wait_until_ready_on_the_ground(link):
    """Waits, sending nothing, until every report the flight needs is fresh and the drone is on
    the ground, disarmed, with working camera tracking, for READY_STEADY_SECONDS in a row. Waits
    as long as it takes, reminding the people what is missing."""
    link.stage = "waiting until ready"
    link.log.event("stage", "Waiting until the drone is ready on the ground. Nothing is being sent yet.")
    ready_since_seconds = None
    last_reminder_seconds = link.now_seconds()
    while True:
        problems = what_is_not_ready(link)
        now = link.now_seconds()
        if problems:
            ready_since_seconds = None
            if now - last_reminder_seconds >= SECONDS_BETWEEN_REMINDERS:
                link.log.event("waiting", "Not ready yet: " + " ".join(problems), problems=problems)
                last_reminder_seconds = now
        elif ready_since_seconds is None:
            ready_since_seconds = now
        elif now - ready_since_seconds >= READY_STEADY_SECONDS:
            link.log.event("ready", "The drone is ready on the ground: fresh reports, landed, disarmed, tracking.")
            announce_which_checks_can_run(link)
            return
        link.record_drone_condition()
        link.wait_one_cycle()


def what_is_not_ready(link):
    """Sentences saying what stops the flight from starting, or an empty list."""
    reports = link.reports
    old_reports = reports.reports_older_than(REPORTS_NEEDED_BEFORE_START, READY_REPORT_LONGEST_AGE_SECONDS)
    if old_reports:
        return [f"These reports have not arrived in the last {READY_REPORT_LONGEST_AGE_SECONDS:.0f} s: "
                f"{', '.join(old_reports)}. Check that PX4, the bottom laser and flight_interface are running."]
    problems = []
    if link.camera_tracking_age_seconds() > READY_REPORT_LONGEST_AGE_SECONDS or link.camera_tracking_says_lost():
        problems.append("Camera tracking is not reporting a position. Check that cuVSLAM is running and can see detail.")
    state = reports.state()
    if not state["px4_link"]:
        problems.append("flight_interface says it is not hearing from PX4.")
    if state["landed"] is not True:
        problems.append("PX4 does not say the drone is landed.")
    if state["armed"]:
        problems.append("The drone is armed. Disarm it: the hand-over only follows an arming that starts after this.")
    if not reports.px4_height_estimate_is_working() or not reports.latest["px4_position"].xy_valid:
        problems.append("PX4 has no valid position or height estimate yet.")
    return problems


def announce_which_checks_can_run(link):
    """Decides, once, whether the laser's raw reading is used, and says loudly which safety
    checks run in a weaker form because a report is missing. Never waits for a missing report."""
    laser_arrives = link.reports.seconds_since("laser") <= READY_REPORT_LONGEST_AGE_SECONDS
    link.raw_laser_available = laser_arrives and not link.ignore_raw_laser
    if link.ignore_raw_laser:
        link.log.event("checks", "NO RAW LASER (ignored on purpose, --ignore-raw-laser): the height check watches "
                                 "PX4's own laser flags, and a landing after a height problem uses PX4's distance to "
                                 "the ground, or a gentle descent until PX4 says landed.")
    elif not laser_arrives:
        link.log.event("checks", f"NO RAW LASER: nothing has arrived on {link.reports.topic_names['laser']}. The height "
                                 f"check watches PX4's own laser flags instead, and a landing after a height problem "
                                 f"uses PX4's distance to the ground, or a gentle descent until PX4 says landed.")
    missing_reports = link.reports.reports_older_than(
        ["px4_odometry", "px4_attitude", "motion_sensors", "estimator_flags"], ESTIMATOR_FLAGS_LONGEST_AGE_SECONDS)
    attitude_missing = "px4_odometry" in missing_reports and "px4_attitude" in missing_reports
    turn_rate_missing = "px4_odometry" in missing_reports and "motion_sensors" in missing_reports
    if attitude_missing or turn_rate_missing:
        link.log.event("checks", f"NO TOUCH DETECTOR: PX4's tilt or turn rate is not arriving on "
                                 f"{link.reports.topic_names['px4_odometry']}, {link.reports.topic_names['px4_attitude']} "
                                 f"or {link.reports.topic_names['motion_sensors']}, so a touch can't be noticed.")
    elif "px4_odometry" in missing_reports:
        link.log.event("checks", "TOUCH DETECTOR USES BACKUPS: vehicle_odometry is not arriving, so tilt comes from "
                                 "vehicle_attitude and turn rate from the raw gyro in sensor_combined, which is noisier.")
    if "estimator_flags" in missing_reports and not link.raw_laser_available:
        link.log.event("checks", "NO HEIGHT CHECK: neither the laser's raw reading nor PX4's estimator flags are "
                                 "arriving, so PX4's height going wrong can't be noticed.")
    link.log.event("checks", f"Raw laser used: {link.raw_laser_available}.", raw_laser_available=link.raw_laser_available,
                   missing_optional_reports=link.reports.reports_older_than(OPTIONAL_REPORTS, ESTIMATOR_FLAGS_LONGEST_AGE_SECONDS))


def remember_launch_mark(link):
    """Records what the drone's reports say on the launch mark. Returns the LaunchReadings and
    keeps them in link.launch. Call it only when what_is_not_ready(link) is empty."""
    reports = link.reports
    link.launch = LaunchReadings(
        tracked_position_metres=link.tracked_position(),
        px4_z_metres=reports.px4_z_metres(),
        px4_distance_to_ground_metres=reports.px4_distance_to_ground_metres(),
        raw_laser_metres=reports.raw_laser_reading_metres(),
    )
    return link.launch


def hold_above_launch_until_control_is_ours(link, height_metres, launch_pose_problem=None):
    """Sends a target height_metres above the launch mark, every cycle, until the teammate has
    armed the drone and flipped the switch to Position, and flight_interface has handed control
    to us. Waits as long as it takes. Until then the target has no effect.

    While the drone is still disarmed on the ground, the launch readings are taken again every
    cycle, so they are the freshest ones when the flight starts, even if the drone was moved.

    launch_pose_problem is optional (the fallback missions pass one): a function of the link that
    returns a sentence when the launch pose is unacceptable, or None. It is asked every cycle while
    the drone is disarmed; on a sentence this stops sending the target, so the hand-over can't
    happen, and raises LaunchPoseRefused."""
    link.stage = "waiting for the start switch"
    remember_launch_mark(link)
    log_launch_readings(link)
    link.log.event("stage", f"Sending a target {height_metres:.2f} m above the launch mark. Waiting for a teammate to "
                            f"arm the drone and flip the switch to Position.")
    last_reminder_seconds = link.now_seconds()
    while not (link.control_is_ours() and link.is_armed()):
        if not link.is_armed() and link.is_landed() and not what_is_not_ready(link):
            remember_launch_mark(link)
        if launch_pose_problem is not None and not link.is_armed():
            problem = launch_pose_problem(link)
            if problem is not None:
                raise LaunchPoseRefused(problem)
        link.send_position_from_launch((0.0, 0.0, height_metres))
        link.record_drone_condition()
        if link.now_seconds() - last_reminder_seconds >= SECONDS_BETWEEN_REMINDERS:
            link.log.event("waiting", "Still waiting for the start switch. " + _arming_summary(link))
            last_reminder_seconds = link.now_seconds()
        link.wait_one_cycle()
    log_launch_readings(link)
    link.log.event("control", "Control is ours: PX4 is in Offboard mode and follows our targets.")


def log_launch_readings(link):
    launch = link.launch
    raw_laser_text = "none" if launch.raw_laser_metres is None else f"{launch.raw_laser_metres:.3f} m"
    link.log.event(
        "launch_mark",
        f"On the launch mark: camera tracking at {np.round(launch.tracked_position_metres, 3).tolist()}, PX4's distance "
        f"to the ground {launch.px4_distance_to_ground_metres:.3f} m, raw laser {raw_laser_text}.",
        tracked_position_metres=launch.tracked_position_metres.tolist(), px4_z_metres=launch.px4_z_metres,
        px4_distance_to_ground_metres=launch.px4_distance_to_ground_metres, raw_laser_metres=launch.raw_laser_metres,
    )


def _arming_summary(link):
    state = link.reports.state() or {}
    return f"Armed: {state.get('armed')}. Mode: {state.get('nav_state_name')}. Landed: {state.get('landed')}."


def climb_above_launch(link, watch, height_metres):
    """Climbs straight up to height_metres above the launch mark, then settles there."""
    def has_reached_height():
        from_launch = link.position_from_launch()
        return from_launch is not None and abs(from_launch[2] - height_metres) < HEIGHT_TOLERANCE_METRES

    def send_target():
        link.send_position_from_launch((0.0, 0.0, height_metres))

    fly_until(link, has_reached_height, send_target, LONGEST_CLIMB_SECONDS, "the climb", watch)
    link.log.event("climb", f"Reached {link.position_from_launch()[2]:.2f} m above the launch mark.")
    hold_still_for(link, watch, SECONDS_TO_SETTLE)


def hold_still_for(link, watch, seconds):
    """Repeats the last target for this long, with the safety checks running."""
    end_seconds = link.now_seconds() + seconds
    fly_until(link, lambda: link.now_seconds() >= end_seconds, link.targets.resend_last, seconds + 1.0,
              "a timed hold", watch)


def choose_landing_height_source(link, px4_height_is_in_doubt):
    """How a landing should find the floor. Normally PX4's distance to the ground. When PX4's
    height is in doubt: the laser's raw reading if we have it (it stays right whatever PX4 does);
    otherwise PX4's distance to the ground if PX4 still reports a working estimate, because over a
    short descent it drifts much less than the descent itself, and touchdown also needs the drone to
    stop moving down; otherwise no height at all."""
    if px4_height_is_in_doubt and link.raw_laser_available:
        return LASER_RAW_READING
    if link.px4_distance_to_ground_is_usable():
        return PX4_DISTANCE_TO_GROUND
    return NO_HEIGHT_READING


def land(link, height_source=PX4_DISTANCE_TO_GROUND, watch=None):
    """Lands where the drone is and waits until PX4 switches the motors off (section 7).
    height_source is one of LASER_RAW_READING, PX4_DISTANCE_TO_GROUND or NO_HEIGHT_READING
    (choose_landing_height_source picks one). watch, a SafetyWatch, is optional: with it, the
    safety checks keep running until touchdown, and if PX4's height goes wrong during the descent,
    the rest of it finds the floor with the laser's raw reading (DescentHeight). Raises
    ControlIsNoLongerOurs if control is taken away before the motors are off."""
    link.stage = "landing"
    link.log.event("landing", f"Landing, finding the floor with {height_source}.", height_source=height_source)
    if height_source == NO_HEIGHT_READING:
        descend_gently_until_the_motors_are_off(link)
        return
    descent_height = DescentHeight(link, height_source, watch)
    descend_to_gentle_height(link, descent_height.metres_above_floor, descent_height.check_once_per_cycle)
    touchdown_seconds = descend_gently_until_touchdown(link, descent_height.metres_above_floor,
                                                       descent_height.check_once_per_cycle)
    push_down_until_motors_are_off(link, touchdown_seconds)


class DescentHeight:
    """The height above the floor that a landing's descent goes by, and the safety checks that
    keep running until touchdown.

    It starts with the source the landing was given. If the checks find PX4's height wrong during
    the descent, for example because PX4 stopped using the laser over the pad, and the laser's raw
    reading exists, the rest of the descent uses the raw reading, which stays right whatever PX4
    does. Without a raw reading (the real drone) there is nothing better to switch to, so the
    problem is only recorded; touchdown then also counts when the drone stops moving down.

    The checks stop at touchdown, on purpose: while the drone presses down on the floor, PX4's
    height drifts up to 45 cm away from the truth (seen in simulator flights; docs/ARCHITECTURE.md,
    section 7). That is harmless, because the drone is already down, and must not look like a
    problem."""

    def __init__(self, link, height_source, watch):
        self.link = link
        self.height_source = height_source
        self.watch = watch

    def metres_above_floor(self):
        if self.height_source == LASER_RAW_READING:
            return self.link.laser_height_risen_metres()
        return self.link.px4_height_above_floor_metres()

    def check_once_per_cycle(self):
        if self.watch is None:
            return
        px4_height_is_wrong = self.watch.check_during_landing()
        if not px4_height_is_wrong or self.height_source == LASER_RAW_READING:
            return
        if not self.link.raw_laser_available:
            return
        self.height_source = LASER_RAW_READING
        self.link.log.event("landing", "PX4's height can't be trusted any more, so the rest of the descent finds the "
                                       "floor with the bottom laser's raw reading.", height_source=self.height_source)


def descend_gently_until_the_motors_are_off(link):
    """The landing with no height reading: the gentle speed all the way, which is safe to hit the
    floor at, until PX4 decides it has landed and switches the motors off. No push-down."""
    start_seconds = link.now_seconds()
    last_reminder_seconds = start_seconds
    while link.is_armed():
        if link.now_seconds() - last_reminder_seconds >= SECONDS_BETWEEN_REMINDERS:
            link.log.event("waiting", "Still descending gently, waiting for PX4 to say landed. " + _arming_summary(link))
            last_reminder_seconds = link.now_seconds()
        _send_landing_target(link, GENTLE_DESCENT_METRES_PER_SECOND, motors_off_is_expected=True)
    link.log.event("motors_off", f"The motors are off, {link.now_seconds() - start_seconds:.1f} s after the gentle "
                                 f"descent began.")


def descend_to_gentle_height(link, height_above_floor, check_each_cycle=None):
    """Descends at the medium speed until height_above_floor() is below the gentle descent's
    starting height. check_each_cycle, if given, is called at the start of every cycle."""
    start_seconds = link.now_seconds()
    while True:
        if check_each_cycle is not None:
            check_each_cycle()
        height = height_above_floor()
        if height is None or height < HEIGHT_TO_START_GENTLE_DESCENT_METRES:
            return   # With no height reading at all, the gentle speed is the safe one.
        if link.now_seconds() - start_seconds > LONGEST_DESCENT_STAGE_SECONDS:
            link.log.event("landing", "The medium-speed descent took too long. Carrying on more gently.")
            return
        _send_landing_target(link, MEDIUM_DESCENT_METRES_PER_SECOND)


def descend_gently_until_touchdown(link, height_above_floor, check_each_cycle=None):
    """Descends at the gentle speed until touchdown, and returns the time of touchdown.
    check_each_cycle, if given, is called at the start of every cycle before touchdown."""
    detector = TouchdownDetector(link.now_seconds())
    while True:
        if check_each_cycle is not None:
            check_each_cycle()
        now = link.now_seconds()
        touched_down_because = detector.check(now, height_above_floor(), link.downward_speed_metres_per_second())
        if touched_down_because is not None:
            link.log.event("touchdown", touched_down_because, height_above_floor_metres=height_above_floor())
            return now
        if now - detector.start_seconds > LONGEST_DESCENT_STAGE_SECONDS:
            link.log.event("landing", "No touchdown was seen during the gentle descent. Pushing down anyway.")
            return now
        _send_landing_target(link, GENTLE_DESCENT_METRES_PER_SECOND)


def push_down_until_motors_are_off(link, touchdown_seconds):
    """Commands a fast descent, which the floor stops, so that PX4 lowers the motor power quickly,
    decides the drone has landed and switches the motors off. Waits as long as that takes:
    pushing down on the floor does no harm."""
    link.log.event("landing", f"Pushing down at {PUSH_DOWN_METRES_PER_SECOND} m/s until PX4 switches the motors off.")
    px4_said_landed = False
    last_reminder_seconds = link.now_seconds()
    while link.is_armed():
        if not px4_said_landed and link.is_landed():
            px4_said_landed = True
            link.log.event("landing", f"PX4 says landed, {link.now_seconds() - touchdown_seconds:.1f} s after touchdown.")
        if link.now_seconds() - last_reminder_seconds >= SECONDS_BETWEEN_REMINDERS:
            link.log.event("waiting", "Still waiting for PX4 to switch the motors off. " + _arming_summary(link))
            last_reminder_seconds = link.now_seconds()
        _send_landing_target(link, PUSH_DOWN_METRES_PER_SECOND, motors_off_is_expected=True)
    link.log.event("motors_off", f"The motors are off, {link.now_seconds() - touchdown_seconds:.1f} s after touchdown.")


def _send_landing_target(link, downward_metres_per_second, motors_off_is_expected=False):
    """One landing cycle: stop if control has gone, send the downward speed, record, wait."""
    if not motors_off_is_expected and not link.is_armed():
        raise ControlIsNoLongerOurs("The motors were switched off before the landing finished (the kill switch, or PX4).")
    if link.is_armed() and not link.control_is_ours():
        raise ControlIsNoLongerOurs("PX4 left Offboard mode during the landing, so a teammate or PX4 has taken over.")
    link.targets.send_downward_speed(downward_metres_per_second)
    link.record_drone_condition()
    link.wait_one_cycle()


class TouchdownDetector:
    """Decides when the drone has touched the floor during the gentle descent. Plain arithmetic,
    so it is tested on its own (test/test_flight_steps.py).

    Touchdown is either of:
      - the height is within TOUCHDOWN_HEIGHT_TOLERANCE_METRES of the launch mark's and the drone
        is not moving down, for TOUCHDOWN_SECONDS_REQUIRED (the normal case, section 7);
      - the drone has stopped moving down for STALLED_DESCENT_SECONDS, once the gentle descent
        has run for GENTLE_DESCENT_BEFORE_A_STALL_COUNTS_SECONDS. This covers a laser that is
        stuck or silent, and the drone sitting on something above the floor."""

    def __init__(self, start_seconds):
        self.start_seconds = start_seconds
        self.near_floor_since_seconds = None
        self.not_descending_since_seconds = None

    def check(self, now_seconds, height_above_floor_metres, downward_speed_metres_per_second):
        """Returns None until touchdown, then a sentence saying why it counts as touchdown."""
        not_descending = (downward_speed_metres_per_second is not None
                          and downward_speed_metres_per_second < TOUCHDOWN_LARGEST_DOWNWARD_SPEED_METRES_PER_SECOND)
        near_floor = (height_above_floor_metres is not None
                      and height_above_floor_metres < TOUCHDOWN_HEIGHT_TOLERANCE_METRES)
        self.not_descending_since_seconds = _since(self.not_descending_since_seconds, not_descending, now_seconds)
        self.near_floor_since_seconds = _since(self.near_floor_since_seconds, near_floor and not_descending, now_seconds)

        if self.near_floor_since_seconds is not None and now_seconds - self.near_floor_since_seconds >= TOUCHDOWN_SECONDS_REQUIRED:
            return "Touchdown: the height is back to the launch mark's and the drone has stopped moving down."
        stall_may_count = now_seconds - self.start_seconds >= GENTLE_DESCENT_BEFORE_A_STALL_COUNTS_SECONDS
        stalled = (self.not_descending_since_seconds is not None
                   and now_seconds - self.not_descending_since_seconds >= STALLED_DESCENT_SECONDS)
        if stall_may_count and stalled:
            return (f"Touchdown: the drone has stopped moving down for {STALLED_DESCENT_SECONDS:.0f} s, although the "
                    f"height reading ({height_above_floor_metres}) is not back to the launch mark's.")
        return None


def _since(since_seconds, condition_holds, now_seconds):
    """Keeps the time a condition began, or None while it doesn't hold."""
    if not condition_holds:
        return None
    return now_seconds if since_seconds is None else since_seconds
