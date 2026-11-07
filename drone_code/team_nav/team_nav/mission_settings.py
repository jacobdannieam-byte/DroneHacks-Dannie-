"""Every number the mission program uses, with its unit and where it came from.

How it fits: mission.py, course_stages.py and landing.py read these values; mission_node.py turns
each one into a ROS parameter of the same name, so the team can change it at launch without
editing code. This file holds no logic.

Each value is labelled as one of: measured, estimated, from PX4 source code, from a PX4 setting on
drone 16, from the rulebook, from the organizers' code, from docs/ARCHITECTURE.md (the team plan),
or our own choice.
"""
from dataclasses import dataclass


@dataclass
class MissionSettings:
    # ------------------------------------------------------------------ flight
    # From the team plan (section 1): low enough to fit through the window, whose top edge is
    # 1.22 m above the floor.
    cruise_height_metres: float = 0.8
    # Our own choice: a gentle climb, so the takeoff does not overshoot.
    climb_speed_metres_per_second: float = 0.5
    # From the organizers' code: a position target is never lower than 0.3 m above where the
    # camera tracking started, so a takeoff target starts there.
    lowest_position_target_metres: float = 0.3
    # From a PX4 setting on drone 16 (COM_DISARM_PRFLT = 10 s): PX4 switches the motors off if
    # the drone has not left the ground within 10 s of arming.
    leave_ground_deadline_seconds: float = 10.0
    # Our own choice: this far above the starting height counts as having left the ground.
    left_ground_height_metres: float = 0.1
    # Our own choice: within this of cruise height, the takeoff is complete.
    cruise_height_tolerance_metres: float = 0.1
    # Our own choice: if cruise height is not reached in this time, land.
    takeoff_time_limit_seconds: float = 15.0
    # From the organizer code audit: velocity commands must keep coming at 20 Hz through the whole
    # landing (one expires after 1 s, and then the drone flies back to the last position target).
    # The whole mission steps at this rate.
    steps_per_second: float = 20.0
    # From the rulebook: a run has 3 minutes (180 s). Our own choice: start landing at 170 s,
    # which leaves about 10 s for the landing itself (section 7 measured about 9 s).
    flight_time_limit_seconds: float = 170.0
    # Our own choice: the drone cannot really move this far in one step (a twentieth of a second), so
    # a jump this big means camera tracking has jumped, and every position goal is now wrong.
    position_jump_limit_metres: float = 0.5

    # ------------------------------------------------------------------ freshness of inputs
    # From the team plan (section 11): if position reports stop for more than 0.2 s, hold.
    position_report_max_age_seconds: float = 0.2
    # Our own choice: the organizers' code reports the flight state 5 times per second.
    flight_state_max_age_seconds: float = 1.0
    # Our own choice: PX4 sends its position report many times per second.
    px4_report_max_age_seconds: float = 0.5
    # Our own choice, for the bottom laser sensor and every detector input below.
    laser_report_max_age_seconds: float = 0.5
    line_max_age_seconds: float = 0.5
    pad_max_age_seconds: float = 0.5
    clearances_max_age_seconds: float = 0.5
    vision_max_age_seconds: float = 0.5
    # Our own choice: allows for small clock differences between programs.
    future_stamp_tolerance_seconds: float = 0.05

    # ------------------------------------------------------------------ moving between points
    # From the team plan (section 6): the target ("carrot") is kept about 0.5 m ahead of the drone,
    # which makes it fly at about 0.5 m/s.
    carrot_lead_metres: float = 0.5
    # Our own choice: sideways and backward the forward camera sees nothing, so stay this close.
    sideways_carrot_lead_metres: float = 0.3
    # Our own choice.
    cruise_speed_metres_per_second: float = 0.5
    slow_speed_metres_per_second: float = 0.3
    sideways_speed_limit_metres_per_second: float = 0.2
    turn_rate_degrees_per_second: float = 45.0
    # Our own choice: never aim closer than this to something the depth camera sees ahead.
    stopping_distance_metres: float = 0.9
    # Our own choice: less room ahead than this counts as blocked (a hold, not a crawl).
    smallest_useful_room_metres: float = 0.1

    # ------------------------------------------------------------------ holds and time limits
    # Our own choice: a hold that waits for an input (line lost, way ahead unknown, camera
    # tracking unsure) lands after this long.
    longest_wait_for_input_seconds: float = 5.0
    # Our own choice: a stage past its time limit hovers this long, then lands.
    hover_after_time_limit_seconds: float = 5.0
    # Our own choice: a detection must be seen in this many reports in a row.
    sightings_to_confirm: int = 3
    # Our own choice, per stage.
    follow_line_time_limit_seconds: float = 60.0
    gate_a_time_limit_seconds: float = 15.0
    tunnel_align_time_limit_seconds: float = 20.0
    tunnel_cross_time_limit_seconds: float = 15.0
    tunnel_exit_time_limit_seconds: float = 10.0
    chicane_time_limit_seconds: float = 40.0
    find_pad_time_limit_seconds: float = 25.0

    # ------------------------------------------------------------------ stage settings
    # Our own choice: off until the tunnel crossing has been tested. Off means the drone hovers
    # at the tunnel entrance and lands there, which keeps checkpoints 1 and 2.
    tunnel_enabled: bool = False
    # Our own choice: off until a window detector exists. On only says so and uses Gate B.
    window_enabled: bool = False
    # Our own choice: sideways speed per metre that the line is off centre.
    line_follow_gain_per_second: float = 0.8
    # Our own choice: a line or pad further round than this, turn on the spot before moving.
    turn_on_the_spot_degrees: float = 30.0
    # Estimated: how far past the end of the line counts as through Gate A. Measure on the course.
    gate_a_pass_distance_metres: float = 1.2
    # Our own choice: sideways speed per metre of difference between the left and right gaps.
    gap_centring_gain_per_second: float = 0.5
    # Our own choice: both side gaps nearer than this means an opening in the barrier.
    barrier_side_distance_metres: float = 1.6
    # Our own choice: the barrier this close ahead means the drone is at the tunnel entrance.
    tunnel_standoff_metres: float = 1.2
    # Our own choice: square to the barrier when the left and right gaps agree this closely.
    square_up_tolerance_metres: float = 0.1
    square_up_gain_radians_per_metre: float = 0.5
    hover_at_tunnel_entrance_seconds: float = 3.0
    # Estimated from the course model: the tunnel is 1.52 m long, plus the standoff and a margin.
    tunnel_travel_limit_metres: float = 3.5
    tunnel_minimum_travel_metres: float = 0.5
    # Our own choice: depth back this far ahead after a dark stretch means out of the tunnel.
    tunnel_exit_clear_metres: float = 1.5
    tunnel_exit_settle_seconds: float = 1.0
    # Our own choice: in the chicane, way ahead nearer than this means turn toward the open side,
    # and once turning, it must be this open again before going on (stops it flickering).
    chicane_turn_distance_metres: float = 1.5
    chicane_go_again_distance_metres: float = 1.8
    # Our own choice: +1 turns left, -1 turns right when neither side is more open.
    chicane_default_turn: int = 1
    # Our own choice: each new pad sighting moves the pad's remembered position half way.
    pad_smoothing_fraction: float = 0.5
    # From the team plan (section 7): the drone's centre can be up to about 48 cm from the pad
    # centre and still be inside the ring. Our own choice: aim for 15 cm.
    pad_centre_tolerance_metres: float = 0.15
    pad_settle_seconds: float = 1.0
    # Our own choice: closer than this to the pad, stop turning to face it.
    stop_facing_pad_metres: float = 0.5

    # ------------------------------------------------------------------ landing (plan section 7)
    # From the team plan, step 2: settle over the landing spot for up to 3 s.
    landing_settle_time_limit_seconds: float = 3.0
    # Our own choice: slower than this sideways counts as settled.
    settled_speed_metres_per_second: float = 0.1
    # Our own choice: the laser must read within this of the expected height before descending
    # at the medium speed (step 3).
    laser_expected_tolerance_metres: float = 0.15
    # Our own choice: how long to hold and look again when the laser reading is unexpected.
    laser_recheck_seconds: float = 3.0
    # From the team plan, step 4.
    medium_descent_speed_metres_per_second: float = 0.5
    gentle_descent_height_metres: float = 0.3
    # From the team plan, step 5: just above PX4's landing check of 0.275 m/s (from PX4 source
    # code: 1.1 x LNDMC_Z_VEL_MAX, which is 0.25 on drone 16).
    gentle_descent_speed_metres_per_second: float = 0.35
    # From the team plan, step 5: a sideways push at touchdown is what tips a drone over.
    no_sideways_corrections_below_metres: float = 0.2
    # Our own choice: how hard to steer back over the landing spot while above that height.
    landing_sideways_gain_per_second: float = 0.5
    # From the team plan, step 6: touchdown is the laser within 3 cm of the floor reading and the
    # drone not moving down, both for 0.3 s.
    touchdown_laser_tolerance_metres: float = 0.03
    touchdown_confirm_seconds: float = 0.3
    # Our own choice: moving down slower than this counts as stopped.
    stopped_moving_down_metres_per_second: float = 0.05
    # Our own choice: a laser that changes less than this over the confirm time is stuck.
    laser_stuck_change_metres: float = 0.01
    # From the team plan, step 7: push down at 1 m/s once on the floor.
    push_down_speed_metres_per_second: float = 1.0
    # From the team plan ("what if"): keep pushing past 8 s; after 15 s lift to 0.3 m and try once
    # more.
    motors_off_patience_seconds: float = 8.0
    landing_retry_after_seconds: float = 15.0
    landing_retry_lift_metres: float = 0.3
    # Our own choice.
    landing_retry_lift_speed_metres_per_second: float = 0.35
    landing_retry_lift_time_limit_seconds: float = 3.0
    # From a PX4 setting on drone 16 (EKF2_MIN_RNG = 0.1): the laser never reads below 0.1 m, so
    # this is the floor reading when none was recorded before takeoff.
    laser_floor_reading_if_unrecorded_metres: float = 0.1
    # Our own choice: how many seconds of laser readings on the launch mark make the floor reading.
    floor_reading_window_seconds: float = 2.0

    # ------------------------------------------------------------------ camera exposure
    # From the organizer code audit: cap the camera's automatic exposure, so a long exposure in dim
    # light does not blur the pictures camera tracking depends on.
    cap_camera_exposure: bool = True
    # Estimated from the camera topic names (/oak/left/..., /oak/right/...). Check on the drone with
    # "ros2 param list /oak".
    camera_node_name: str = "/oak"
    # From the organizer code audit: the longest automatic exposure, in microseconds.
    camera_auto_exposure_limit_microseconds: int = 8000
    # Our own choice: how often to check that the camera still has the cap (it loses it if the
    # camera program restarts).
    camera_check_seconds: float = 5.0
