"""Table check: PX4's position estimator uses the sensors drone 16's settings say it should.

REAL DRONE CODE, run on the table with the PROPELLERS OFF. It only listens to the drone.

Why this is checked this way
----------------------------
PX4 is the flight controller's software. Inside it, a part called EKF2 (the "estimator")
combines the sensors into one best guess of where the drone is. Which sensors it uses is set by
PX4 settings ("parameters"), and our whole plan depends on two of them:
  - EKF2_HGT_REF = 2: the bottom laser is the main source of height.
  - EKF2_EV_CTRL = 9: camera tracking ("external vision", EV) gives sideways position (bit 0,
    worth 1) and heading (bit 3, worth 8), but not height (bit 1) or speed (bit 2).
These values come from drone 16's settings file of 2026-11-03 (simulator/px4_changes/
drone_16_settings.sh copies them).

PX4's settings can't be read through the ROS link our code uses. But EKF2 reports what it is
using right now, in /fmu/out/estimator_status_flags, and those flags follow from the settings.
So this check reads the flags and compares them with what the settings should produce. It
can't see settings that don't change a flag, such as EKF2_MIN_RNG, EKF2_RNG_K_GATE, the landing
settings or the safety settings; README.md lists what still needs QGroundControl.

PASS means every flag in EXPECTED_FLAGS has the expected value. The other flags are recorded.
"""
from team_nav.hand_tests.hand_test_tools import failed, passed, px4_silence_problem

# (flag, expected value, what it means and why we expect that value)
EXPECTED_FLAGS = [
    ("cs_tilt_align", True, "EKF2 knows which way is down"),
    ("cs_ev_pos", True, "camera tracking gives sideways position (EKF2_EV_CTRL bit 0)"),
    ("cs_ev_yaw", True, "camera tracking gives heading (EKF2_EV_CTRL bit 3)"),
    ("cs_ev_hgt", False, "camera tracking does not give height (EKF2_EV_CTRL bit 1 is off)"),
    ("cs_ev_vel", False, "camera tracking does not give speed (EKF2_EV_CTRL bit 2 is off)"),
    # In our simulator this is true with the drone on the floor. On drone 16 on 2026-11-07 it was
    # false on the floor, so its height came from the barometer; the laser_heights hand test
    # finds out whether, and from what height, PX4 starts using the laser.
    ("cs_rng_hgt", True, "the bottom laser gives height (EKF2_HGT_REF = 2)"),
    ("cs_rng_fault", False, "EKF2 has not declared the laser faulty"),
    ("cs_gnss_pos", False, "no GPS is used indoors (EKF2_GPS_CTRL = 0)"),
    ("cs_inertial_dead_reckoning", False, "EKF2 has a sensor that holds its position, not motion sensors alone"),
]

# Recorded for the team to look at, with no expected value.
# cs_yaw_align was false in our simulator, where heading comes from camera tracking, so it is
# recorded rather than required.
OTHER_FLAGS_TO_RECORD = ["cs_yaw_align", "cs_rng_terrain", "cs_in_air", "cs_baro_hgt", "cs_rng_kin_consistent", "cs_rng_stuck", "cs_mag_hdg",
                         "cs_mag_3d", "cs_vehicle_at_rest"]

# Our own choice. The flags come about once a second (PX4's EKF2.cpp), so wait a little longer.
SECONDS_TO_WAIT_FOR_FLAGS = 3.0


def judge_estimator_flags(flag_values):
    """flag_values maps each flag name to True or False. Returns the verdict."""
    wrong = []
    for flag_name, expected, meaning in EXPECTED_FLAGS:
        if flag_values.get(flag_name) != expected:
            wrong.append(f"{flag_name} is {flag_values.get(flag_name)} but should be {expected} ({meaning})")
    if wrong:
        return failed("PX4's estimator is not using the sensors drone 16's settings say: " + "; ".join(wrong)
                      + ". Check those settings in QGroundControl, and that camera tracking and the laser are running.",
                      flags=flag_values)
    return passed("PX4's estimator uses the laser for height and camera tracking for position and heading, "
                  "as drone 16's settings EKF2_HGT_REF = 2 and EKF2_EV_CTRL = 9 should make it.", flags=flag_values)


# ---- Running it on the drone -------------------------------------------------------------------

def run(listener, person, log):
    listener.wait(SECONDS_TO_WAIT_FOR_FLAGS)
    problem = px4_silence_problem(listener.reports.seconds_since("px4_status"))
    if problem:
        return problem
    flags_message = listener.reports.latest.get("estimator_flags")
    if flags_message is None:
        return failed(f"PX4's estimator flags never arrived on {listener.reports.topic_names['estimator_flags']}, "
                      f"although PX4 is talking. Check 'ros2 topic list' for estimator_status_flags.")
    flag_values = {}
    for flag_name in [name for name, _, _ in EXPECTED_FLAGS] + OTHER_FLAGS_TO_RECORD:
        flag_values[flag_name] = bool(getattr(flags_message, flag_name))
    log.event("flags", "PX4's estimator flags.", flags=flag_values)
    return judge_estimator_flags(flag_values)
