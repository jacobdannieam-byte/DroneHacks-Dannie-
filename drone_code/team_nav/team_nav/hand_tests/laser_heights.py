"""Hand test: hold the drone at known heights, and find out whether the bottom laser works and
from what height PX4 uses it.

REAL DRONE CODE, run with the PROPELLERS OFF. It only listens to the drone.

What it proves
--------------
The bottom laser points straight down and measures the distance to whatever is below. PX4, the
flight controller's software, should use it as the drone's height (drone 16's setting
EKF2_HGT_REF = 2), and our landing depends on it (docs/ARCHITECTURE.md, section 7). On
2026-11-07, with the drone on the floor, PX4 was NOT using the laser: its estimator flag
cs_rng_hgt was false, so its height came from the air pressure sensor (the "barometer"). This
test is our main way to find out whether the real laser works at all.

What it can read
----------------
On 2026-11-07 drone 16 did not send the laser's own raw reading (/fmu/out/distance_sensor). So
the test reads what PX4 reports:
  - dist_bottom: PX4's distance from the drone's centre to the ground below;
  - its estimator flags cs_rng_hgt (laser used for height) and cs_rng_terrain (laser used to
    measure the ground), which together say whether the laser is behind dist_bottom at all.
    When neither is set, dist_bottom comes from the barometer and says nothing about the laser.
    cs_rng_fault says PX4 has declared the laser faulty.
If the raw reading does arrive, the test also compares it with the tape.

How it works
------------
For each height, the person holds the drone level with the laser's lens at that height above the
floor and presses Enter. The test collects SECONDS_AT_EACH_HEIGHT of readings and prints, for
each height: is PX4 using the laser, and what does dist_bottom say against the tape.

PASS means PX4 used the laser at one or more of the held heights (or the raw reading arrived),
and wherever the laser was in use, its height matched the tape within LARGEST_HEIGHT_ERROR_METRES.
The result also says the lowest height at which PX4 used the laser. While the drone is disarmed
(its motors not allowed to spin) PX4 believes it is on the ground, which may change when PX4
starts using the laser; that is part of what this test finds out.
"""
import math
import statistics
from dataclasses import dataclass

from team_nav.hand_tests.hand_test_tools import failed, passed, px4_silence_problem

# Our own choices. 0.10 to 0.30 m is the landing's last stretch (section 7 asks for 0 to 30 cm),
# 0.50 m is in between, 0.80 m is our cruise height, and 1.20 m is above the window's lower wall.
HELD_HEIGHTS_METRES = [0.10, 0.20, 0.30, 0.50, 0.80, 1.20]

# Our own choice. Enough PX4 position reports (about 100 a second) and estimator flags (about 2 a
# second) at each height, and enough raw laser readings (about 5 a second) if they arrive.
SECONDS_AT_EACH_HEIGHT = 3.0
# Our own choice: fewer readings than this in SECONDS_AT_EACH_HEIGHT means they are not arriving.
FEWEST_READINGS_AT_EACH_HEIGHT = 5
# Our own choice: PX4 "uses the laser" at a height if its flags say so in at least this share of
# the reports there.
SHARE_OF_REPORTS_TO_COUNT_AS_USING_THE_LASER = 0.5

# From drone 16's settings file of 2026-11-03 (EKF2_RNG_POS_Z, copied in
# simulator/px4_changes/drone_16_settings.sh): the laser sits this far below the drone's centre.
# PX4 adds it to the laser's reading, so dist_bottom is measured from the centre. We assume the
# motion sensor is at the centre (EKF2_IMU_POS_Z is not in our copy of the settings).
LASER_BELOW_CENTRE_METRES = 0.03445

# Our own choice. A tape held by hand is good to about 2 cm, and a drone held by hand moves about
# 2 cm. A larger error is a real problem, and 5 cm is small against the 17 cm of room above and
# below the drone in the window.
LARGEST_HEIGHT_ERROR_METRES = 0.05
# Our own choice. "Level" when held by hand. dist_bottom allows for tilt itself, and the raw
# reading is corrected for it, but a large tilt means the person should hold it better.
LARGEST_TILT_WHILE_HELD_DEGREES = 10.0


@dataclass
class ReadingsAtOneHeight:
    tape_height_metres: float          # None means resting on its legs on the floor.
    px4_distances_to_ground_metres: list
    px4_uses_laser_flags: list         # One per PX4 report: True, False, or None if unknown.
    px4_declared_laser_faulty: bool
    raw_laser_readings_metres: list    # Empty if the drone doesn't send them.
    tilt_degrees: float

    def px4_uses_the_laser(self):
        known = [flag for flag in self.px4_uses_laser_flags if flag is not None]
        if not known:
            return False
        return sum(1 for flag in known if flag) / len(known) >= SHARE_OF_REPORTS_TO_COUNT_AS_USING_THE_LASER

    def px4_height_metres(self):
        """PX4's distance to the ground, as a height of the laser's lens above the floor."""
        return statistics.median(self.px4_distances_to_ground_metres) - LASER_BELOW_CENTRE_METRES

    def raw_height_metres(self):
        """The raw laser's height above the floor, corrected for tilt, or None."""
        if len(self.raw_laser_readings_metres) < FEWEST_READINGS_AT_EACH_HEIGHT:
            return None
        return statistics.median(self.raw_laser_readings_metres) * math.cos(math.radians(self.tilt_degrees))

    def place(self):
        if self.tape_height_metres is None:
            return "On the floor"
        return f"At {self.tape_height_metres:.2f} m"


@dataclass
class HeightOutcome:
    line: str              # One line for the screen: what PX4 and the laser said here.
    laser_was_seen: bool   # PX4 used the laser here, or the raw reading arrived.
    problem: str           # A sentence if the laser was seen and disagreed with the tape, else "".


def judge_one_height(readings):
    """What happened at one held height (or on the floor, where only the line is reported)."""
    if len(readings.px4_distances_to_ground_metres) < FEWEST_READINGS_AT_EACH_HEIGHT:
        return HeightOutcome(f"{readings.place()}: PX4's position reports did not arrive.", False,
                             f"{readings.place()} PX4's position reports did not arrive.")
    px4_height = readings.px4_height_metres()
    raw_height = readings.raw_height_metres()
    using = readings.px4_uses_the_laser()
    line = (f"{readings.place()}: PX4 {'IS' if using else 'is NOT'} using the laser"
            f"{' (and has declared it faulty)' if readings.px4_declared_laser_faulty else ''}; "
            f"dist_bottom gives {px4_height:.2f} m{'' if using else ' (from the barometer, not the laser)'}")
    if raw_height is not None:
        line += f"; the raw laser gives {raw_height:.2f} m"
    if readings.tape_height_metres is None:
        return HeightOutcome(line + ".", using or raw_height is not None, "")
    line += f"; tape {readings.tape_height_metres:.2f} m."
    problems = []
    if readings.tilt_degrees > LARGEST_TILT_WHILE_HELD_DEGREES:
        problems.append(f"{readings.place()} the drone was tilted {readings.tilt_degrees:.0f} degrees; hold it level.")
    if using and abs(px4_height - readings.tape_height_metres) > LARGEST_HEIGHT_ERROR_METRES:
        problems.append(f"{readings.place()} PX4 used the laser but dist_bottom gave {px4_height:.2f} m, "
                        f"{abs(px4_height - readings.tape_height_metres) * 100:.0f} cm off.")
    if raw_height is not None and abs(raw_height - readings.tape_height_metres) > LARGEST_HEIGHT_ERROR_METRES:
        problems.append(f"{readings.place()} the raw laser read {raw_height:.2f} m, "
                        f"{abs(raw_height - readings.tape_height_metres) * 100:.0f} cm off.")
    return HeightOutcome(line, using or raw_height is not None, " ".join(problems))


def judge_laser_heights(floor_readings, held_readings):
    """The whole test's verdict, from the floor and every held height."""
    floor_outcome = judge_one_height(floor_readings)
    held_outcomes = [judge_one_height(readings) for readings in held_readings]
    lines = [floor_outcome.line] + [outcome.line for outcome in held_outcomes]
    heights_using_laser = [readings.tape_height_metres for readings in held_readings if readings.px4_uses_the_laser()]
    measurements = {"what_each_height_showed": lines,
                    "heights_where_px4_used_the_laser_metres": heights_using_laser,
                    "px4_declared_the_laser_faulty": any(r.px4_declared_laser_faulty for r in [floor_readings] + held_readings)}
    problems = [outcome.problem for outcome in held_outcomes if outcome.problem]
    if not any(outcome.laser_was_seen for outcome in held_outcomes):
        faulty = " PX4 declared the laser faulty (cs_rng_fault)." if measurements["px4_declared_the_laser_faulty"] else ""
        return failed("PX4 never used the laser at any held height (cs_rng_hgt and cs_rng_terrain stayed false) and "
                      "no raw laser reading arrived, so dist_bottom came from the barometer and we can't tell whether "
                      "the laser works." + faulty + " In QGroundControl, check that the laser sends readings (MAVLink "
                      "Inspector, DISTANCE_SENSOR) and check the settings EKF2_RNG_CTRL and EKF2_HGT_REF.", **measurements)
    if problems:
        return failed(" ".join(problems), **measurements)
    lowest = f"{min(heights_using_laser):.2f} m" if heights_using_laser else "no held height (only the raw reading)"
    return passed(f"The laser works: wherever it was in use it matched the tape within "
                  f"{LARGEST_HEIGHT_ERROR_METRES * 100:.0f} cm. PX4 used it from {lowest} up "
                  f"(at {', '.join(f'{h:.2f}' for h in heights_using_laser) or 'none'} m).", **measurements)


# ---- Running it on the drone -------------------------------------------------------------------

def run(listener, person, log):
    problem = px4_silence_problem(listener.reports.seconds_since("px4_status"))
    if problem:
        return problem
    person.tell("This test checks the bottom laser against a tape measure. Measure each height from the "
                "laser's lens, on the drone's underside, straight down to the floor.")
    person.do_this_then_press_enter("Stand the drone on its legs on clear, flat floor, and let go of it.")
    floor_readings = collect_readings(listener, log, None)
    person.tell(judge_one_height(floor_readings).line)
    held_readings = []
    for height_metres in HELD_HEIGHTS_METRES:
        person.do_this_then_press_enter(
            f"Hold the drone so the bottom laser is {height_metres:.2f} m above the floor (measure with a tape), "
            f"keep it level and still, then press Enter and hold it there for {SECONDS_AT_EACH_HEIGHT:.0f} more seconds.")
        readings = collect_readings(listener, log, height_metres)
        person.tell(judge_one_height(readings).line)
        held_readings.append(readings)
    return judge_laser_heights(floor_readings, held_readings)


def collect_readings(listener, log, tape_height_metres):
    """Listens for SECONDS_AT_EACH_HEIGHT and records PX4's height reports and any raw laser
    readings, with PX4's flags."""
    start_seconds = listener.now()
    listener.wait(SECONDS_AT_EACH_HEIGHT)
    tilt = listener.tilt_degrees_now()
    distances = []
    flags = []   # True if PX4 used the laser for height or for the ground, None if unknown.
    faulty = False
    for sample in listener.px4_heights_since(start_seconds):
        distances.append(sample.distance_to_ground_metres)
        flags.append(None if sample.uses_laser_for_height is None
                     else bool(sample.uses_laser_for_height or sample.uses_laser_for_ground))
        faulty = faulty or bool(sample.laser_declared_faulty)
        log.sample("px4_height", tape_height_metres=tape_height_metres, dist_bottom_metres=sample.distance_to_ground_metres,
                   z_metres=sample.z_metres, uses_laser_for_height=sample.uses_laser_for_height,
                   uses_laser_for_ground=sample.uses_laser_for_ground, laser_declared_faulty=sample.laser_declared_faulty)
    raw = []
    for reading in listener.laser_since(start_seconds):
        raw.append(reading.distance_metres)
        log.sample("raw_laser", tape_height_metres=tape_height_metres, raw_metres=reading.distance_metres,
                   signal_quality_percent=reading.signal_quality_percent)
    log.event("height", f"Collected {len(distances)} PX4 reports and {len(raw)} raw laser readings at tape height "
                        f"{tape_height_metres}.", tape_height_metres=tape_height_metres, tilt_degrees=round(tilt, 2))
    return ReadingsAtOneHeight(tape_height_metres, distances, flags, faulty, raw, tilt)
