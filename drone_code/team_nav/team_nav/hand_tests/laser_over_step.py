"""Hand test: carry the drone at flying height over the edge of a box, and measure how quickly
the bottom laser's reading changes. That tells us how wide the laser's beam is.

REAL DRONE CODE, run with the PROPELLERS OFF. It only listens to the drone.

Why this matters
----------------
This is our biggest known risk (docs/ARCHITECTURE.md, section 7). PX4, the flight controller's
software, uses the bottom laser as the drone's height. It also checks that the laser's changes
agree with the drone's up-and-down speed. A sudden jump in the reading, such as passing over the
window's lower wall or a boulder, upsets that check, and PX4 can then stop using the laser for
the rest of the flight. How sudden the jump is depends on the beam's width: a narrow beam (about
1 degree) jumps from one reading to the next; a wide one (up to 27 degrees) blends the two
heights over 20 cm or more. Nobody knows which drone 16 has (section 12).

What it measures
----------------
The person holds the drone level at CARRY_HEIGHT_METRES and walks slowly across a box. The test
records the height readings and camera tracking's position. At each edge of the box it finds
the readings between "floor" and "box top", and turns the time they took into a distance using
the walking speed from camera tracking. That distance is roughly the width of the laser's spot
on the floor, and from it the beam's angle. The result is a range, not one number.

Which height readings: the laser's own raw reading if the drone sends it. On 2026-11-07 drone 16
did not (/fmu/out/distance_sensor was missing), so the test then uses PX4's dist_bottom, its
estimate of the distance to the ground below. That only shows the box while PX4 is using the
laser (its flags cs_rng_hgt or cs_rng_terrain), and PX4 smooths it and may refuse a sudden
jump. So from dist_bottom the spot width is only an upper bound, and a step that doesn't show
at all means PX4 didn't follow the laser, not that the laser failed.

The test also records whether PX4 kept using the laser. That is NOT part of the pass rule: PX4
only runs the check that drops the laser while it believes the drone is flying
(range_height_control.cpp in PX4 1.16.2 updates it only "in_air"), and a disarmed drone is
never flying. So this test measures the beam that feeds the check, not the check itself.
Practice flights over a mock window must test that.

PASS means the measurement worked: the floor and the box top were both seen clearly, the step
between them matched the box, and at least one edge was crossed while tracking measured the speed.
"""
import math
import statistics
from dataclasses import dataclass

from team_nav.hand_tests.hand_test_tools import failed, passed, px4_silence_problem

# From drone 16's settings file of 2026-11-03 (EKF2_RNG_POS_Z): the laser sits this far below
# the drone's centre, and PX4's dist_bottom is measured from the centre.
LASER_BELOW_CENTRE_METRES = 0.03445
# Our own choice: fewer raw readings than this means the drone doesn't send them.
FEWEST_RAW_READINGS = 5

# Our own choice: the drone's cruise height (section 1 of docs/ARCHITECTURE.md).
CARRY_HEIGHT_METRES = 0.80
# Our own choice. A reading more than this fraction of the step away from the floor reading, and
# more than this fraction away from the box-top reading, is "in between": the beam sees both.
BAND_FRACTION_OF_STEP = 0.2
# Our own choice: how many first readings, taken before the box, give the floor reading.
READINGS_FOR_THE_FLOOR_LEVEL = 5
# Our own choice: the box top must be seen at least this many times to measure it.
FEWEST_READINGS_ON_THE_BOX = 3
# Our own choice. A tape held by hand and a drone held by hand are each good to about 2 cm.
LARGEST_STEP_ERROR_METRES = 0.05
# Our own choice. Slower than this, tracking's speed is mostly noise.
SLOWEST_USEFUL_SPEED_METRES_PER_SECOND = 0.05
# Our own choice: how much tracking around an edge is used to work out the walking speed.
SECONDS_OF_TRACKING_AROUND_AN_EDGE = 0.5


@dataclass
class HeightReading:
    """One reading of the distance below the laser, from whichever source the test uses."""
    arrival_seconds: float
    distance_metres: float


@dataclass
class EdgeCrossing:
    name: str                       # "onto the box" or "off the box".
    readings_in_between: int
    shortest_seconds: float         # The change took at least this long ...
    longest_seconds: float          # ... and at most this long (readings are about 0.2 s apart).
    speed_metres_per_second: float  # None if tracking couldn't say.

    def spot_width_range_metres(self):
        if self.speed_metres_per_second is None:
            return None
        return self.shortest_seconds * self.speed_metres_per_second, self.longest_seconds * self.speed_metres_per_second


@dataclass
class StepAnalysis:
    floor_reading_metres: float
    box_top_reading_metres: float   # None if never seen.
    readings_on_the_box: int
    crossings: list


def classify(distance_metres, floor_reading_metres, step_metres):
    """'floor', 'box' or 'between'."""
    if distance_metres > floor_reading_metres - BAND_FRACTION_OF_STEP * step_metres:
        return "floor"
    if distance_metres < floor_reading_metres - (1.0 - BAND_FRACTION_OF_STEP) * step_metres:
        return "box"
    return "between"


def analyse_step(laser_readings, tracking_reports, box_height_metres):
    """laser_readings and tracking_reports are drone_listener.py's LaserReading and
    TrackingReport objects, oldest first. Returns a StepAnalysis, or None if there were too few
    readings to find the floor."""
    if len(laser_readings) < READINGS_FOR_THE_FLOOR_LEVEL:
        return None
    floor_reading = statistics.median(reading.distance_metres for reading in laser_readings[:READINGS_FOR_THE_FLOOR_LEVEL])
    kinds = [classify(reading.distance_metres, floor_reading, box_height_metres) for reading in laser_readings]
    on_box = [reading.distance_metres for reading, kind in zip(laser_readings, kinds) if kind == "box"]
    box_top_reading = statistics.median(on_box) if on_box else None
    crossings = []
    for name, from_kind, to_kind in (("onto the box", "floor", "box"), ("off the box", "box", "floor")):
        crossing = find_crossing(name, laser_readings, kinds, from_kind, to_kind, tracking_reports)
        if crossing is not None:
            crossings.append(crossing)
    return StepAnalysis(floor_reading, box_top_reading, len(on_box), crossings)


def find_crossing(name, laser_readings, kinds, from_kind, to_kind, tracking_reports):
    """The first change from from_kind to to_kind, with any 'between' readings in the middle."""
    last_from_index = None
    for index, kind in enumerate(kinds):
        if kind == from_kind:
            last_from_index = index
        elif kind == to_kind and last_from_index is not None:
            first_to_index = index
            in_between = first_to_index - last_from_index - 1
            longest = laser_readings[first_to_index].arrival_seconds - laser_readings[last_from_index].arrival_seconds
            shortest = 0.0
            if in_between > 0:
                shortest = laser_readings[first_to_index - 1].arrival_seconds - laser_readings[last_from_index + 1].arrival_seconds
            speed = walking_speed(tracking_reports, laser_readings[last_from_index].arrival_seconds,
                                  laser_readings[first_to_index].arrival_seconds)
            return EdgeCrossing(name, in_between, shortest, longest, speed)
    return None


def walking_speed(tracking_reports, start_seconds, end_seconds):
    """Sideways speed from camera tracking around this time, or None if too few reports."""
    nearby = []
    for report in tracking_reports:
        if start_seconds - SECONDS_OF_TRACKING_AROUND_AN_EDGE <= report.arrival_seconds <= end_seconds + SECONDS_OF_TRACKING_AROUND_AN_EDGE:
            nearby.append(report)
    if len(nearby) < 2 or nearby[-1].arrival_seconds <= nearby[0].arrival_seconds:
        return None
    distance = math.dist((nearby[0].forward_metres, nearby[0].left_metres), (nearby[-1].forward_metres, nearby[-1].left_metres))
    return distance / (nearby[-1].arrival_seconds - nearby[0].arrival_seconds)


def beam_angle_degrees(spot_width_metres, height_metres):
    """The full angle of a cone that makes a spot this wide at this distance."""
    return 2.0 * math.degrees(math.atan(spot_width_metres / (2.0 * height_metres)))


def describe_crossing(crossing, height_metres):
    widths = crossing.spot_width_range_metres()
    if widths is None:
        return f"Crossing {crossing.name}: tracking couldn't give the walking speed."
    low_angle, high_angle = beam_angle_degrees(widths[0], height_metres), beam_angle_degrees(widths[1], height_metres)
    return (f"Crossing {crossing.name} at {crossing.speed_metres_per_second:.2f} m/s took {crossing.shortest_seconds:.2f} "
            f"to {crossing.longest_seconds:.2f} s with {crossing.readings_in_between} readings in between: a spot "
            f"{widths[0] * 100:.0f} to {widths[1] * 100:.0f} cm wide, a beam of about {low_angle:.0f} to {high_angle:.0f} degrees.")


def judge_step(analysis, box_height_metres, from_px4_estimate=False):
    """from_px4_estimate is True when the readings are PX4's dist_bottom, not the raw laser."""
    source = "PX4's dist_bottom" if from_px4_estimate else "the raw laser"
    if analysis is None:
        return failed(f"Fewer than {READINGS_FOR_THE_FLOOR_LEVEL} readings came from {source}, so the floor couldn't "
                      f"be measured.")
    if analysis.readings_on_the_box < FEWEST_READINGS_ON_THE_BOX:
        if from_px4_estimate:
            return failed("PX4's dist_bottom never showed the box. Either PX4 was not using the laser, or it refused "
                          "the sudden step, or the person walked too fast (about 1 m every 5 seconds is right). This "
                          "does not show whether the laser saw it.")
        return failed(f"The laser saw the box top only {analysis.readings_on_the_box} times. Walk more slowly "
                      f"(about 1 m every 5 seconds) straight over the box's middle.")
    step = analysis.floor_reading_metres - analysis.box_top_reading_metres
    if abs(step - box_height_metres) > LARGEST_STEP_ERROR_METRES:
        return failed(f"The step in {source} was {step:.2f} m but the box is {box_height_metres:.2f} m tall. Keep the "
                      f"drone at the same height while walking, and check the box height.")
    usable = [crossing for crossing in analysis.crossings
              if crossing.speed_metres_per_second is not None
              and crossing.speed_metres_per_second >= SLOWEST_USEFUL_SPEED_METRES_PER_SECOND]
    if not usable:
        return failed("No edge was crossed while camera tracking measured a walking speed, so the beam width "
                      "couldn't be worked out. Keep walking steadily across both edges.")
    sentences = [describe_crossing(crossing, analysis.floor_reading_metres) for crossing in usable]
    if from_px4_estimate:
        sentences.append("These come from PX4's smoothed estimate, so the true spot is at most this wide.")
    return passed(f"The step of {step:.2f} m in {source} matched the box. " + " ".join(sentences),
                  floor_reading_metres=round(analysis.floor_reading_metres, 3),
                  box_top_reading_metres=round(analysis.box_top_reading_metres, 3),
                  crossings=[crossing.__dict__ for crossing in analysis.crossings])


def describe_px4_laser_flag(flag_samples):
    """One sentence about whether PX4 used the laser during the carry. flag_samples are True
    (cs_rng_hgt or cs_rng_terrain set), False, or None (flags not yet arrived)."""
    known = [flag for flag in flag_samples if flag is not None]
    if not known:
        return "PX4's estimator flags never arrived, so we don't know whether it used the laser."
    if all(known):
        return ("PX4 used the laser throughout (cs_rng_hgt or cs_rng_terrain stayed true). By hand this is expected, "
                "because PX4 only runs its laser check in flight.")
    if not any(known):
        return "PX4 never used the laser during the carry (cs_rng_hgt and cs_rng_terrain stayed false)."
    return (f"PX4 was not using the laser in {known.count(False)} of {len(known)} reports, even by hand. "
            f"Look at px4_height.csv to see when.")


# ---- Running it on the drone -------------------------------------------------------------------

def run(listener, person, log):
    problem = px4_silence_problem(listener.reports.seconds_since("px4_status"))
    if problem:
        return problem
    person.tell("You need a box at least 40 cm long and 15 to 40 cm tall, on clear flat floor, with 1.5 m of "
                "clear floor before and after it.")
    box_height_metres = person.type_a_number("Measure the box's height in metres and type it (for example 0.30):")
    person.do_this_then_press_enter(
        f"Stand 1.5 m before the box. Hold the drone level with the bottom laser {CARRY_HEIGHT_METRES:.2f} m above the "
        f"floor, nose pointing at the box. After pressing Enter, wait 2 seconds, then walk slowly (about 1 m every "
        f"5 seconds) straight over the middle of the box and 1 m past it, keeping the drone at the same height.")
    start_seconds = listener.now()
    person.do_this_then_press_enter("When you are 1 m past the box, stop and press Enter.")
    readings, from_px4_estimate, flag_samples = choose_and_record_readings(listener, log, start_seconds)
    analysis = analyse_step(readings, listener.tracking_since(start_seconds), box_height_metres)
    verdict = judge_step(analysis, box_height_metres, from_px4_estimate)
    flag_sentence = describe_px4_laser_flag(flag_samples)
    person.tell(flag_sentence)
    verdict.measurements.update(box_height_metres=box_height_metres, px4_laser_flag=flag_sentence,
                                readings_from="PX4 dist_bottom" if from_px4_estimate else "raw laser")
    return verdict


def choose_and_record_readings(listener, log, start_seconds):
    """Saves every PX4 height report and raw laser reading since start_seconds. Returns the
    readings to analyse (the raw laser if it arrived, otherwise PX4's dist_bottom), whether they
    are PX4's, and PX4's laser flag for each PX4 report."""
    px4_readings = []
    flag_samples = []
    for sample in listener.px4_heights_since(start_seconds):
        uses_laser = None
        if sample.uses_laser_for_height is not None:
            uses_laser = bool(sample.uses_laser_for_height or sample.uses_laser_for_ground)
        flag_samples.append(uses_laser)
        px4_readings.append(HeightReading(sample.arrival_seconds, sample.distance_to_ground_metres - LASER_BELOW_CENTRE_METRES))
        log.sample("px4_height", dist_bottom_metres=sample.distance_to_ground_metres, z_metres=sample.z_metres,
                   px4_uses_laser=uses_laser, laser_declared_faulty=sample.laser_declared_faulty)
    raw_readings = []
    for reading in listener.laser_since(start_seconds):
        raw_readings.append(HeightReading(reading.arrival_seconds, reading.distance_metres))
        log.sample("raw_laser", raw_metres=reading.distance_metres, signal_quality_percent=reading.signal_quality_percent)
    if len(raw_readings) >= FEWEST_RAW_READINGS:
        return raw_readings, False, flag_samples
    return px4_readings, True, flag_samples
