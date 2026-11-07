"""Works out how quickly the drone follows its target, from the positions recorded during the
out-and-back practice flight (out_and_back.py).

REAL DRONE CODE, also used unchanged in the simulator. It is plain arithmetic on lists of
numbers, with no ROS, so it is tested on its own (test/test_practice_flights_measurements.py).

The two measurements
--------------------
docs/ARCHITECTURE.md, section 9, models PX4's position control, seen from outside, as a
first-order low-pass filter: the drone moves towards its target like a capacitor charging
towards a new voltage through a resistor. Such a system is described by one number, its time
constant. We measure it two ways, which should roughly agree:

  1. The step. The target jumps 1 m forward at once. The response delay is the time from the
     jump until the drone has covered 63.2% of the step, which for a first-order system is one
     time constant. We also note how long until the drone stays within 5 cm of the target,
     and how far it overshoots.
  2. The slide. The target moves back at a steady speed. A first-order system then runs a
     steady distance behind it: the speed times the time constant. We report that trailing
     distance, and the same thing in seconds (distance divided by speed), which should be
     close to the step's response delay.

Positions are "forward" distances in metres from the launch mark, as camera tracking reports
them, because camera tracking is what our program steers by. So the delay includes camera
tracking's own delay, which is the delay our program really lives with.

What this file does not do: fly, read reports, or decide anything about safety.
"""
import math
from dataclasses import dataclass

# From the definition of a first-order system's time constant: after one time constant it has
# covered 1 - 1/e of a step, about 63.2%.
FRACTION_COVERED_AFTER_ONE_TIME_CONSTANT = 1.0 - math.exp(-1.0)

# Our own choice, the same as route_follower.py's corner tolerance: "settled" means within 5 cm.
SETTLED_TOLERANCE_METRES = 0.05

# Our own choice. The trailing distance is averaged over the second half of the slide only,
# because during the first half the drone is still getting up to speed.
FRACTION_OF_SLIDE_BEFORE_TRAILING_IS_STEADY = 0.5


@dataclass
class PositionSample:
    """One cycle's record: when, where the target was, and where camera tracking put the drone."""
    seconds: float
    target_forward_metres: float
    drone_forward_metres: float


@dataclass
class StepResponse:
    step_metres: float                       # How far the target jumped (negative means backwards).
    delay_to_63_percent_seconds: object      # None if the drone never covered 63.2% of the step.
    seconds_until_settled: object            # None if it never stayed within SETTLED_TOLERANCE_METRES.
    overshoot_metres: float                  # How far past the target it went; 0 if it never did.

    def describe(self):
        return (f"Step of {self.step_metres:+.2f} m: response delay (63% of the step) {_seconds_text(self.delay_to_63_percent_seconds)}, "
                f"settled within {SETTLED_TOLERANCE_METRES * 100:.0f} cm after {_seconds_text(self.seconds_until_settled)}, "
                f"overshoot {self.overshoot_metres:.3f} m.")


@dataclass
class TrailingDistance:
    slide_speed_metres_per_second: float
    average_trailing_metres: object          # Over the steady part of the slide. None if no samples there.
    largest_trailing_metres: object          # Over the whole slide.
    trailing_seconds: object                 # average_trailing_metres / speed: comparable with the step's delay.

    def describe(self):
        if self.average_trailing_metres is None:
            return "Slide: not enough samples to measure how far the drone trails its target."
        return (f"Slide at {self.slide_speed_metres_per_second:.2f} m/s: the drone ran {self.average_trailing_metres:.3f} m "
                f"behind its target on average once steady ({self.trailing_seconds:.2f} s of target movement), "
                f"at most {self.largest_trailing_metres:.3f} m.")


def measure_step_response(samples, step_start_seconds, start_forward_metres, target_forward_metres):
    """samples are PositionSamples from the moment the target jumped onwards, in time order.
    start_forward_metres is where the drone was when the target jumped."""
    step_metres = target_forward_metres - start_forward_metres
    if step_metres == 0.0:
        raise ValueError("The step has no length, so there is no response to measure. This is a bug in the flight.")
    return StepResponse(
        step_metres=step_metres,
        delay_to_63_percent_seconds=_time_to_cover_fraction(samples, step_start_seconds, start_forward_metres, step_metres),
        seconds_until_settled=_time_until_settled(samples, step_start_seconds, target_forward_metres),
        overshoot_metres=_overshoot_metres(samples, target_forward_metres, step_metres),
    )


def measure_trailing_distance(samples, slide_start_seconds, slide_end_seconds, slide_forward_metres_per_second):
    """samples cover the slide, in time order. The target moves forward at
    slide_forward_metres_per_second, which is negative for a slide backwards. "Trailing" is
    measured along the way the target moves, so it is positive when the drone is behind."""
    if slide_forward_metres_per_second == 0.0:
        raise ValueError("The slide speed must not be zero. This is a bug in the flight.")
    slide_direction = 1.0 if slide_forward_metres_per_second > 0 else -1.0
    slide_speed_metres_per_second = abs(slide_forward_metres_per_second)
    steady_from_seconds = slide_start_seconds + FRACTION_OF_SLIDE_BEFORE_TRAILING_IS_STEADY * (slide_end_seconds - slide_start_seconds)
    all_gaps_metres = []
    steady_gaps_metres = []
    for sample in samples:
        if not slide_start_seconds <= sample.seconds <= slide_end_seconds:
            continue
        gap_metres = (sample.target_forward_metres - sample.drone_forward_metres) * slide_direction
        all_gaps_metres.append(gap_metres)
        if sample.seconds >= steady_from_seconds:
            steady_gaps_metres.append(gap_metres)
    if not steady_gaps_metres:
        return TrailingDistance(slide_speed_metres_per_second, None, None, None)
    average_metres = sum(steady_gaps_metres) / len(steady_gaps_metres)
    return TrailingDistance(slide_speed_metres_per_second, average_metres, max(all_gaps_metres),
                            average_metres / slide_speed_metres_per_second)


# ---- The pieces ---------------------------------------------------------------------------------

def _time_to_cover_fraction(samples, step_start_seconds, start_forward_metres, step_metres):
    """The first time the drone has covered FRACTION_COVERED_AFTER_ONE_TIME_CONSTANT of the step,
    from the step's start. Between two samples it assumes the drone moved in a straight line, so
    the answer is finer than the 0.1 s between samples."""
    previous = None
    for sample in samples:
        fraction_now = (sample.drone_forward_metres - start_forward_metres) / step_metres
        if fraction_now >= FRACTION_COVERED_AFTER_ONE_TIME_CONSTANT:
            if previous is None:
                return sample.seconds - step_start_seconds
            fraction_before = (previous.drone_forward_metres - start_forward_metres) / step_metres
            share_of_interval = (FRACTION_COVERED_AFTER_ONE_TIME_CONSTANT - fraction_before) / (fraction_now - fraction_before)
            crossing_seconds = previous.seconds + share_of_interval * (sample.seconds - previous.seconds)
            return crossing_seconds - step_start_seconds
        previous = sample
    return None


def _time_until_settled(samples, step_start_seconds, target_forward_metres):
    """The time of the first sample after which every sample is within SETTLED_TOLERANCE_METRES
    of the target. None if the last sample is still outside it."""
    settled_since_seconds = None
    for sample in samples:
        inside = abs(sample.drone_forward_metres - target_forward_metres) <= SETTLED_TOLERANCE_METRES
        if not inside:
            settled_since_seconds = None
        elif settled_since_seconds is None:
            settled_since_seconds = sample.seconds
    if settled_since_seconds is None:
        return None
    return settled_since_seconds - step_start_seconds


def _overshoot_metres(samples, target_forward_metres, step_metres):
    direction = 1.0 if step_metres > 0 else -1.0
    overshoot_metres = 0.0
    for sample in samples:
        beyond_target_metres = (sample.drone_forward_metres - target_forward_metres) * direction
        overshoot_metres = max(overshoot_metres, beyond_target_metres)
    return overshoot_metres


def _seconds_text(seconds):
    return "never" if seconds is None else f"{seconds:.2f} s"
