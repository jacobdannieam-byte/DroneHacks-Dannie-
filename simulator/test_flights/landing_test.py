"""A simple test flight: start, climb to 0.8 m, fly 1 m forward, and land by itself.

SIMULATOR TEST. Do not run this on the real drone: it asks for the simulator's start service,
and the real landing logic will live in the flight code instead.

Why this test exists
--------------------
The plan's landing design comes from reading PX4's source code and doing arithmetic. This test
runs it against PX4's real land detector, the part of PX4 that decides the drone has landed, so
that we can see whether the design works, and how long it takes. The results so far are in
section 7 of docs/ARCHITECTURE.md. It has not been tried on the real drone.

    --with-push-down      Use the plan's design: command 1 m/s down after touchdown.
    --without-push-down   Keep commanding the gentle descent speed. This is for comparison.

simulator_test_flight.py holds the steps every test flight shares, including the landing
itself and why it has a push-down step. This file only adds the short hop forward.
"""
import argparse
import sys

import rclpy

from simulator_test_flight import CRUISE_HEIGHT_METRES, LONGEST_FLIGHT_STAGE_SECONDS, SECONDS_TO_SETTLE_AT_EACH_POINT, SimulatorTestFlight

# Our own choices for this test.
FORWARD_DISTANCE_METRES = 1.0
POSITION_TOLERANCE_METRES = 0.10


class LandingTest(SimulatorTestFlight):

    def __init__(self, use_push_down):
        super().__init__("landing_test")
        self.use_push_down = use_push_down

    def run(self):
        self.refuse_to_run_outside_the_simulator()
        self.wait_for_simulator()
        self.remember_the_floor()
        self.press_start()
        self.climb()
        self.fly_forward()
        time_of_touchdown = self.descend_until_touchdown()
        self.wait_for_motors_off(time_of_touchdown, self.use_push_down)

    def fly_forward(self):
        target_forward = self.start_position[0] + FORWARD_DISTANCE_METRES

        def hold_at_target():
            self.send_target_position(target_forward, self.start_position[1], CRUISE_HEIGHT_METRES)

        def reached_target():
            return abs(self.tracked_position()[0] - target_forward) < POSITION_TOLERANCE_METRES

        self.run_until(reached_target, hold_at_target, LONGEST_FLIGHT_STAGE_SECONDS, "the forward flight")
        self.run_for_seconds(SECONDS_TO_SETTLE_AT_EACH_POINT, hold_at_target)
        self.say(f"Flew forward. The tracked position is {[round(v, 2) for v in self.tracked_position()]}.")


def main():
    parser = argparse.ArgumentParser(description="Fly a short hop and land, in the simulator.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--with-push-down", action="store_true")
    group.add_argument("--without-push-down", action="store_true")
    arguments = parser.parse_args()

    rclpy.init()
    test = LandingTest(use_push_down=arguments.with_push_down)
    try:
        test.run()
    except TimeoutError as problem:
        print(f"TEST FAILED: {problem}", flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
