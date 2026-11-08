"""Flies the real drone's rise-and-land program in the 3D rendered sim. SIMULATOR TEST CODE.

The flight itself is the real program, drone_code/smoke_flights/team_smoke_flights/rise_and_land.py,
unchanged. This file only does what people do on the real drone:

  - It presses start. On the real drone, a teammate arms the drone and flips the switch to
    Position. Here, the simulator's start service stands in for them.
  - It waits for the flight to finish, and says whether it passed.

Run it from the top of the repository:
    simulator/run_test_flight.sh rise_and_land_test.py
"""
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

# The real program's package isn't built inside the simulator's box, so we read it from the
# repository, which the box shares at /repository.
sys.path.insert(0, "/repository/drone_code/smoke_flights")
from team_smoke_flights.rise_and_land import RiseAndLand, Stage  # noqa: E402

# Our own choices, for how long each wait may take before the test fails.
LONGEST_WAIT_FOR_SIMULATOR_SECONDS = 180.0
SECONDS_OF_TARGETS_BEFORE_START = 3.0
LONGEST_FLIGHT_SECONDS = 90.0

RECORDINGS_FOLDER = "/repository/simulator/generated_files/smoke_flight_recordings"


def spin_until(flight, is_finished, longest_wait_seconds, what_we_are_waiting_for):
    end_time = time.monotonic() + longest_wait_seconds
    while not is_finished():
        if time.monotonic() > end_time:
            raise TimeoutError(f"Waited {longest_wait_seconds:.0f} s for {what_we_are_waiting_for}. "
                               f"The flight's stage was: {flight.stage}.")
        rclpy.spin_once(flight, timeout_sec=0.05)


def main():
    rclpy.init()
    flight = RiseAndLand(parameter_overrides=[Parameter("log_folder", value=RECORDINGS_FOLDER)])
    start_service = flight.create_client(Trigger, "/simulator_only/start_run")
    try:
        # This service only exists in the simulator, so this file can never start a real flight.
        if not start_service.wait_for_service(timeout_sec=LONGEST_WAIT_FOR_SIMULATOR_SECONDS):
            raise TimeoutError("The simulator's start service never appeared. Use simulator/run_test_flight.sh.")
        spin_until(flight, flight.is_ready_on_the_ground, LONGEST_WAIT_FOR_SIMULATOR_SECONDS,
                   "the drone to be ready on the ground")

        # Let the program send targets for a while, as it would on the real drone before the flip.
        start_time = time.monotonic()
        spin_until(flight, lambda: time.monotonic() - start_time > SECONDS_OF_TARGETS_BEFORE_START,
                   SECONDS_OF_TARGETS_BEFORE_START + 1.0, "targets to flow")

        flight.say("Pressing start, in place of a teammate arming the drone and flipping the switch.")
        reply = start_service.call_async(Trigger.Request())
        spin_until(flight, reply.done, 10.0, "the start service to reply")
        if not reply.result().success:
            raise TimeoutError(f"The simulator refused start. {reply.result().message}")

        spin_until(flight, lambda: flight.stage in (Stage.FINISHED, Stage.PILOT_TOOK_OVER),
                   LONGEST_FLIGHT_SECONDS, "the flight to finish")
    except TimeoutError as problem:
        print(f"TEST FAILED: {problem}", flush=True)
        sys.exit(1)
    finally:
        flight.recorder.close()

    if flight.stage != Stage.FINISHED:
        print(f"TEST FAILED: the flight ended with '{flight.stage}'.", flush=True)
        sys.exit(1)
    print("TEST PASSED: the drone rose, held, descended and switched its motors off.", flush=True)


if __name__ == "__main__":
    main()
