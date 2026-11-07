"""A simple test flight: start, climb to 0.8 m, fly 1 m forward, and land by itself.

SIMULATOR TEST. Do not run this on the real drone: it asks for the simulator's start service,
and the real landing logic will live in the flight code instead.

Why this test exists
--------------------
The plan's landing design comes from reading PX4's source code and doing arithmetic. This test
runs it against PX4's real land detector, the part of PX4 that decides the drone has landed, so
that we can see whether the design works, and how long it takes. The results so far are in
section 7 of docs/ARCHITECTURE.md. It has not been tried on the real drone.

The problem it tests
--------------------
Our code cannot switch the motors off. Only PX4 can, once it decides the drone has landed. To
decide that, PX4 needs to see the motors at low power. After touching the floor they stay near
hover power and only come down gradually, because PX4 keeps trying to reach the commanded
descent speed. The plan says: once the bottom laser shows we have touched the floor, command a
faster descent. The floor stops the drone from moving, but the motor power falls more quickly.

    --with-push-down      Use the plan's design: command 1 m/s down after touchdown.
    --without-push-down   Keep commanding the gentle descent speed. This is for comparison.

This program uses only what real flight code may use: the same topics, the same rules.
The single exception is the call to /simulator_only/start_run, which stands in for a person.
"""
import argparse
import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Our own choice. The organizers' program ignores a speed target older than 1 second, so 10
# targets per second leaves plenty of margin.
SETPOINTS_PER_SECOND = 10.0

# Section 1 of docs/ARCHITECTURE.md explains why we fly at this height.
CRUISE_HEIGHT_METRES = 0.8

# Our own choices for this test. They decide when the test moves on to its next step.
HEIGHT_TOLERANCE_METRES = 0.10
FORWARD_DISTANCE_METRES = 1.0
POSITION_TOLERANCE_METRES = 0.10
SECONDS_TO_SETTLE_AT_EACH_POINT = 2.0

# The descent plan, from section 7 of docs/ARCHITECTURE.md.
MEDIUM_DESCENT_METRES_PER_SECOND = 0.5
GENTLE_DESCENT_METRES_PER_SECOND = 0.35
PUSH_DOWN_METRES_PER_SECOND = 1.0
HEIGHT_TO_START_GENTLE_DESCENT_METRES = 0.30

# Touchdown means: the laser is this close to its reading on the launch mark, and the drone
# is not moving down, for this long.
TOUCHDOWN_LASER_TOLERANCE_METRES = 0.03
TOUCHDOWN_LARGEST_DOWNWARD_SPEED_METRES_PER_SECOND = 0.05
TOUCHDOWN_SECONDS_REQUIRED = 0.3

# How long each part of the test may take before we call it a failure.
LONGEST_WAIT_FOR_SIMULATOR_SECONDS = 180.0
LONGEST_FLIGHT_STAGE_SECONDS = 20.0
LONGEST_WAIT_FOR_MOTORS_OFF_SECONDS = 40.0

PX4_MESSAGE_SETTINGS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class LandingTest(Node):

    def __init__(self, use_push_down):
        super().__init__("landing_test")
        self.use_push_down = use_push_down
        self.drone_state = None
        self.px4_position = None
        self.pose_publisher = self.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
        self.twist_publisher = self.create_publisher(TwistStamped, "/skybeta/setpoint/twist", 10)
        self.create_subscription(String, "/skybeta/state", self.on_drone_state, 10)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            self.on_px4_position, PX4_MESSAGE_SETTINGS,
        )
        self.start_service = self.create_client(Trigger, "/simulator_only/start_run")

    def on_drone_state(self, message):
        self.drone_state = json.loads(message.data)

    def on_px4_position(self, message):
        self.px4_position = message

    def say(self, text):
        print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)

    # ---- Reading the drone's condition -------------------------------------------------------

    def laser_reading_metres(self):
        """Returns PX4's distance to the ground, or None if PX4 has no height estimate yet.

        Our flight code cannot read the laser itself on the real drone. PX4 reports this value,
        which is based on the laser. "dist_bottom" means distance to the ground below.

        We deliberately ignore the flag "dist_bottom_valid" that sits next to this value. With
        drone 16's settings it stays false for the whole flight. PX4 only sets it when it uses
        the laser to work out the height of the ground, and drone 16 uses the laser as its main
        height source instead (EKF2_HGT_REF = 2). See terrain_control.cpp in PX4's source code.
        So we check "z_valid", which says that PX4's height estimate is working.
        """
        if self.px4_position is None or not self.px4_position.z_valid:
            return None
        return float(self.px4_position.dist_bottom)

    def downward_speed_metres_per_second(self):
        return float(self.px4_position.vz)  # In PX4's directions, z points downward.

    def tracked_position(self):
        """Returns the camera tracking's current position as (forward, left, up)."""
        return self.drone_state["odom_enu"]

    def drone_is_ready_to_start(self):
        state = self.drone_state
        return (
            state is not None and state["px4_link"] and state["landed"] is True
            and state["odom_enu"] is not None and state["local_enu"] is not None
            and self.laser_reading_metres() is not None
        )

    # ---- Sending targets ---------------------------------------------------------------------

    def send_target_position(self, forward_metres, left_metres, up_metres):
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "odom"
        target.pose.position.x = float(forward_metres)
        target.pose.position.y = float(left_metres)
        target.pose.position.z = float(up_metres)
        # An orientation of all zeros tells the organizers' code to hold the current heading.
        target.pose.orientation.w = 0.0
        self.pose_publisher.publish(target)

    def send_downward_speed(self, metres_per_second):
        target = TwistStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "odom"
        target.twist.linear.z = -float(metres_per_second)
        self.twist_publisher.publish(target)

    # ---- Waiting -----------------------------------------------------------------------------

    def run_until(self, is_finished, send_target, longest_wait_seconds, what_we_are_waiting_for):
        """Sends a target 10 times per second until is_finished() is true. Stops the test if it
        takes too long. This is how the test does everything, so that targets never stop."""
        start_time = time.monotonic()
        while not is_finished():
            if time.monotonic() - start_time > longest_wait_seconds:
                raise TimeoutError(
                    f"Waited {longest_wait_seconds:.0f} seconds for {what_we_are_waiting_for}, "
                    f"and it did not happen. The last drone state was: {self.drone_state}"
                )
            if send_target is not None:
                send_target()
            rclpy.spin_once(self, timeout_sec=1.0 / SETPOINTS_PER_SECOND)

    def run_for_seconds(self, seconds, send_target):
        end_time = time.monotonic() + seconds
        self.run_until(lambda: time.monotonic() > end_time, send_target, seconds + 1.0, "a timed wait")

    # ---- The test, step by step --------------------------------------------------------------

    def run(self):
        self.refuse_to_run_outside_the_simulator()
        self.wait_for_simulator()
        self.remember_the_floor()
        self.press_start()
        self.climb()
        self.fly_forward()
        time_of_touchdown = self.descend_until_touchdown()
        self.wait_for_motors_off(time_of_touchdown)

    def refuse_to_run_outside_the_simulator(self):
        # This service only exists in the simulator. We check for it before sending any target,
        # so that running this file on the real drone by mistake does nothing at all.
        if not self.start_service.wait_for_service(timeout_sec=LONGEST_WAIT_FOR_SIMULATOR_SECONDS):
            raise TimeoutError(
                "The service /simulator_only/start_run did not appear, so this does not look like "
                "the simulator. Nothing was sent. Use simulator/run_test_flight.sh to run this test."
            )

    def wait_for_simulator(self):
        self.say("Waiting for PX4, the camera tracking and the laser to be ready.")
        self.run_until(self.drone_is_ready_to_start, None, LONGEST_WAIT_FOR_SIMULATOR_SECONDS,
                       "the simulator to become ready")
        self.say("The simulator is ready.")
        # Give the height estimate time to settle after it first becomes available.
        self.run_for_seconds(5.0, None)

    def remember_the_floor(self):
        self.floor_laser_reading_metres = self.laser_reading_metres()
        self.start_position = list(self.tracked_position())
        self.say(f"On the launch mark, PX4's distance to the ground reads "
                 f"{self.floor_laser_reading_metres:.3f} m. This is our floor reference.")

    def hold_above_start(self):
        self.send_target_position(self.start_position[0], self.start_position[1], CRUISE_HEIGHT_METRES)

    def press_start(self):
        self.say("Sending targets first, because the organizers' code refuses to start without them.")
        self.run_for_seconds(2.0, self.hold_above_start)

        reply_future = self.start_service.call_async(Trigger.Request())
        self.run_until(reply_future.done, self.hold_above_start, 5.0, "the start service to reply")
        self.say("Start pressed. Waiting for the hand-over to automatic flight.")
        self.run_until(lambda: self.drone_state["offboard_active"], self.hold_above_start, 10.0,
                       "the drone to enter automatic flight (Offboard)")
        self.say("The drone is in automatic flight.")

    def climb(self):
        def reached_height():
            return abs(self.tracked_position()[2] - CRUISE_HEIGHT_METRES) < HEIGHT_TOLERANCE_METRES

        self.run_until(reached_height, self.hold_above_start, LONGEST_FLIGHT_STAGE_SECONDS, "the climb")
        self.say(f"Reached {self.tracked_position()[2]:.2f} m.")
        self.run_for_seconds(SECONDS_TO_SETTLE_AT_EACH_POINT, self.hold_above_start)

    def fly_forward(self):
        target_forward = self.start_position[0] + FORWARD_DISTANCE_METRES

        def hold_at_target():
            self.send_target_position(target_forward, self.start_position[1], CRUISE_HEIGHT_METRES)

        def reached_target():
            return abs(self.tracked_position()[0] - target_forward) < POSITION_TOLERANCE_METRES

        self.run_until(reached_target, hold_at_target, LONGEST_FLIGHT_STAGE_SECONDS, "the forward flight")
        self.run_for_seconds(SECONDS_TO_SETTLE_AT_EACH_POINT, hold_at_target)
        self.say(f"Flew forward. The tracked position is {[round(v, 2) for v in self.tracked_position()]}.")

    def height_above_floor_metres(self):
        laser = self.laser_reading_metres()
        return None if laser is None else laser - self.floor_laser_reading_metres

    def descend_until_touchdown(self):
        self.say("Descending.")

        def has_reached_gentle_height():
            height = self.height_above_floor_metres()
            return height is not None and height < HEIGHT_TO_START_GENTLE_DESCENT_METRES

        self.run_until(has_reached_gentle_height,
                       lambda: self.send_downward_speed(MEDIUM_DESCENT_METRES_PER_SECOND),
                       LONGEST_FLIGHT_STAGE_SECONDS, "the medium descent")

        self.time_touchdown_conditions_began = None

        def has_touched_down():
            height = self.height_above_floor_metres()
            conditions_are_met = (
                height is not None
                and height < TOUCHDOWN_LASER_TOLERANCE_METRES
                and self.downward_speed_metres_per_second() < TOUCHDOWN_LARGEST_DOWNWARD_SPEED_METRES_PER_SECOND
            )
            if not conditions_are_met:
                self.time_touchdown_conditions_began = None
                return False
            if self.time_touchdown_conditions_began is None:
                self.time_touchdown_conditions_began = time.monotonic()
            return time.monotonic() - self.time_touchdown_conditions_began >= TOUCHDOWN_SECONDS_REQUIRED

        self.run_until(has_touched_down,
                       lambda: self.send_downward_speed(GENTLE_DESCENT_METRES_PER_SECOND),
                       LONGEST_FLIGHT_STAGE_SECONDS, "touchdown")
        self.say("Touchdown confirmed by the laser and the stopped descent.")
        return time.monotonic()

    def wait_for_motors_off(self, time_of_touchdown):
        speed_after_touchdown = (
            PUSH_DOWN_METRES_PER_SECOND if self.use_push_down else GENTLE_DESCENT_METRES_PER_SECOND
        )
        self.say(f"Commanding {speed_after_touchdown} m/s down until the motors stop.")
        time_px4_said_landed = None

        def motors_are_off():
            nonlocal time_px4_said_landed
            if time_px4_said_landed is None and self.drone_state["landed"]:
                time_px4_said_landed = time.monotonic()
                self.say(f"PX4 says landed, {time_px4_said_landed - time_of_touchdown:.1f} s after touchdown.")
            return not self.drone_state["armed"]

        self.run_until(motors_are_off, lambda: self.send_downward_speed(speed_after_touchdown),
                       LONGEST_WAIT_FOR_MOTORS_OFF_SECONDS, "the motors to switch off")
        self.say(f"RESULT: the motors switched off {time.monotonic() - time_of_touchdown:.1f} s "
                 f"after touchdown ({'with' if self.use_push_down else 'without'} the push-down step).")


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
