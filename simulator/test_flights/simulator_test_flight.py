"""The steps every simulator test flight shares: waiting for the simulator, starting, climbing,
landing, and sending targets without a break.

SIMULATOR TEST CODE. Do not run this on the real drone: it asks for the simulator's start
service, and the real start and landing will live in the flight code instead.

landing_test.py and route_test.py each build on the SimulatorTestFlight class below. Each adds
only what happens between the climb and the landing.

These flights use only what real flight code may use: the same topics, the same rules. The
single exception is the call to /simulator_only/start_run, which stands in for a person.

The landing
-----------
Our code cannot switch the motors off. Only PX4 can, once it decides the drone has landed. To
decide that, PX4 needs to see the motors at low power. After touching the floor they stay near
hover power and only come down gradually, because PX4 keeps trying to reach the commanded
descent speed. The plan says: once the bottom laser shows we have touched the floor, command a
faster descent. The floor stops the drone from moving, but the motor power falls more quickly.
landing_test.py compares this with leaving that step out. Section 7 of docs/ARCHITECTURE.md
has the results.
"""
import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import DistanceSensor, VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from team_nav.height_check import laser_height_above_launch_metres
from team_nav.touch_detector import rotation_degrees_per_second, tilt_degrees

# Our own choice. The organizers' program ignores a speed target older than 1 second, so 10
# targets per second leaves plenty of margin.
SETPOINTS_PER_SECOND = 10.0

# Section 1 of docs/ARCHITECTURE.md explains why we fly at this height.
CRUISE_HEIGHT_METRES = 0.8

# Our own choices. They decide when a flight moves on to its next step.
HEIGHT_TOLERANCE_METRES = 0.10
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

# How long each part of a flight may take before we call it a failure.
LONGEST_WAIT_FOR_SIMULATOR_SECONDS = 180.0
LONGEST_FLIGHT_STAGE_SECONDS = 20.0
LONGEST_WAIT_FOR_MOTORS_OFF_SECONDS = 40.0

PX4_MESSAGE_SETTINGS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class SimulatorTestFlight(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.drone_state = None
        self.px4_position = None
        self.laser_message = None
        self.px4_odometry = None
        self.pose_publisher = self.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
        self.twist_publisher = self.create_publisher(TwistStamped, "/skybeta/setpoint/twist", 10)
        self.create_subscription(String, "/skybeta/state", self.on_drone_state, 10)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            self.on_px4_position, PX4_MESSAGE_SETTINGS,
        )
        # The bottom laser's raw reading, which PX4 passes on.
        self.create_subscription(DistanceSensor, "/fmu/out/distance_sensor", self.on_laser_reading, PX4_MESSAGE_SETTINGS)
        # PX4's estimate of the drone's attitude and rotation speed, among other things.
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_px4_odometry, PX4_MESSAGE_SETTINGS)
        self.start_service = self.create_client(Trigger, "/simulator_only/start_run")

    def on_drone_state(self, message):
        self.drone_state = json.loads(message.data)

    def on_px4_position(self, message):
        self.px4_position = message

    def on_laser_reading(self, message):
        self.laser_message = message

    def on_px4_odometry(self, message):
        self.px4_odometry = message

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

    def raw_laser_reading_metres(self):
        """Returns the bottom laser's latest raw reading, measured along the beam, or None if
        none has arrived. Unlike laser_reading_metres, this is not PX4's estimate, so it stays
        right even if PX4 stops trusting the laser."""
        if self.laser_message is None:
            return None
        return float(self.laser_message.current_distance)

    def tilt_degrees_now(self):
        return tilt_degrees(self.px4_odometry.q)  # PX4 stores the quaternion as (w, x, y, z).

    def rotation_degrees_per_second_now(self):
        return rotation_degrees_per_second(self.px4_odometry.angular_velocity)

    def laser_height_above_floor_metres(self):
        """How far the laser has risen since the launch mark, from its raw reading, or None."""
        reading = self.raw_laser_reading_metres()
        if reading is None or self.px4_odometry is None:
            return None
        return laser_height_above_launch_metres(reading, self.floor_raw_laser_reading_metres, self.tilt_degrees_now())

    def px4_height_risen_since_launch_metres(self):
        return self.px4_z_on_launch_mark_metres - float(self.px4_position.z)  # PX4's z points down.

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

    def height_above_floor_metres(self):
        laser = self.laser_reading_metres()
        return None if laser is None else laser - self.floor_laser_reading_metres

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
        """Sends a target 10 times per second until is_finished() is true. Stops the flight if it
        takes too long. This is how every flight does everything, so that targets never stop."""
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

    # ---- Starting ----------------------------------------------------------------------------

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
        self.floor_raw_laser_reading_metres = self.raw_laser_reading_metres()
        self.px4_z_on_launch_mark_metres = float(self.px4_position.z)
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

    # ---- Landing -----------------------------------------------------------------------------

    def descend_until_touchdown(self, height_above_floor=None):
        """height_above_floor is the function that reads the height. Normally it is PX4's
        estimate. After PX4's height has gone wrong, the flight passes in the laser's raw reading."""
        if height_above_floor is None:
            height_above_floor = self.height_above_floor_metres
        self.say("Descending.")

        def has_reached_gentle_height():
            height = height_above_floor()
            return height is not None and height < HEIGHT_TO_START_GENTLE_DESCENT_METRES

        self.run_until(has_reached_gentle_height,
                       lambda: self.send_downward_speed(MEDIUM_DESCENT_METRES_PER_SECOND),
                       LONGEST_FLIGHT_STAGE_SECONDS, "the medium descent")

        self.time_touchdown_conditions_began = None

        def has_touched_down():
            height = height_above_floor()
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

    def wait_for_motors_off(self, time_of_touchdown, use_push_down=True):
        speed_after_touchdown = PUSH_DOWN_METRES_PER_SECOND if use_push_down else GENTLE_DESCENT_METRES_PER_SECOND
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
                 f"after touchdown ({'with' if use_push_down else 'without'} the push-down step).")
