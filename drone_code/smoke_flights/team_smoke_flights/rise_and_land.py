"""The first real flight: rise straight up, hold still, come back down and land. FOR THE REAL DRONE.

This is the smallest flight that tests the whole chain: the hand-over to our program, the climb,
holding a height, the descent, the touchdown and PX4 switching the motors off. It goes nowhere
sideways. The same file also runs unchanged in the 3D rendered sim, through
simulator/test_flights/rise_and_land_test.py.

How it starts
-------------
This program never arms the drone and never changes its flight mode. A teammate does both on
the remote control. They arm the drone, then flip the mode switch to Position. The organizers'
flight interface then hands control to our program; this hand-over is called entering Offboard
mode. Until that moment, our targets have no effect, and the drone stays on the ground.

The teammate holding the kill switch can stop the motors at any time, and flipping the mode switch
takes control back from our program. If that happens, this program stops sending targets and
does nothing more.

What it does
------------
1. Waits on the ground. It keeps sending a target above where it sits, because the organizers'
   interface only hands over control to a program that is already sending targets.
2. Climbs to rise_height_metres above where it started.
3. Holds that height for hold_seconds.
4. Descends: medium speed at first, then gentle near the floor (section 7 of docs/ARCHITECTURE.md).
5. Once the bottom laser shows it has touched the floor, it commands a faster descent. The floor
   stops the drone, and the faster command makes PX4 lower the motor power sooner, so PX4 decides
   it has landed and switches the motors off.

Every tenth of a second it writes one line to a recording file, so that every flight can be
studied afterwards. See telemetry_recorder.py.

What it does not do
-------------------
It doesn't plan a route or read a map. It doesn't move sideways on purpose.
"""
import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from team_smoke_flights.telemetry_recorder import TelemetryRecorder

# Our own choice. The organizers' interface ignores a target older than 1 second, so 10 targets
# per second leaves plenty of margin. Every stage below runs once per tick.
TICKS_PER_SECOND = 10.0

# Our own choices for the first flight: low and short, as the test order in section 11 of
# docs/ARCHITECTURE.md asks. The organizers' interface refuses any height target below 0.3 m
# above the starting point, so the height must be above that.
DEFAULT_RISE_HEIGHT_METRES = 0.5
DEFAULT_HOLD_SECONDS = 5.0
DEFAULT_LOG_FOLDER = "~/team_ws/flight_logs"

# Our own choice: close enough to the target height to start the hold.
HEIGHT_TOLERANCE_METRES = 0.10

# The descent plan, from section 7 of docs/ARCHITECTURE.md, tested in the 3D rendered sim.
MEDIUM_DESCENT_METRES_PER_SECOND = 0.5
GENTLE_DESCENT_METRES_PER_SECOND = 0.35
PUSH_DOWN_METRES_PER_SECOND = 1.0
HEIGHT_TO_START_GENTLE_DESCENT_METRES = 0.30

# Touchdown means: the laser reads within this distance of its reading on the floor, and the
# drone is not moving down, for this long. From section 7 of docs/ARCHITECTURE.md.
TOUCHDOWN_HEIGHT_TOLERANCE_METRES = 0.03
TOUCHDOWN_LARGEST_DOWNWARD_SPEED_METRES_PER_SECOND = 0.05
TOUCHDOWN_SECONDS_REQUIRED = 0.3

# Our own choices. If a stage takes longer than this, something is wrong, and the program moves
# on to landing. It never simply stops sending targets while the drone is in the air.
LONGEST_CLIMB_SECONDS = 20.0
LONGEST_DESCENT_SECONDS = 20.0

# Our own choice. If camera tracking sends nothing for this long, the drone no longer knows
# where it is, so the program lands straight away.
LONGEST_SILENCE_FROM_CAMERA_TRACKING_SECONDS = 0.5

# PX4's messages need these settings to be received (from PX4's ROS 2 guide).
PX4_MESSAGE_SETTINGS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class Stage:
    WAITING_ON_THE_GROUND = "waiting on the ground"
    CLIMBING = "climbing"
    HOLDING = "holding"
    DESCENDING = "descending"
    PUSHING_DOWN = "pushing down after touchdown"
    FINISHED = "finished: motors are off"
    PILOT_TOOK_OVER = "stopped: the pilot took control"


class RiseAndLand(Node):

    def __init__(self, **node_options):
        super().__init__("team_rise_and_land", **node_options)
        self.rise_height_metres = self.declare_parameter("rise_height_metres", DEFAULT_RISE_HEIGHT_METRES).value
        self.hold_seconds = self.declare_parameter("hold_seconds", DEFAULT_HOLD_SECONDS).value
        log_folder = self.declare_parameter("log_folder", DEFAULT_LOG_FOLDER).value

        # What the drone reports. None until the first message arrives.
        self.drone_state = None              # the organizers' summary: armed, landed, Offboard...
        self.tracked_position = None         # camera tracking: (forward, left, up) in metres
        self.time_of_last_tracking_seconds = None
        self.px4_position = None             # PX4's own estimate, including the laser distance

        # Remembered on the ground, just before the hand-over.
        self.start_position = None
        self.floor_laser_reading_metres = None
        self.start_px4_height_up_metres = None   # PX4's own height on the floor, with up positive.

        self.stage = Stage.WAITING_ON_THE_GROUND
        self.time_stage_began_seconds = time.monotonic()
        self.time_touchdown_conditions_began_seconds = None
        self.last_target_description = "none"
        self.said_this_stage = set()

        self.pose_publisher = self.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
        self.twist_publisher = self.create_publisher(TwistStamped, "/skybeta/setpoint/twist", 10)
        self.create_subscription(String, "/skybeta/state", self.on_drone_state, 10)
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self.on_tracked_position, 10)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                                 self.on_px4_position, PX4_MESSAGE_SETTINGS)

        self.recorder = TelemetryRecorder(log_folder, "rise_and_land", self.say)
        self.create_timer(1.0 / TICKS_PER_SECOND, self.tick)
        self.say(f"Ready. Plan: rise {self.rise_height_metres:.2f} m, hold {self.hold_seconds:.0f} s, land. "
                 "Waiting for a teammate to arm the drone and flip the switch to Position.")

    # ---- What the drone reports ----------------------------------------------------------------

    def on_drone_state(self, message):
        try:
            self.drone_state = json.loads(message.data)
        except ValueError:
            pass

    def on_tracked_position(self, message):
        position = message.pose.pose.position
        self.tracked_position = (float(position.x), float(position.y), float(position.z))
        self.time_of_last_tracking_seconds = time.monotonic()

    def on_px4_position(self, message):
        self.px4_position = message

    def state_says(self, field_name):
        return self.drone_state is not None and bool(self.drone_state.get(field_name))

    def laser_reading_metres(self):
        """PX4's distance to the floor below, which comes from the bottom laser. None if PX4's
        height estimate isn't working. (We check z_valid rather than dist_bottom_valid: with
        drone 16's settings the second one stays false; see team_nav/height_check.py.)"""
        if self.px4_position is None or not self.px4_position.z_valid:
            return None
        return float(self.px4_position.dist_bottom)

    def height_above_floor_metres(self):
        """How high the drone is above where it started, from the laser. Falls back to camera
        tracking if the laser's reading isn't available."""
        laser = self.laser_reading_metres()
        if laser is not None and self.floor_laser_reading_metres is not None:
            return laser - self.floor_laser_reading_metres
        if self.tracked_position is not None and self.start_position is not None:
            return self.tracked_position[2] - self.start_position[2]
        return None

    def downward_speed_metres_per_second(self):
        if self.px4_position is None:
            return 0.0
        return float(self.px4_position.vz)  # PX4's z axis points down, so positive is downward.

    def camera_tracking_has_gone_quiet(self):
        if self.time_of_last_tracking_seconds is None:
            return True
        return time.monotonic() - self.time_of_last_tracking_seconds > LONGEST_SILENCE_FROM_CAMERA_TRACKING_SECONDS

    def is_ready_on_the_ground(self):
        return (self.state_says("px4_link") and self.state_says("landed")
                and self.tracked_position is not None and self.laser_reading_metres() is not None)

    # ---- Sending targets -----------------------------------------------------------------------

    def send_target_above_start(self, height_metres):
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "odom"
        target.pose.position.x = self.start_position[0]
        target.pose.position.y = self.start_position[1]
        # The organizers' interface gives our "up" to PX4 as its height with no shift, but PX4's
        # height and camera tracking's height have different zero points. Measured on drone 16
        # on 2026-11-08: a 0.80 m target stopped 0.65 m above the floor, because PX4 read
        # 0.15 m on the floor. So we give the height as PX4 reads it.
        target.pose.position.z = self.start_px4_height_up_metres + height_metres
        # An orientation of all zeros tells the organizers' interface to keep the current heading.
        target.pose.orientation.w = 0.0
        self.pose_publisher.publish(target)
        self.last_target_description = f"hold {height_metres:.2f} m above start"

    def send_downward_speed(self, metres_per_second):
        target = TwistStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "odom"
        target.twist.linear.z = -float(metres_per_second)  # These directions have up as positive.
        self.twist_publisher.publish(target)
        self.last_target_description = f"descend at {metres_per_second:.2f} m/s"

    # ---- The stages ----------------------------------------------------------------------------

    def tick(self):
        self.recorder.write(self.telemetry_row())
        if self.stage in (Stage.FINISHED, Stage.PILOT_TOOK_OVER):
            return
        if self.stage != Stage.WAITING_ON_THE_GROUND and self.pilot_has_taken_control():
            self.move_to(Stage.PILOT_TOOK_OVER)
            return
        if self.stage in (Stage.CLIMBING, Stage.HOLDING) and self.camera_tracking_has_gone_quiet():
            self.say("Camera tracking has gone quiet, so the drone doesn't know where it is. Landing now.")
            self.move_to(Stage.DESCENDING)

        if self.stage == Stage.WAITING_ON_THE_GROUND:
            self.wait_on_the_ground()
        elif self.stage == Stage.CLIMBING:
            self.climb()
        elif self.stage == Stage.HOLDING:
            self.hold()
        elif self.stage == Stage.DESCENDING:
            self.descend()
        elif self.stage == Stage.PUSHING_DOWN:
            self.push_down_until_motors_stop()

    def pilot_has_taken_control(self):
        # Once the motors are off, Offboard ends too; that is a normal finish, not a take-over.
        return self.state_says("armed") and not self.state_says("offboard_active")

    def wait_on_the_ground(self):
        if not self.is_ready_on_the_ground():
            return
        # Keep the floor reference fresh until the moment of the hand-over.
        self.start_position = self.tracked_position
        self.floor_laser_reading_metres = self.laser_reading_metres()
        self.start_px4_height_up_metres = -float(self.px4_position.z)   # PX4's z points down.
        self.send_target_above_start(self.rise_height_metres)
        if self.state_says("offboard_active"):
            self.say(f"Control handed to us. Floor laser reading {self.floor_laser_reading_metres:.3f} m.")
            self.move_to(Stage.CLIMBING)

    def climb(self):
        self.send_target_above_start(self.rise_height_metres)
        height = self.height_above_floor_metres()
        if height is not None and abs(height - self.rise_height_metres) < HEIGHT_TOLERANCE_METRES:
            self.say(f"Reached {height:.2f} m. Holding for {self.hold_seconds:.0f} s.")
            self.move_to(Stage.HOLDING)
        elif self.seconds_in_this_stage() > LONGEST_CLIMB_SECONDS:
            self.say(f"The climb took more than {LONGEST_CLIMB_SECONDS:.0f} s without reaching the height. "
                     f"Last height: {height}. Landing instead.")
            self.move_to(Stage.DESCENDING)

    def hold(self):
        self.send_target_above_start(self.rise_height_metres)
        if self.seconds_in_this_stage() >= self.hold_seconds:
            self.say("Hold finished. Descending.")
            self.move_to(Stage.DESCENDING)

    def descend(self):
        height = self.height_above_floor_metres()
        if height is not None and height < HEIGHT_TO_START_GENTLE_DESCENT_METRES:
            self.send_downward_speed(GENTLE_DESCENT_METRES_PER_SECOND)
        else:
            self.send_downward_speed(MEDIUM_DESCENT_METRES_PER_SECOND)

        if self.has_touched_down(height):
            self.say(f"Touchdown, {self.seconds_in_this_stage():.1f} s after starting down. "
                     f"Pushing down so PX4 switches the motors off.")
            self.move_to(Stage.PUSHING_DOWN)
        elif self.seconds_in_this_stage() > LONGEST_DESCENT_SECONDS:
            # Keep descending gently: stopping the targets in the air would be worse.
            self.say_once_per_stage("The descent is taking longer than expected. Still descending gently.")

    def has_touched_down(self, height):
        conditions_are_met = (
            height is not None
            and height < TOUCHDOWN_HEIGHT_TOLERANCE_METRES
            and self.downward_speed_metres_per_second() < TOUCHDOWN_LARGEST_DOWNWARD_SPEED_METRES_PER_SECOND
        )
        if not conditions_are_met:
            self.time_touchdown_conditions_began_seconds = None
            return False
        if self.time_touchdown_conditions_began_seconds is None:
            self.time_touchdown_conditions_began_seconds = time.monotonic()
        return time.monotonic() - self.time_touchdown_conditions_began_seconds >= TOUCHDOWN_SECONDS_REQUIRED

    def push_down_until_motors_stop(self):
        if self.drone_state is not None and not self.state_says("armed"):
            self.say(f"PX4 switched the motors off, {self.seconds_in_this_stage():.1f} s after touchdown. "
                     "RESULT: the rise-and-land flight finished.")
            self.move_to(Stage.FINISHED)
            return
        self.send_downward_speed(PUSH_DOWN_METRES_PER_SECOND)

    # ---- Bookkeeping ---------------------------------------------------------------------------

    def move_to(self, stage):
        self.stage = stage
        self.time_stage_began_seconds = time.monotonic()
        self.said_this_stage = set()
        self.say(f"Stage: {stage}.")

    def seconds_in_this_stage(self):
        return time.monotonic() - self.time_stage_began_seconds

    def say(self, text):
        self.get_logger().info(text)

    def say_once_per_stage(self, text):
        if text not in self.said_this_stage:
            self.said_this_stage.add(text)
            self.say(text)

    def telemetry_row(self):
        """One line of the recording: everything this program knows at this moment."""
        tracked = self.tracked_position or (None, None, None)
        return {
            "stage": self.stage,
            "armed": self.state_says("armed"),
            "offboard_active": self.state_says("offboard_active"),
            "px4_says_landed": self.state_says("landed"),
            "px4_link": self.state_says("px4_link"),
            "tracked_forward_metres": tracked[0],
            "tracked_left_metres": tracked[1],
            "tracked_up_metres": tracked[2],
            "laser_reading_metres": self.laser_reading_metres(),
            "height_above_floor_metres": self.height_above_floor_metres(),
            "downward_speed_metres_per_second": self.downward_speed_metres_per_second(),
            "target": self.last_target_description,
        }


def main():
    rclpy.init()
    flight = RiseAndLand()
    try:
        rclpy.spin(flight)
    except KeyboardInterrupt:
        pass
    finally:
        flight.recorder.close()
        flight.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
