"""Hold outside a window until stereo jamb measurements confirm alignment.

Uses the flight's stamped depth and camera tracking. It searches camera headings,
filters the map-frame translation, and reports completion; it never reads simulator
truth or authorizes a crossing without fresh measurements.
"""
import math
from collections import deque
import numpy as np
from team_nav import flight_steps
from team_nav.practice_flights.five_waypoint_plan import angle_difference
from team_nav.practice_flights.five_waypoints import turn_towards

# Our choices: tight alignment gates for the mapped 60.38 cm aperture.
ALIGNMENT_TOLERANCE_METRES = .015
ALIGNMENT_HEIGHT_TOLERANCE_METRES = .025
ALIGNMENT_HEADING_RADIANS = math.radians(2)
MAXIMUM_CORRECTION_STEP_METRES = .01
MAXIMUM_TOTAL_CORRECTION_METRES = .40
ALIGNMENT_TIMEOUT_SECONDS = 60
CONSISTENT_PICTURES = 5
OBSERVATION_WINDOW_SECONDS = 3.
CORRECTION_GAIN = .25
# Our choices: begin slightly off-normal and scan if dense jamb returns disappear.
INSPECTION_BIAS_RADIANS = math.radians(8)
SCAN_OFFSETS_DEGREES = [8,16,24,32,0,-8,-16]
SCAN_DWELL_SECONDS = 4.
MAXIMUM_PICTURE_AGE_SECONDS = 1.
MAXIMUM_FUTURE_STAMP_SECONDS = .1
MAXIMUM_UPDATE_SECONDS = .20
# Our choice: gentle inspection and final yaw reduce the height transient seen
# when returning from a 10-degree inspection offset in Gazebo.
WINDOW_TURN_RATE_RADIANS_PER_SECOND = math.radians(5)


class WindowInspection:
    def __init__(self,flight,target,heading,observe_window):
        self.flight = flight
        self.target = target
        self.heading = heading
        self.observe_window = observe_window
        self.wall = next(wall for wall in flight.course_map.walls if wall.openings)
        self.recent = deque(maxlen=CONSISTENT_PICTURES)
        self.last_stamp = None
        self.complete = False
        self.initial = flight.start_frame.drift_correction_metres.copy()
        self.commanded = flight.watch.drone_heading_on_course()
        self.wanted = heading+INSPECTION_BIAS_RADIANS
        self.scan_index = 0
        self.last_observed = self.last_cycle = flight.link.now_seconds()

    def send(self):
        now = self.flight.link.now_seconds()
        seconds = min(now-self.last_cycle,MAXIMUM_UPDATE_SECONDS)
        self.last_cycle = now
        self.scan_if_needed(now)
        self.commanded = turn_towards(self.commanded,self.wanted,seconds,WINDOW_TURN_RATE_RADIANS_PER_SECOND)
        actual = self.flight.watch.drone_on_course()
        self.flight.correct_height(self.target[2],actual,seconds)
        if self.picture_is_fresh(now):
            self.consume_picture(now)
        self.flight.watch.send_target_on_course(self.target,heading_on_course_radians=self.commanded)

    def scan_if_needed(self,now):
        if now-self.last_observed <= SCAN_DWELL_SECONDS:
            return
        self.scan_index = (self.scan_index+1)%len(SCAN_OFFSETS_DEGREES)
        self.wanted = self.heading+math.radians(SCAN_OFFSETS_DEGREES[self.scan_index])
        self.last_observed = now
        self.recent.clear()
        self.flight.log.event('window_scan','Looking for both window jambs while holding position.',
                              heading_degrees=math.degrees(self.wanted))

    def picture_is_fresh(self,now):
        depth = self.flight.depth
        if depth.stamp is None or depth.stamp==self.last_stamp:
            return False
        if now-depth.arrival >= MAXIMUM_PICTURE_AGE_SECONDS or depth.picture_heading is None:
            return False
        exposure_age = self.flight.link.node.get_clock().now().nanoseconds*1e-9-depth.stamp
        facing = abs(angle_difference(self.commanded,depth.picture_heading))<ALIGNMENT_HEADING_RADIANS
        return -MAXIMUM_FUTURE_STAMP_SECONDS<=exposure_age<=MAXIMUM_PICTURE_AGE_SECONDS and facing

    def consume_picture(self,now):
        self.last_stamp = self.flight.depth.stamp
        while self.recent and now-self.recent[0][0]>OBSERVATION_WINDOW_SECONDS:
            self.recent.popleft()
        observation = self.observe_window(self.flight.depth.points,self.wall)
        if observation is None:
            self.flight.log.event('window_unobserved','Both window jambs are not yet reliably visible.')
            return
        # Hold the angle that showed both jambs. A predicted centre may move a
        # jamb into a poorly matched part of the stereo image.
        self.wanted = self.flight.depth.picture_heading
        self.last_observed = now
        self.apply_correction(observation)
        self.recent.append((now,observation.correction_metres))
        self.complete = self.alignment_is_confirmed()

    def apply_correction(self,observation):
        step = observation.correction_metres*CORRECTION_GAIN
        size = np.linalg.norm(step)
        if size>MAXIMUM_CORRECTION_STEP_METRES:
            step *= MAXIMUM_CORRECTION_STEP_METRES/size
        proposed = self.flight.start_frame.drift_correction_metres+step
        if np.linalg.norm(proposed-self.initial)>MAXIMUM_TOTAL_CORRECTION_METRES:
            raise flight_steps.RunMustStop('Window alignment needs more than 0.40 m correction. Check the map and tracking.')
        self.flight.start_frame.drift_correction_metres = proposed
        self.flight.log.event('window_observation','Measured window alignment.',
            correction_metres=observation.correction_metres.tolist(),total_correction_metres=proposed.tolist(),
            width_metres=observation.width_metres,plane_spread_metres=observation.plane_spread_metres,
            wall_points=observation.wall_points,stamp=self.flight.depth.stamp,
            measured_position_metres=self.flight.watch.drone_on_course().tolist(),
            measured_heading_degrees=math.degrees(self.flight.watch.drone_heading_on_course()))

    def alignment_is_confirmed(self):
        if len(self.recent)<CONSISTENT_PICTURES:
            return False
        actual = self.flight.watch.drone_on_course()
        measured = self.flight.watch.drone_heading_on_course()
        return (max(np.linalg.norm(correction) for _,correction in self.recent)<ALIGNMENT_TOLERANCE_METRES
                and np.linalg.norm(actual[:2]-self.target[:2])<ALIGNMENT_TOLERANCE_METRES
                and abs(actual[2]-self.target[2])<ALIGNMENT_HEIGHT_TOLERANCE_METRES
                and abs(angle_difference(self.wanted,measured))<ALIGNMENT_HEADING_RADIANS)
