"""Execute the corrected full course, measuring window alignment before crossing.

Uses stereo points and matched camera tracking through ForwardDepth, never simulator
truth. Alignment corrects the course/start-frame translation while holding outside the
wall. Missing, inconsistent, or stale observations stop the maneuver through existing
flight checks. The over-wall route is not supported.
"""
import json
import math
import numpy as np
from team_nav import flight_steps
from team_nav.practice_flights.astar_five_waypoints import AstarFlight
from team_nav.practice_flights.five_waypoint_plan import angle_difference
from team_nav.practice_flights.full_course import LANDING_SPEED_METRES_PER_SECOND
from team_nav.practice_flights.five_waypoints import MAXIMUM_HEIGHT_CORRECTION_METRES
from team_nav.practice_flights.window_inspection import (WindowInspection, ALIGNMENT_TIMEOUT_SECONDS,
    ALIGNMENT_HEADING_RADIANS, INSPECTION_BIAS_RADIANS, WINDOW_TURN_RATE_RADIANS_PER_SECOND)
from team_nav.window_alignment import observe_window

# Our choices: slow crossing and short lead reduce tracking lag at the narrow opening.
CROSSING_SPEED_METRES_PER_SECOND = .10
CROSSING_LEAD_METRES = .08
CROSS_TRACK_LIMIT_METRES = .025
# Our choices: gentle vertical motion, and a one-second low-pass filter on the
# measured difference between the two height estimates. Unlike accumulated target
# error, this does not treat climb lag as a sensor offset or retain an old offset.
VERTICAL_SPEED_METRES_PER_SECOND = .05
HEIGHT_OFFSET_FILTER_SECONDS = 1.


class WindowFlight(AstarFlight):
    def correct_height(self,wanted,actual,seconds):
        estimated = self.link.px4_height_risen_metres()
        if actual is None or estimated is None:
            return  # The existing flight watch handles missing or stale estimates.
        difference = estimated-actual[2]
        difference = float(np.clip(difference,-MAXIMUM_HEIGHT_CORRECTION_METRES,
                                   MAXIMUM_HEIGHT_CORRECTION_METRES))
        fraction = 1.-math.exp(-seconds/HEIGHT_OFFSET_FILTER_SECONDS)
        self.height_keeper.correction_metres += fraction*(difference-self.height_keeper.correction_metres)
        self.link.height_correction_metres = self.height_keeper.correction_metres

    def fly(self):
        self.log.save_text_file('mission.json', json.dumps(self.plan, indent=2))
        flight_steps.wait_until_ready_on_the_ground(self.link)
        flight_steps.hold_above_launch_until_control_is_ours(self.link, .75)
        self.flight_start_seconds = self.link.now_seconds()
        self.link.stage = 'climb'
        flight_steps.climb_above_launch(self.link, self.watch, .75)
        start = np.array(self.plan['start_metres'])
        for stage in self.plan['stages']:
            for index,point in enumerate(stage['points']):
                end = np.array(point)
                crossing = index==stage.get('align_at_point',-2)+1
                self.fly_segment(stage['name'],start,end,crossing)
                start = end
        self.link.stage = 'landing on pad'
        source = flight_steps.choose_landing_height_source(self.link,self.watch.px4_height_is_in_doubt())
        flight_steps.land(self.link,source,self.watch,descent_speed_limit=LANDING_SPEED_METRES_PER_SECOND)
        self.log.event('result','SENSOR-ALIGNED WINDOW ROUTE FINISHED. Check the independent referee.')

    def fly_segment(self,name,start,end,crossing):
        vertical = np.linalg.norm(end[:2]-start[:2])<1e-6
        heading = (self.plan['window_heading_radians'] if vertical
                   else math.atan2(end[1]-start[1],end[0]-start[0]))
        self.link.stage = name+': turn'
        self.turn_and_settle(start,heading)
        self.link.stage = name+': advance'
        self.log.event('stage',f'{self.link.stage}, target {end.tolist()}, height {end[2]:.3f} m',stage=self.link.stage)
        if crossing:
            self.cross_window(start,end,heading)
        elif vertical:
            self.advance(start,end,heading,False,VERTICAL_SPEED_METRES_PER_SECOND,.10)
        else:
            self.advance(start,end,heading,False)

    def cross_window(self,start,end,heading):
        self.align_or_retreat(start,heading)
        self.turn_and_settle(start,heading,WINDOW_TURN_RATE_RADIANS_PER_SECOND)
        self.link.stage = 'Chicane exit: sensor-aligned window crossing'
        normal = (end[:2]-start[:2])/np.linalg.norm(end[:2]-start[:2])
        along = np.array([-normal[1],normal[0]])
        def hold_if_off_line(actual,measured):
            if abs((actual[:2]-start[:2])@along)>CROSS_TRACK_LIMIT_METRES:
                return 'window cross-track error exceeds 2.5 cm'
            if abs(angle_difference(heading,measured))>ALIGNMENT_HEADING_RADIANS:
                return 'window heading error exceeds 2 degrees'
            return None
        self.advance(start,end,heading,False,CROSSING_SPEED_METRES_PER_SECOND,
                     CROSSING_LEAD_METRES,hold_if_off_line)

    def align_or_retreat(self,target,heading):
        try:
            self.align_window(target,heading)
        except flight_steps.RunMustStop as problem:
            # Only an observation timeout permits this mapped retreat. Critical
            # stale-telemetry, authority, contact, and height faults keep their
            # existing immediate response; they never enter this maneuver.
            timeout = f'Waited {ALIGNMENT_TIMEOUT_SECONDS:.0f} seconds for five consistent stereo'
            if not str(problem).startswith(timeout):
                raise
            retreat = np.array(self.plan['alignment_retreat_metres'])
            wanted = math.atan2(retreat[1]-target[1],retreat[0]-target[0])
            self.link.stage = 'window timeout: retreat to clear landing area'
            self.log.event('window_retreat','Alignment timed out; moving to the checked clear-floor landing point.')
            self.turn_and_settle(target,wanted)
            self.advance(target,retreat,wanted,False)
            raise  # The existing failure handler lands and records the failed flight.

    def align_window(self,target,heading):
        self.link.stage = 'window alignment'
        inspection = WindowInspection(self,target,heading,observe_window)
        flight_steps.fly_until(self.link,lambda:inspection.complete,inspection.send,ALIGNMENT_TIMEOUT_SECONDS,
                               'five consistent stereo window observations and measured alignment',self.watch)
        self.log.event('window_aligned','Window alignment confirmed from five fresh stereo observations.',
                       correction_metres=self.start_frame.drift_correction_metres.tolist())
