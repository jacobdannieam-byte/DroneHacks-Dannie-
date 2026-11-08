"""Fly five main goals through A*-generated intermediate points, all with explicit heights.

Uses the participant map at startup and the existing measured-arrival/depth controller
in flight. It does not read simulator truth or claim online obstacle replanning.
"""
import json
import math
import numpy as np
from team_nav.course_map import load_course_map
from team_nav.course_run import EXIT_REFUSED_TO_START, find_package_file
from team_nav.practice_flights import first_maze_leg
from team_nav.practice_flights.five_waypoints import FiveWaypointFlight, TURN_RATE_RADIANS_PER_SECOND
from team_nav.practice_flights.astar_geometry import RouteGeometry
from team_nav.practice_flights.astar_route import plan_mission
from team_nav.start_frame import StartFrame

FLIGHT_NAME = 'astar_five_waypoints'
# Our own choice (2026-11-08): the organizers' model without the maze, plus the sketched start area.
DEFAULT_MAP_FILE = 'real_course_2026-11-08.json'
# Our choice: the first checkpoint has little turn room, so slew the heading more gently.
TIGHT_TURN_RATE_RADIANS_PER_SECOND = math.radians(5)


def run(words, run_mode, log):
    arguments = first_maze_leg.make_argument_parser(FLIGHT_NAME, default_map=DEFAULT_MAP_FILE).parse_args(words)
    try:
        course_map = load_course_map(find_package_file(arguments.map,'course_maps'))
        with open(find_package_file('astar_five_goals.json','missions'),encoding='utf-8') as source:
            mission = json.load(source)
        if first_maze_leg.problems_with_the_settings(arguments,run_mode):
            raise ValueError('This full-course mission does not support stop-after-stage.')
        if not math.isclose(arguments.start_heading_degrees,90):
            raise ValueError('The first checkpoint approach requires a north-facing launch.')
        log.event('planning','Computing A* intermediate waypoints from the participant map before starting ROS control.')
        plan = plan_mission(course_map,mission)
        log.save_text_file('astar_plan.json',json.dumps(plan,indent=2))
        rows = ['number,stage,east_metres,north_metres,height_metres']
        number = 0
        for stage in plan['stages']:
            for point in stage['points']:
                number += 1
                rows.append(f"{number},{stage['name']},"+','.join(f'{value:.4f}' for value in point))
        rows.append(f"{number+1},Touchdown,{plan['stages'][-1]['points'][-1][0]:.4f},"
                    f"{plan['stages'][-1]['points'][-1][1]:.4f},0.0000")
        log.save_text_file('waypoints_with_heights.csv','\n'.join(rows)+'\n')
    except (OSError,ValueError,KeyError) as problem:
        log.event('refused',f'A* mission refused before arming: {problem}')
        return EXIT_REFUSED_TO_START
    frame = StartFrame(course_map.launch_mark_centre_metres,arguments.start_heading_degrees)
    def make_flight(*args,**kwargs):
        return AstarFlight(*args,plan=plan,**kwargs)
    return first_maze_leg.fly_with_ros(course_map,frame,arguments,log,flight_class=make_flight)


class AstarFlight(FiveWaypointFlight):
    def turn_and_settle(self, position, heading, turn_rate=TURN_RATE_RADIANS_PER_SECOND):
        geometry = RouteGeometry(self.course_map)
        if geometry.centre_clearance(position[:2])[0]<.35:
            turn_rate = TIGHT_TURN_RATE_RADIANS_PER_SECOND
        super().turn_and_settle(position,heading,turn_rate)
