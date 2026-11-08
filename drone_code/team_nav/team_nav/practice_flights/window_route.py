"""Plan the corrected course with a measured-alignment stop before the window.

Participant-map planning only. A* connects the five goals; the aperture crossing is a
straight, level maneuver after an in-place climb. Runtime sensor alignment must succeed
before that crossing is allowed. This module neither reads sensors nor sends targets.
"""
import math
import numpy as np
from team_nav.course_map import BlockingStrip
from team_nav.practice_flights.astar_geometry import RouteGeometry, BODY_MARGIN_METRES
from team_nav.practice_flights.astar_route import search_route

# Our choices: hold outside the frame, with room for the laser beside the nearby boulder.
STANDOFF_METRES = .78
CRUISE_HEIGHT_METRES = .75
# Our choice: clear floor east of the window for landing after an alignment timeout.
ALIGNMENT_RETREAT_METRES = np.array([.35,-1.65])


def window_geometry(course_map, height):
    """Use rectangular jambs, not rounded line caps that artificially shrink the aperture.

Only the window crossing uses this geometry. The checked straight crossing has its
centre inside the gap, so containment inside a solid jamb is separately rejected.
"""
    geometry = RouteGeometry(course_map, height)
    shapes = []
    wall = next(wall for wall in course_map.walls if wall.openings)
    for shape in geometry.shapes:
        if not isinstance(shape, BlockingStrip) or shape.name != wall.name:
            shapes.append(shape)
            continue
        direction = wall.direction()
        offset = np.array([-direction[1], direction[0]]) * shape.half_thickness_metres
        corners = [shape.start_metres+offset,shape.end_metres+offset,
                   shape.end_metres-offset,shape.start_metres-offset]
        for start, end in zip(corners, corners[1:]+corners[:1]):
            shapes.append(BlockingStrip(shape.name,start,end,0.))
    geometry.shapes = shapes
    return geometry


def full_window_plan(course_map, first_plan):
    plan = {'start_metres':first_plan['start_metres'], 'stages':list(first_plan['stages'])}
    geometry = RouteGeometry(course_map)
    start = np.array(plan['stages'][-1]['points'][-1][:2])
    heading = math.atan2(start[1]-plan['stages'][0]['points'][-1][1],start[0]-plan['stages'][0]['points'][-1][0])

    def connect(goal):
        nonlocal start, heading
        goal = np.asarray(goal)
        wanted = math.atan2(goal[1]-start[1],goal[0]-start[0])
        if geometry.turn_clear(start,heading,wanted) and geometry.segment_clear(start,goal,wanted):
            path = [start,goal]
        else:
            path,_ = search_route(geometry,start,goal,heading)
        heading = math.atan2(path[-1][1]-path[-2][1],path[-1][0]-path[-2][0])
        start = goal
        return [np.append(point,CRUISE_HEIGHT_METRES).tolist() for point in path[1:]]

    # Preserve the tunnel goal; it lies beyond the full corrected tunnel exit.
    tunnel = [2.06,-.65]
    points = connect(tunnel)
    if not geometry.crossing_clear(start,heading,course_map.checkpoint_line_named('tunnel exit')):
        raise ValueError('The full airframe does not clear the tunnel exit.')
    plan['stages'].append({'name':'Tunnel exit','checkpoint':'tunnel exit','points':points})
    wall = next(wall for wall in course_map.walls if wall.openings)
    opening = wall.openings[0]
    centre = wall.point_along_wall_metres((opening.start_along_wall_metres+opening.end_along_wall_metres)/2)
    direction = wall.direction()
    normal = np.array([-direction[1],direction[0]])
    before,after = centre-STANDOFF_METRES*normal,centre+STANDOFF_METRES*normal
    height = (opening.bottom_height_metres+opening.top_height_metres)/2
    crossing_heading = math.atan2(normal[1],normal[0])
    points = connect(before)
    if not geometry.turn_clear(before,heading,crossing_heading):
        raise ValueError('There is no room to face the window at the approach point.')
    for altitude in np.linspace(CRUISE_HEIGHT_METRES,height,20):
        raised = RouteGeometry(course_map,altitude)
        if min(raised.footprint_clearance([before,after],crossing_heading)) < BODY_MARGIN_METRES:
            raise ValueError('The climb or descent beside the window is obstructed.')
    aperture = window_geometry(course_map,height)
    samples = np.linspace(before,after,200)
    if min(aperture.footprint_clearance(samples,crossing_heading)) < BODY_MARGIN_METRES:
        raise ValueError('The nominal window crossing lacks the existing 5 cm body margin.')
    points.extend([np.append(before,height).tolist(),np.append(after,height).tolist(),np.append(after,.75).tolist()])
    alignment_index = len(points)-3
    start,heading = after,crossing_heading
    points.extend(connect([-1.2,-1.35]))
    plan['stages'].append({'name':'Chicane exit','checkpoint':'through the window','points':points,
                           'align_at_point':alignment_index})
    plan['stages'].append({'name':'Landing pad','checkpoint':None,'points':connect(course_map.landing_pad_centre_metres)})
    retreat_heading = math.atan2(*(ALIGNMENT_RETREAT_METRES-before)[::-1])
    raised = RouteGeometry(course_map,height)
    if (not raised.turn_clear(before,crossing_heading,retreat_heading)
            or not raised.segment_clear(before,ALIGNMENT_RETREAT_METRES,retreat_heading)):
        raise ValueError('The alignment-timeout retreat is obstructed. Recheck the window approach.')
    for altitude in np.linspace(.20,height,20):
        landing = RouteGeometry(course_map,altitude)
        if landing.footprint_clearance(ALIGNMENT_RETREAT_METRES,retreat_heading)[0] < BODY_MARGIN_METRES:
            raise ValueError('The alignment-timeout landing point is obstructed.')
    plan['alignment_retreat_metres'] = np.append(ALIGNMENT_RETREAT_METRES,height).tolist()
    plan['window_heading_radians'] = crossing_heading
    return plan
