"""Generate intermediate waypoints with heading-aware A* between five mission goals.

Real flight planning code. The participant map is the input; no world truth is read.
Each edge turns at rest then travels forward. No reverse motion is generated.
"""
import heapq
import math
import numpy as np
from team_nav.practice_flights.astar_geometry import RouteGeometry, ChicaneErrorMargin
from team_nav.practice_flights.five_waypoint_plan import angle_difference

# Our choices: 10 cm grid, extra shallow directions for the first tight forward exit,
# clearance preference, and a finite search budget so bad maps cannot hang startup.
GRID_METRES = .10
COMFORTABLE_ROOM_METRES = .60
CLEARANCE_WEIGHT = 8.
TURN_COST_METRES_PER_RADIAN = .15
SEARCH_LIMIT = 100000
GOAL_CONNECTION_METRES = .22
DIRECTIONS = sorted({(x,y) for x,y in [(1,0),(1,1),(2,1),(4,1)]
                     for x,y in [(x,y),(y,x)] for x in {x,-x} for y in {y,-y}})
HEADINGS = [math.atan2(y,x) for x,y in DIRECTIONS]


def search_route(geometry, start, goal, initial_heading, required_heading=None):
    start,goal = np.asarray(start,dtype=float),np.asarray(goal,dtype=float)
    # Anchor the grid at the measured start, avoiding a sideways jump to a grid centre.
    first = (0,0,-1)
    queue = [(float(np.linalg.norm(goal-start)),0.,first)]
    best = {first:0.}
    previous = {first:None}
    edge_cache = {}
    turn_cache = {}
    expanded = 0
    while queue and expanded<SEARCH_LIMIT:
        _,cost,state = heapq.heappop(queue)
        if cost>best[state]:
            continue
        expanded += 1
        x,y,direction = state
        point = start+GRID_METRES*np.array([x,y])
        heading = initial_heading if direction==-1 else (0. if direction==-2 else HEADINGS[direction])
        if np.linalg.norm(point-goal)<GOAL_CONNECTION_METRES:
            final_heading = math.atan2(goal[1]-point[1],goal[0]-point[0]) if np.linalg.norm(point-goal)>1e-6 else heading
            if (geometry.turn_clear(point,heading,final_heading) and geometry.segment_clear(point,goal,final_heading)
                    and (required_heading is None or abs(angle_difference(final_heading,required_heading))<1e-6)):
                path = [goal]
                while state is not None:
                    path.append(start+GRID_METRES*np.array(state[:2]))
                    state = previous[state]
                path = list(reversed(path))
                return (simplify(geometry,path,initial_heading) if required_heading is None else path),expanded
        for number,(dx,dy) in enumerate(DIRECTIONS):
            target = point+GRID_METRES*np.array([dx,dy])
            low,high = geometry.course_map.lowest_corner_metres,geometry.course_map.highest_corner_metres
            if np.any(target<low) or np.any(target>high):
                continue
            edge_key = (x,y,number)
            if edge_key not in edge_cache:
                edge_cache[edge_key] = geometry.segment_clear(point,target,HEADINGS[number])
            if not edge_cache[edge_key]:
                continue
            turn_key = (x,y,direction,number)
            if turn_key not in turn_cache:
                turn_cache[turn_key] = direction==-2 or geometry.turn_clear(point,heading,HEADINGS[number])
            if not turn_cache[turn_key]:
                continue
            room = min(geometry.centre_clearance(np.array([point,target])))
            penalty = CLEARANCE_WEIGHT*max(0.,COMFORTABLE_ROOM_METRES-room)**2
            length = GRID_METRES*math.hypot(dx,dy)
            new_cost = cost+length*(1+penalty)
            # At a centre with full circular clearance all headings are equivalent for
            # feasibility. Merge those states; retain heading in constrained pockets.
            next_heading = -2 if geometry.centre_clearance(target)[0]>=.35 else number
            neighbour = (x+dx,y+dy,next_heading)
            if new_cost<best.get(neighbour,math.inf):
                best[neighbour] = new_cost
                previous[neighbour] = state
                heapq.heappush(queue,(new_cost+np.linalg.norm(target-goal),new_cost,neighbour))
    raise ValueError(f'A* found no forward route from {start.tolist()} to {goal.tolist()} after {expanded} states. Check map clearance.')


def simplify(geometry, points, initial_heading):
    result = [points[0]]
    index,heading = 0,initial_heading
    while index<len(points)-1:
        chosen = index+1
        for candidate in range(len(points)-1,index,-1):
            start,end = points[index],points[candidate]
            if np.linalg.norm(end-start)<1e-6:
                continue
            wanted = math.atan2(end[1]-start[1],end[0]-start[0])
            if not geometry.turn_clear(start,heading,wanted) or not geometry.segment_clear(start,end,wanted):
                continue
            if candidate<len(points)-1:
                after = points[candidate+1]-end
                if not geometry.turn_clear(end,wanted,math.atan2(after[1],after[0])):
                    continue
            # A shortcut must retain the endpoint clearance, up to our preferred 45 cm.
            room = geometry.centre_clearance(np.linspace(start,end,40))
            required = min(.45,float(min(room[0],room[-1]))-.01)
            if room.min()<required:
                continue
            chosen = candidate
            break
        delta = points[chosen]-result[-1]
        if np.linalg.norm(delta)>1e-6:
            heading = math.atan2(delta[1],delta[0])
            result.append(points[chosen])
        index = chosen
    return result


def plan_mission(course_map, mission):
    if len(mission['goals'])!=5:
        raise ValueError('This mission requires five main goals.')
    if any(not np.isclose(stage['point_metres'][2],.75) for stage in mission['goals']):
        raise ValueError('This planner checks the course at 0.75 m. Keep every flight goal at that height.')
    geometry = RouteGeometry(course_map)
    start = np.array(mission['start_metres'][:2],dtype=float)
    heading = math.pi/2
    plan = {'start_metres':mission['start_metres'],'stages':[]}
    for number,stage in enumerate(mission['goals']):
        goal = np.array(stage['point_metres'][:2],dtype=float)
        if number==0:
            # A clear direct connection needs no intermediate points. If obstructed, use
            # A* with a north-facing final approach so the entire airframe clears the line.
            wanted = math.atan2(goal[1]-start[1],goal[0]-start[0])
            if geometry.turn_clear(start,heading,wanted) and geometry.segment_clear(start,goal,wanted):
                path,expanded = [start,goal],0
            else:
                path,expanded = search_route(geometry,start,goal,heading,required_heading=math.pi/2)
        else:
            search_geometry = ChicaneErrorMargin(geometry) if stage['name']=='Chicane exit' else geometry
            path,expanded = search_route(search_geometry,start,goal,heading)
        heading = math.atan2(path[-1][1]-path[-2][1],path[-1][0]-path[-2][0])
        if stage['checkpoint'] and not geometry.crossing_clear(goal,heading,course_map.checkpoint_line_named(stage['checkpoint'])):
            raise ValueError(f"The full airframe does not clear {stage['name']}. Check the goal region.")
        plan['stages'].append({'name':stage['name'],'checkpoint':stage['checkpoint'],
            'points':[[float(p[0]),float(p[1]),stage['point_metres'][2]] for p in path[1:]],'expanded_states':expanded})
        start = goal
    if np.linalg.norm(goal-course_map.landing_pad_centre_metres)+.35>course_map.landing_pad_outer_radius_metres:
        raise ValueError('The final goal does not leave room for the full airframe on the landing pad.')
    return plan
