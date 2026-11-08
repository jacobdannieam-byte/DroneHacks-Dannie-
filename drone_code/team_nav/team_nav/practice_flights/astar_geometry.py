"""Check a turning airframe against the participant map, without ROS or simulator data.

The convex motor quadrilateral expanded by the propeller radius contains the frame and
propellers. This avoids the empty corners of the old rectangular envelope in tight turns.
"""
import math
import numpy as np
from team_nav.course_map import BlockingStrip, shapes_blocking_between_heights, distance_to_shapes_metres
from team_nav.practice_flights.five_waypoint_plan import angle_difference

# Measured motor locations (forward, left), and the team's estimated 7-inch propellers.
MOTORS_METRES = np.array([[-.15,-.14],[.08,-.16],[.08,.16],[-.15,.14]])
PROPELLER_RADIUS_METRES = .0889
# Existing route margins; sampling choices for translation and yaw checks.
BODY_MARGIN_METRES = .05
LASER_MARGIN_METRES = .15
TRANSLATION_SAMPLE_METRES = .025
TURN_SAMPLE_RADIANS = math.radians(2)
OUTER_RADIUS_METRES = .30


def point_edge_distance(point, starts, ends):
    along = ends-starts
    fraction = np.clip(np.sum((point-starts)*along, axis=-1)/np.sum(along*along, axis=-1), 0, 1)
    return np.linalg.norm(point-starts-fraction[...,None]*along, axis=-1)


def cross(first, second):
    return first[...,0]*second[...,1]-first[...,1]*second[...,0]


def inside(point, polygon):
    signs = cross(np.roll(polygon,-1,axis=1)-polygon, point-polygon)
    return np.all(signs >= 0,axis=1) | np.all(signs <= 0,axis=1)


class RouteGeometry:
    def __init__(self, course_map, height_metres=.75):
        self.course_map = course_map
        self.shapes = shapes_blocking_between_heights(course_map, height_metres-.20, height_metres+.20)
        self._centre_cache = {}

    def centre_clearance(self, points):
        points = np.atleast_2d(points)
        keys = [tuple(np.round(point,8)) for point in points]
        missing = list(dict.fromkeys(key for key in keys if key not in self._centre_cache))
        if missing:
            distances = distance_to_shapes_metres(np.array(missing),self.shapes)[0]
            self._centre_cache.update(zip(missing,distances))
        return np.array([self._centre_cache[key] for key in keys])

    def footprint_clearance(self, centres, headings):
        centres = np.atleast_2d(centres)
        headings = np.broadcast_to(headings, (len(centres),))
        cosine, sine = np.cos(headings)[:,None], np.sin(headings)[:,None]
        forward, left = MOTORS_METRES.T
        polygon = centres[:,None,:]+np.stack((cosine*forward-sine*left, sine*forward+cosine*left),axis=-1)
        ends = np.roll(polygon,-1,axis=1)
        nearest = np.full(len(centres),np.inf)
        for shape in self.shapes:
            if isinstance(shape, BlockingStrip):
                a,b = shape.start_metres,shape.end_metres
                distances = point_edge_distance(polygon,a,b).min(axis=1)
                distances = np.minimum(distances,point_edge_distance(a,polygon,ends).min(axis=1))
                distances = np.minimum(distances,point_edge_distance(b,polygon,ends).min(axis=1))
                first = cross(b-a,polygon-a)*cross(b-a,ends-a)
                second = cross(ends-polygon,a-polygon)*cross(ends-polygon,b-polygon)
                overlap = np.all((np.maximum(polygon,ends)>=np.minimum(a,b))
                                 & (np.minimum(polygon,ends)<=np.maximum(a,b)),axis=-1)
                intersects = np.any((first <= 0)&(second <= 0)&overlap,axis=1)
                distances = np.where(intersects|inside(a,polygon)|inside(b,polygon),0,distances)
                distances -= shape.half_thickness_metres
            else:
                distances = point_edge_distance(shape.centre_metres,polygon,ends).min(axis=1)
                distances = np.where(inside(shape.centre_metres,polygon),0,distances)-shape.radius_metres
            nearest = np.minimum(nearest,distances-PROPELLER_RADIUS_METRES)
        return nearest

    def turn_clear(self, point, old_heading, new_heading):
        if self.centre_clearance(point)[0] >= OUTER_RADIUS_METRES+BODY_MARGIN_METRES:
            return True
        change = angle_difference(new_heading,old_heading)
        count = max(2,int(math.ceil(abs(change)/TURN_SAMPLE_RADIANS))+1)
        headings = np.linspace(old_heading,old_heading+change,count)
        centres = np.tile(point,(count,1))
        return bool(np.all(self.footprint_clearance(centres,headings) >= BODY_MARGIN_METRES))

    def segment_clear(self, start, end, heading):
        count = max(2,int(math.ceil(np.linalg.norm(end-start)/TRANSLATION_SAMPLE_METRES))+1)
        centres = np.linspace(start,end,count)
        room = self.centre_clearance(centres)
        tight = room < OUTER_RADIUS_METRES+BODY_MARGIN_METRES
        if np.any(tight) and np.any(self.footprint_clearance(centres[tight],heading) < BODY_MARGIN_METRES):
            return False
        for obstacle in self.course_map.low_obstacles:
            if np.min(np.linalg.norm(centres-obstacle.centre_metres,axis=1)) < obstacle.radius_metres+LASER_MARGIN_METRES:
                return False
        return True

    def crossing_clear(self, point, heading, line):
        along = line.end_metres-line.start_metres
        length = np.linalg.norm(along)
        along /= length
        normal = np.array([-along[1],along[0]])
        if normal@(line.far_side_point_metres-line.start_metres)<0:
            normal = -normal
        rotation = np.array([[math.cos(heading),-math.sin(heading)],[math.sin(heading),math.cos(heading)]])
        motors = MOTORS_METRES@rotation.T+point
        past = np.min((motors-line.start_metres)@normal)-PROPELLER_RADIUS_METRES
        across = (motors-line.start_metres)@along
        return past>.02 and across.min()-PROPELLER_RADIUS_METRES>=0 and across.max()+PROPELLER_RADIUS_METRES<=length


class ChicaneErrorMargin:
    """Apply the previous branch's tested drift cases while searching the final chicane.

These are declared planning assumptions, not measurements of simulator truth in flight.
They do not guarantee tolerance of arbitrary localization errors.
"""
    OFFSETS_METRES = np.array([[0.,0.],[-.03,.25],[0.,.25],[.03,.25]])

    def __init__(self, geometry):
        self.geometry = geometry
        self.course_map = geometry.course_map

    def centre_clearance(self, points):
        points = np.atleast_2d(points)
        expanded = (points[:,None,:]+self.OFFSETS_METRES).reshape(-1,2)
        return self.geometry.centre_clearance(expanded).reshape(len(points),-1).min(axis=1)

    def turn_clear(self, point, before, after):
        return all(self.geometry.turn_clear(point+offset,before,after) for offset in self.OFFSETS_METRES)

    def segment_clear(self, start, end, heading):
        return all(self.geometry.segment_clear(start+offset,end+offset,heading) for offset in self.OFFSETS_METRES)
