"""Offline isolated-lane geometry review. Never imports ROS or starts a simulator.

Generated collision geometry is design evidence, not observed policy clearance,
braking measurements, or a hardware envelope. Explicitly supply the archived or
current generated drone SDF so its identity can be recorded and checked again.
"""
import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from make_gazebo_world import drone_sdf, DRONE_16_WITH_CAMERA_MODEL
from course_world_sdf import world_sdf
from team_nav.course_map import load_course_map

MAP = ROOT / 'scenarios/calibration/straight_lane_v1.json'
BODY_SPHERE_M = .35  # Proposed all-attitude design envelope, not measured hardware.
POSITION_RESERVE_M = .25  # Includes <= .10 m allowed launch offset; not measured uncertainty.
LOW_ENU = (-.5, -.5, .75)
HIGH_ENU = (.5, 4., 1.25)


def numbers(text, count):
    values = tuple(float(v) for v in text.split())
    if len(values) != count or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid finite geometry')
    return values


def pose(element):
    node = element.find('pose')
    if node is None:
        return (0.,) * 6
    if node.get('relative_to'):
        raise ValueError('unresolved relative pose in vehicle model')
    return numbers(node.text, 6)


def collision_radius(model, offset=(0., 0., 0.)):
    """Conservative sphere using triangle inequality; rotating propellers fit.

    Only the existing flat box-collision model is accepted; unfamiliar nested
    models/includes or non-box collisions need a new geometry review.
    """
    if model.findall('include') or model.findall('model') or any(pose(model)):
        raise ValueError('unreviewed vehicle model structure or origin')
    rows = []
    for link in model.findall('link'):
        lp = pose(link)
        if any(lp[3:]):
            raise ValueError('unreviewed link rotation')
        for collision in link.findall('collision'):
            cp = pose(collision)
            geometry = collision.find('geometry')
            size = collision.find('geometry/box/size')
            if size is None or geometry is None or len(geometry) != 1:
                raise ValueError('unreviewed non-box collision')
            dims = numbers(size.text, 3)
            if min(dims) <= 0:
                raise ValueError('invalid collision dimensions')
            # Rotation cannot increase a box's circumscribed sphere.
            radius = (math.dist((0., 0., 0.), tuple(a+b for a, b in zip(offset, lp[:3])))
                      + math.hypot(*cp[:3]) + math.hypot(*dims) / 2)
            rows.append({'link': link.get('name'), 'collision': collision.get('name'),
                         'size_m': dims, 'sphere_radius_bound_m': radius})
    if not rows:
        raise ValueError('vehicle has no collision geometry')
    return rows


def validate_world(text):
    """Read back the generated collision faces and conservatively bound the lane."""
    world = ET.fromstring(text).find('world')
    origin = numbers(world.find("frame[@name='course']/pose").text, 6)
    if origin != (0., 2., -.04, 0., 0., 0.):
        raise ValueError('unexpected launch-origin transform')
    models = world.findall('model')
    if {m.get('name') for m in models} != {'floor', 'safety_net'} or len(models) != 2:
        raise ValueError('calibration world must have only floor and enclosure')
    include = world.findall('include')
    if (len(include) != 1 or include[0].findtext('uri') != 'model://drone_16_with_oak_d_s2'
            or numbers(include[0].findtext('pose'), 6) != (0., 0., 0., 0., 0., 1.5708)):
        raise ValueError('unexpected vehicle or start pose')
    net = world.find("model[@name='safety_net']")
    floor = world.find("model[@name='floor']")
    for model in (net, floor):
        if model.findtext('static') != 'true':
            raise ValueError('enclosure must remain static')
        model_pose = model.find('pose')
        if (model_pose.get('relative_to') != 'course'
                or any(numbers(model_pose.text, 6))):
            raise ValueError('unexpected enclosure transform')
        links = model.findall('link')
        if len(links) != 1 or any(pose(links[0])):
            raise ValueError('unexpected enclosure link transform')
    if len(floor.findall('.//collision')) != 1 or floor.find('.//geometry/plane') is None:
        raise ValueError('unexpected floor collision')
    if numbers(floor.findtext('.//plane/normal'), 3) != (0., 0., 1.):
        raise ValueError('unexpected floor normal')
    if any(numbers(floor.findtext('.//collision/pose'), 6)):
        raise ValueError('unexpected floor offset')
    sides = {}
    collisions = net.findall('.//collision')
    if len(collisions) != 5:
        raise ValueError('unexpected enclosure collision count')
    for collision in collisions:
        p = pose(collision)
        size = numbers(collision.findtext('geometry/box/size'), 3)
        if any(p[3:]) or min(size) <= 0:
            raise ValueError('unexpected enclosure rotation or size')
        center = tuple(a+b for a, b in zip(origin[:3], p[:3]))
        sides[collision.get('name')] = (center, size)
    expected = {'net_side_west_collision', 'net_side_east_collision',
                'net_side_south_collision', 'net_side_north_collision', 'net_roof_collision'}
    if set(sides) != expected:
        raise ValueError('unexpected enclosure collisions')
    expected_sizes = {'net_side_west_collision': (.1, 8.2, 3.048),
                      'net_side_east_collision': (.1, 8.2, 3.048),
                      'net_side_south_collision': (6.2, .1, 3.048),
                      'net_side_north_collision': (6.2, .1, 3.048),
                      'net_roof_collision': (6.2, 8.2, .1)}
    if any(sides[name][1] != size for name, size in expected_sizes.items()):
        raise ValueError('unexpected enclosure coverage')
    def face(name, axis, sign):
        center, size = sides[name + '_collision']
        return center[axis] + sign * size[axis] / 2
    low = (face('net_side_west', 0, 1), face('net_side_south', 1, 1), origin[2])
    high = (face('net_side_east', 0, -1), face('net_side_north', 1, -1), face('net_roof', 2, -1))
    if (not all(math.isclose(a, b, abs_tol=1e-9) for a,b in zip(low, (-3., -2., -.04)))
            or not all(math.isclose(a, b, abs_tol=1e-9) for a,b in zip(high, (3., 6., 3.008)))):
        raise ValueError('unexpected enclosure extent')
    inflated = BODY_SPHERE_M + POSITION_RESERVE_M
    margins = {name: value for name, value in zip(
        ('west', 'south', 'floor', 'east', 'north', 'roof'),
        [a-b-inflated for a,b in zip(LOW_ENU, low)] +
        [a-b-inflated for a,b in zip(high, HIGH_ENU)])}
    if min(margins.values()) <= 0:
        raise ValueError('proposed navigation envelope intersects world geometry')
    return {'interior_low_enu_m': low, 'interior_high_enu_m': high,
            'navigation_low_enu_m': LOW_ENU, 'navigation_high_enu_m': HIGH_ENU,
            'inflated_body_radius_m': inflated, 'navigation_clearance_design_m': margins}


def review(generated_drone, map_path=MAP):
    course = load_course_map(map_path)
    if course.walls or course.tubes or course.low_obstacles or course.checkpoint_lines:
        raise ValueError('calibration lane must remain separate and obstacle free')
    if tuple(course.launch_mark_centre_metres) != (0., -2.):
        raise ValueError('unexpected launch mark')
    text = world_sdf(course, drone_sdf(DRONE_16_WITH_CAMERA_MODEL, 0., 0.), (0., 2., -.04))
    geometry = validate_world(text)
    model_path = Path(generated_drone)
    rows = collision_radius(ET.parse(model_path).getroot().find('model'))
    camera_path = HERE / 'models/oak_d_s2_camera/model.sdf'
    wrapper_path = HERE / 'models/drone_16_with_oak_d_s2/model.sdf'
    wrapper = ET.parse(wrapper_path).getroot().find('model')
    if (wrapper.findall('link') or wrapper.findall('model') or any(pose(wrapper)) or len(wrapper.findall('include')) != 2
            or {e.findtext('uri') for e in wrapper.findall('include')} !=
            {'model://drone_16_like', 'model://oak_d_s2_camera'}):
        raise ValueError('unreviewed wrapper model')
    base_include = next(e for e in wrapper.findall('include')
                        if e.findtext('uri') == 'model://drone_16_like')
    if any(pose(base_include)):
        raise ValueError('unreviewed base mount offset')
    camera_include = next(e for e in wrapper.findall('include')
                          if e.findtext('uri') == 'model://oak_d_s2_camera')
    camera_pose = pose(camera_include)
    if any(camera_pose[3:]):
        raise ValueError('unreviewed camera mount rotation')
    rows += collision_radius(ET.parse(camera_path).getroot().find('model'), camera_pose[:3])
    radius = max(row['sphere_radius_bound_m'] for row in rows)
    if radius > BODY_SPHERE_M:
        raise ValueError('generated body exceeds proposed design envelope')
    sources = {p.relative_to(ROOT).as_posix() if p.is_relative_to(ROOT) else 'externally_supplied_generated_drone_model.sdf':
               sha256(p.read_bytes()).hexdigest() for p in
               (Path(map_path), model_path, camera_path, wrapper_path,
                HERE / 'make_drone_16_model.py', HERE / 'course_world_sdf.py',
                Path(__file__).resolve())}
    return text, {'status': 'OFFLINE_DESIGN_REVIEW_ONLY_NOT_FLIGHT_APPROVAL',
                  'policy_positive_clearance': False, 'hardware_qualified': False,
                  'source_sha256': sources, 'world_sha256': sha256(text.encode()).hexdigest(),
                  'body_sphere_design_m': BODY_SPHERE_M, 'position_reserve_design_m': POSITION_RESERVE_M,
                  'generated_collision_radius_bound_m': radius, 'collision_shapes': rows,
                  'geometry': geometry,
                  'limits': ['Not measured stopping evidence.', 'No native NED-origin validation here.',
                             'Navigation floor margin excludes takeoff and landing contact phases.',
                             'Forward camera cannot observe its own near/rear body volume.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--generated-drone', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    text, report = review(args.generated_drone)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir / 'straight_lane_v1.sdf').write_text(text, encoding='utf-8', newline='\n')
    (args.output_dir / 'geometry-review.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'status': report['status'], 'world_sha256': report['world_sha256']}))


if __name__ == '__main__':
    main()
