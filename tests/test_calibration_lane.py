"""Offline geometry tests; synthetic fixtures grant no flight authority."""
from copy import deepcopy
import math
import xml.etree.ElementTree as ET

import pytest

from simulator.gazebo_trial.calibration_lane import (
    BODY_SPHERE_M, MAP, collision_radius, drone_sdf, load_course_map, validate_world, world_sdf,
)


def lane():
    return world_sdf(load_course_map(MAP), drone_sdf('drone_16_with_oak_d_s2', 0., 0.), (0., 2., -.04))


def test_separate_lane_has_exact_generated_navigation_reserves():
    report = validate_world(lane())
    assert report['interior_low_enu_m'] == pytest.approx((-3., -2., -.04))
    assert report['interior_high_enu_m'] == pytest.approx((3., 6., 3.008))
    assert report['navigation_clearance_design_m'] == pytest.approx(
        {'west': 1.9, 'east': 1.9, 'south': .9, 'north': 1.4, 'floor': .19, 'roof': 1.158})
    course = load_course_map(MAP)
    assert not course.walls and not course.tubes and not course.low_obstacles
    assert 'not a physical survey' in course.description


@pytest.mark.parametrize('mutation', ['floor', 'roof', 'extra_obstacle', 'short_wall', 'launch', 'dynamic'])
def test_changed_generated_geometry_requires_review(mutation):
    root = ET.fromstring(lane())
    world = root.find('world')
    if mutation == 'floor':
        world.find("model[@name='floor']//collision/pose").text = '0 0 .8 0 0 0'
    elif mutation == 'roof':
        world.find("model[@name='safety_net']//collision[@name='net_roof_collision']/pose").text = '0 0 1.4 0 0 0'
    elif mutation == 'extra_obstacle':
        extra = deepcopy(world.find("model[@name='safety_net']"))
        extra.set('name', 'unexpected_obstacle')
        world.append(extra)
    elif mutation == 'short_wall':
        world.find("model[@name='safety_net']//collision[@name='net_side_west_collision']/geometry/box/size").text = '.1 2 3.048'
    elif mutation == 'dynamic':
        world.find("model[@name='safety_net']/static").text = 'false'
    else:
        world.find('include/pose').text = '0 1 0 0 0 1.5708'
    with pytest.raises(ValueError):
        validate_world(ET.tostring(root, encoding='unicode'))


def test_rotating_propeller_box_fits_conservative_all_attitude_sphere():
    # Synthetic box dimensions match the archived generated geometry; this is
    # a mathematical coverage check, not a fresh generated-model measurement.
    model = ET.fromstring('''<model name="fixture"><link name="rotor">
      <pose>-.15 .14 .06 0 0 0</pose><collision name="rotor">
      <geometry><box><size>.1778 .016923076923076923 .0008461538461538462</size></box></geometry>
      </collision></link></model>''')
    bound = collision_radius(model)[0]['sphere_radius_bound_m']
    assert bound == pytest.approx(.3030783638885472)
    assert bound < BODY_SPHERE_M
    for yaw in (0., .7, 1.4, 2.1):
        for x in (-.0889, .0889):
            for y in (-.016923076923076923 / 2, .016923076923076923 / 2):
                point = (-.15 + x * math.cos(yaw) - y * math.sin(yaw),
                         .14 + x * math.sin(yaw) + y * math.cos(yaw), .06042307692307692)
                assert math.hypot(*point) <= bound


def test_unresolved_model_geometry_cannot_be_silently_ignored():
    model = ET.fromstring('<model><include><uri>model://unreviewed</uri></include></model>')
    with pytest.raises(ValueError, match='structure'):
        collision_radius(model)
    model = ET.fromstring('<model><link><collision><geometry><sphere><radius>1</radius></sphere></geometry></collision></link></model>')
    with pytest.raises(ValueError, match='non-box'):
        collision_radius(model)


def test_wrapper_bracket_cannot_be_omitted_from_body_bound(tmp_path, monkeypatch):
    from simulator.gazebo_trial import calibration_lane as lane_module
    original = lane_module.HERE
    camera = tmp_path / 'models/oak_d_s2_camera/model.sdf'
    camera.parent.mkdir(parents=True)
    camera.write_bytes((original / 'models/oak_d_s2_camera/model.sdf').read_bytes())
    wrapper_path = tmp_path / 'models/drone_16_with_oak_d_s2/model.sdf'
    wrapper_path.parent.mkdir(parents=True)
    wrapper = ET.parse(original / 'models/drone_16_with_oak_d_s2/model.sdf')
    ET.SubElement(wrapper.getroot().find('model'), 'link', name='unreviewed_bracket')
    wrapper.write(wrapper_path)
    model = tmp_path / 'generated.sdf'
    model.write_text('<sdf><model><link><collision><geometry><box><size>.2 .2 .08</size></box></geometry></collision></link></model></sdf>')
    monkeypatch.setattr(lane_module, 'HERE', tmp_path)
    with pytest.raises(ValueError, match='wrapper'):
        lane_module.review(model)
