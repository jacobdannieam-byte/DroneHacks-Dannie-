import numpy as np
from dataclasses import replace
from refnav.launch_clearance import SimulatorLaunchReference,directional_error

def reference():
    return SimulatorLaunchReference('a'*64,'sim_local',0,0.,2.,(-2.,-2.),(2.,2.),(.3,.3,.1),.1,.005)

def check(ref=None,**changes):
    args=dict(start=(0.,0.,.1),end=(0.,0.,.8),error_m=(.08,.08,.541),frame_id='sim_local',
              epoch=0,source_sha256='a'*64,phase='climb',landed=True,contact_confirmed=True)
    args.update(changes); return (ref or reference()).check(**args)

def test_contact_and_vertical_uncertainty_retained():
    assert check() is None
    assert check(contact_confirmed=False)=='launch_floor_clearance'
    assert check(landed=False)=='launch_floor_clearance'
    assert check(end=(0.,0.,1.5))=='launch_side_or_ceiling_clearance'


def test_unmeasured_attitude_envelope_blocks_early_airborne_floor_transition():
    ref=replace(reference(),floor_z_m=-.04,body_half_extent_m=(.31,.31,.31),
                contact_height_m=.04,contact_tolerance_m=0.)
    # Resting contact can be reviewed, but that is not permission to ignore
    # the floor after native landed becomes false just above the pad.
    assert check(ref,start=(0.,0.,0.),end=(0.,0.,.8),error_m=(0.,0.,0.)) is None
    assert check(ref,start=(0.,0.,.02),end=(0.,0.,.8),error_m=(0.,0.,0.),
                 landed=False)=='launch_floor_clearance'
    assert check(ref,start=(0.,0.,.4),end=(0.,0.,.8),error_m=(0.,0.,0.),
                 landed=False) is None

def test_no_floor_or_route_exemption():
    assert check(phase='hover')=='launch_floor_clearance'
    assert check(phase='go')=='launch_reference_has_no_route_authority'
    assert check(end=(.1,0.,.8)) is None  # bounded correction to the home column
    assert check(end=(0.,0.,.05))=='launch_requires_nondecreasing_motion'
    assert check(epoch=1)=='launch_reference_mismatch'

def test_side_and_overhead_obstacles_block_contact_launch():
    for box in (((.2,-.1,0.),(.4,.1,.5)),((-.1,-.1,1.),(.1,.1,1.2))):
        assert check(replace(reference(),obstacles=(box,)))=='launch_obstacle_clearance'

def test_directional_rotation_preserves_vertical_bound():
    r=np.array([[0,-1,0],[1,0,0],[0,0,-1.]])
    assert np.allclose(directional_error((.08,.09,.541),r),(.09,.08,.541))


def test_bound_guard_checks_home_correction_and_live_transform():
    from types import SimpleNamespace as S
    from refnav.launch_clearance import BoundLaunchGuard
    from onboard.adapters.geometry import R_ENU_NED
    frame=S(frame_id='sim_local',epoch=0)
    alignment=S(map=frame,ned='ned',map_from_enu_yaw_rad=0.,ned_origin_in_map_m=(0.,0.,0.),
                provenance='review',position_error_m=0.,angle_error_rad=0.)
    # Obstacle intersects the correction toward the stored home, not the
    # previously checked vertical line at the current position.
    ref=replace(reference(),obstacles=(((.5,-.1,.5),(.6,.1,1.5)),))
    guard=BoundLaunchGuard(ref,lambda:True,(0.,0.,0.),R_ENU_NED,(0.,0.,0.),alignment=alignment)
    mission=S(home=(0.,.8,-.8,0.),sp=(0.,.8,-.8,0.),p=S(height_m=.1),phase='climb')
    vehicle=S(pos=(0.,0.,-.8),landed=False)
    inputs=dict(alignment=alignment,vehicle=S(tracking_error_m=0.))
    assert guard.check_owner(mission,vehicle,inputs)=='launch_obstacle_clearance'
    changed=S(**vars(alignment)); changed.map_from_enu_yaw_rad=.2
    assert guard.check_owner(mission,vehicle,dict(inputs,alignment=changed))=='launch_transform_changed'
    mission.sp=(.1,.8,-.8,0.)
    assert guard.check_owner(mission,vehicle,inputs)=='launch_command_outside_home_column'
    mission.home=(0.,0.,-.8,0.); mission.sp=(0.,0.,-.8,0.); mission.phase='go'
    output=S(phase='go',stream=True,setpoint=(.1,0.,-.9,0.))
    assert guard.check_output(mission,vehicle,inputs,output,phase='hover')=='launch_command_outside_home_column'
    output.setpoint=(0.,0.,-.9,0.)
    assert guard.check_output(mission,vehicle,inputs,output,phase='hover') is None
    assert guard.check_output(mission,vehicle,inputs,output,phase='go')=='launch_reference_has_no_route_authority'
    output.phase='land'
    assert guard.check_output(mission,vehicle,inputs,output,phase='hover')=='launch_reference_has_no_landing_authority'


def test_binder_requires_explicit_deployment_and_encloses_all_attitudes(tmp_path,monkeypatch):
    from types import SimpleNamespace as S
    from hashlib import sha256
    import pytest
    from examples.bind_simulator_launch import bind
    import simulator.gazebo_trial.calibration_lane as lane
    world=tmp_path/'world.sdf'; world.write_text('<sdf/>')
    model=tmp_path/'vehicle.sdf'
    model.write_text('<sdf><model name="body"><link name="body"><collision name="c"><geometry><box><size>0.6 0.2 0.08</size></box></geometry></collision></link></model></sdf>')
    monkeypatch.setattr(lane,'validate_world',lambda _:dict(interior_low_enu_m=(-2.,-2.,-.04),interior_high_enu_m=(2.,2.,3.)))
    alignment=S(map=S(frame_id='world',epoch=0),map_from_enu_yaw_rad=0.,ned_origin_in_map_m=(0.,0.,0.))
    runtime=S(adapter=S(simulated_estimator=True),loop=S(replay=S(hybrid=S(alignment=alignment))))
    settings=dict(scope='isolated_calibration_launch',world_file=world.name,resolved_vehicle_file=model.name,
                  world_sha256=sha256(world.read_bytes()).hexdigest(),resolved_vehicle_sha256=sha256(model.read_bytes()).hexdigest(),
                  world_frame_id='world',reviewed_error_ned_m=(0.,0.,0.),uncertainty_review='review')
    with pytest.raises(ValueError,match='launch-only authorization'): bind(runtime,settings,tmp_path)
    settings['deployment_authorization']=dict(scope='participant_approved_isolated_launch_only',
        world_sha256=settings['world_sha256'],resolved_vehicle_sha256=settings['resolved_vehicle_sha256'],
        reviewer='test fixture',navigation_authority=False,frame_id='world',epoch=0,world_from_map='identity_enu')
    guard=bind(runtime,settings,tmp_path)
    assert np.allclose(guard.reference.body_half_extent_m,[np.linalg.norm((.3,.1,.04))]*3)
    alignment.map_from_enu_yaw_rad=.1
    with pytest.raises(ValueError,match='world/map frame'): bind(runtime,settings,tmp_path)
