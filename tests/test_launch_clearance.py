import numpy as np
from dataclasses import asdict, replace
from itertools import product
import json
import math
from types import SimpleNamespace as S

import pytest

from onboard.adapters.geometry import R_ENU_NED
from refnav.core import Output, Vehicle
from refnav.launch_clearance import BoundLaunchGuard, ContactTransition
from refnav.launch_clearance import SimulatorLaunchReference,directional_error
from refnav.world_model.spatial_input import NS

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
    alignment.map_from_enu_yaw_rad=0.
    transition_file = tmp_path / 'transition.json'
    document = dict(scope='reviewed_model_conditioned_isolated_sitl_contact',
                    world_sha256=settings['world_sha256'], resolved_vehicle_sha256=settings['resolved_vehicle_sha256'],
                    support_collisions=['body/c'], bounds=asdict(synthetic_contact()))
    transition_file.write_text(json.dumps(document), encoding='utf-8')
    digest = sha256(transition_file.read_bytes()).hexdigest()
    settings['contact_transition'] = dict(file=transition_file.name, sha256=digest)
    with pytest.raises(ValueError, match='reviewed contact-transition identity'):
        bind(runtime, settings, tmp_path)
    settings['deployment_authorization']['contact_transition_sha256'] = digest
    guard = bind(runtime, settings, tmp_path)
    assert guard.transition.contract == synthetic_contact()
    assert np.count_nonzero(guard.transition.support) == 4 and guard.source_valid()
    transition_file.write_text('changed', encoding='utf-8')
    assert not guard.source_valid()


def synthetic_contact():
    return ContactTransition('synthetic geometry and reachable bounds only', (0., 0.), .6, .01, .04, .02, .08,
                             .004, .003, .003, .025, .15, .5, .2, .6, .1,
                             80_000_000, 100_000_000, 1_050_000_000, 4 * NS, 8 * NS)


class LiftTrace:
    """Measured-state replay of an idealized isolated pad, with no motion output."""

    def __init__(self, **options):
        self.valid = True
        ref = replace(reference(), floor_z_m=-.04, body_half_extent_m=(.31,) * 3,
                      contact_height_m=.04, contact_tolerance_m=0.)
        vertices = [np.array((.1, .1, .04)) * corner for corner in product((-1., 1.), repeat=3)]
        vertices += [np.array((.25, .1, .01)) * corner + (.02, 0., .03)
                     for corner in product((-1., 1.), repeat=3)]
        support = [point[2] == -.04 for point in vertices]
        self.alignment = S(map=S(frame_id='sim_local', epoch=0), ned='ned', map_from_enu_yaw_rad=0.,
                           ned_origin_in_map_m=(0., 0., 0.), provenance='fixture', position_error_m=0.,
                           angle_error_rad=0., time_origin_ns=0)
        self.guard = BoundLaunchGuard(ref, lambda: self.valid, (.01, .01, .03), R_ENU_NED, (0., 0., 0.),
                                     alignment=self.alignment, collision_vertices=vertices, support_vertices=support,
                                     contact_transition=replace(synthetic_contact(), **options))
        self.mission = S(home=(0., 0., 0., 0.), sp=(0., 0., 0., 0.), p=S(height_m=.8, climb_rate_mps=.2), phase='wait')
        self.now = self.wall = NS

    def step(self, height=0., *, command=None, phase='climb', landed=False, armed=True, advance=50_000_000,
             wall_advance=None, reset=0, q=(1., 0., 0., 0.), north=0., speed=.1, attitude_age=0, commit=True):
        self.now += advance
        self.wall += advance if wall_advance is None else wall_advance
        self.mission.phase = phase
        down = -height
        state = S(timestamp_s=self.now / NS, position_m=(north, 0., down), velocity_mps=(0., 0., -speed))
        inputs = dict(state=state, alignment=self.alignment, estimator_reset_counter=0, now_ns=self.now,
                      vehicle=S(tracking_error_m=0., state_position_error_m=.01))
        vehicle = Vehicle(pos=state.position_m, q=q, landed=landed, armed=armed,
                          attitude_capture_ns=self.now-attitude_age, attitude_received_ns=self.now,
                          attitude_reset_counter=reset, landed_capture_ns=self.now)
        setpoint = (0., 0., -command if command is not None else down, 0.)
        self.mission.sp = setpoint
        kwargs = dict(now_ns=self.now, wall_ns=self.wall)
        result = self.guard.check_owner(self.mission, vehicle, inputs, **kwargs)
        if not commit or result is not None:
            return result
        out = Output(phase, setpoint, stream=True)
        result = self.guard.check_output(self.mission, vehicle, inputs, out, phase=phase, **kwargs)
        return result or self.guard.freshness_reason(vehicle, inputs, **kwargs)

    def anchor(self):
        while self.guard.transition.rest is None or self.now - self.guard.transition.rest[2] < 50_000_000:
            assert self.step(phase='wait', armed=False, landed=True, speed=0.) == 'launch_transition_contact_dwell'
        assert self.step(phase='wait', armed=False, landed=True, speed=0.) is None

    def climb(self):
        for centimetres in range(2, 41):
            assert self.step((centimetres - 1) / 100, command=centimetres / 100) is None


def test_contact_transition_admits_pad_contact_after_landed_false_then_hands_off():
    trace = LiftTrace()
    for expected in ('launch_transition_contact_dwell', 'launch_transition_contact_dwell', None):
        assert trace.step(phase='wait', armed=False, landed=True, speed=0., commit=False) == expected
    assert trace.guard.transition.anchor is None  # precheck cannot consume the anchor
    trace.anchor()
    clocks = trace.guard.transition.clocks
    assert trace.step(phase='arm', armed=True, landed=True, speed=0.) is None
    # C0 showed this ordering: the detector released before the center rose.
    assert trace.step(0., command=.01, landed=False, speed=0.) is None
    trace.climb()
    assert trace.guard.transition.handed_off and trace.guard.transition.clocks == clocks
    assert trace.step(.40, phase='hover') is None
    assert trace.step(.02, command=.40, landed=True) == 'launch_floor_clearance_after_handoff'
    assert trace.step(.50, command=.40) == 'launch_floor_clearance_after_handoff'


@pytest.mark.parametrize(('changes', 'reason'), (
    ({'attitude_age': 80_000_001}, 'launch_transition_telemetry_unverified'),
    ({'wall_advance': 250_000_001}, 'launch_transition_control_gap'),
    ({'reset': 1}, 'launch_transition_reset'),
    ({'north': .11}, 'launch_transition_drift_or_height'),
    ({'q': (math.cos(.10), math.sin(.10), 0., 0.)}, 'launch_transition_attitude'),
    ({'command': .081}, 'launch_transition_command_rate'),
    ({'speed': .51}, 'launch_transition_state_or_speed'),
    ({'speed': -.01}, 'launch_transition_descending_state'),
))
def test_contact_transition_violations_latch(changes, reason):
    trace = LiftTrace()
    trace.anchor()
    assert trace.step(**changes) == reason
    assert trace.step(.4) == reason


def test_contact_transition_never_accepts_nonsupport_floor_intersection():
    trace = LiftTrace(max_tilt_rad=.20, attitude_error_rad=.10, max_angular_rate_rad_s=.05)
    # This uncertainty cannot prove the low in-flight protrusion clear even on the pad.
    assert trace.step(phase='wait', armed=False, landed=True, speed=0.) == 'launch_transition_nonsupport_floor_intersection'


def test_contact_transition_clocks_and_source_cannot_be_reacquired():
    trace = LiftTrace(max_ros_ns=120_000_000, max_wall_ns=250_000_000)
    trace.anchor()
    assert trace.step(phase='arm', landed=True) is None
    assert trace.step(phase='arm', landed=True) is None
    assert trace.step(phase='arm', landed=True) == 'launch_transition_clock_or_deadline'
    for failure in ('source', 'owner'):
        trace = LiftTrace()
        trace.anchor()
        if failure == 'source':
            trace.valid = False
            reason = 'launch_source_changed'
        else:
            trace.mission = S(**vars(trace.mission))
            reason = 'launch_transition_owner_changed'
        assert trace.step() == reason
        trace.valid = True
        assert trace.step(.4) == reason


def test_contact_transition_handoff_preserves_reset_and_state_freshness():
    trace = LiftTrace()
    trace.anchor()
    assert trace.step(command=.01, speed=0.) is None
    trace.climb()
    assert trace.guard.transition.handed_off
    assert trace.step(.5, reset=1, phase='hover') == 'launch_transition_reset'


def test_contact_transition_limits_support_command_lead():
    trace = LiftTrace()
    trace.anchor()
    for centimetres in range(1, 9):
        assert trace.step(0., command=centimetres / 100, speed=0.) is None
    assert trace.step(0., command=.09, speed=0.) == 'launch_transition_support_clearance'


def test_contact_dwell_cannot_cross_attitude_reset():
    trace = LiftTrace()
    for reset in (0, 0, 1, 1):
        assert trace.step(phase='wait', armed=False, landed=True, speed=0., reset=reset) == (
            'launch_transition_contact_dwell')
    assert trace.step(phase='wait', armed=False, landed=True, speed=0., reset=1) is None


def test_hybrid_owner_wait_to_arm_anchors_new_home_and_checks_actual_climb_output(monkeypatch):
    from examples.hybrid_mission_smoke import runtime_fixture
    from refnav.core import Mission, NAV_OFFBOARD, Params

    trace = LiftTrace()
    runtime = runtime_fixture()
    runtime.launch_guard = trace.guard
    mission = Mission(Params(height_m=.8, climb_rate_mps=.2, stream_lead_s=.10, health_quiet_s=0., hover_s=.5),
                      hybrid_navigation=True)
    prepared = {}
    monkeypatch.setattr(runtime.loop, 'observation_reason', lambda *_: None)
    monkeypatch.setattr(runtime.loop.replay, 'prepare', lambda _: (None, prepared))
    now = NS

    def tick(height=0., *, armed=False, landed=True):
        nonlocal now
        now += 50_000_000
        capture = now - 1_000_000
        position = (0., 0., -height)
        prepared.update(state=S(timestamp_s=capture / NS, position_m=position,
                                velocity_mps=(0., 0., -.2 if height else 0.)),
                        alignment=trace.alignment, estimator_reset_counter=0, now_ns=now,
                        vehicle=S(tracking_error_m=0., state_position_error_m=.01))
        vehicle = Vehicle(pos=position, heading=0., pos_ok=True, q=(1., 0., 0., 0.), armed=armed, landed=landed,
                          nav_state=NAV_OFFBOARD if armed else 0, attitude_capture_ns=capture,
                          attitude_received_ns=now, attitude_reset_counter=0, landed_capture_ns=capture)
        mission.on_health(now / NS, True)
        mission.depth_t = mission.depth_ready_since = now / NS
        output = runtime.owner_step(mission, vehicle, now_ns=now, wall_ns=now, generation=1,
                                    admission=True, owner_ready=True)
        assert runtime.publication_reason(mission, vehicle, output, now, wall_ns=now) is None
        if output.stream:
            assert runtime.publication_freshness_reason(now, wall_ns=now, vehicle=vehicle) is None
        return output

    assert mission.home is None
    for _ in range(2):
        wait = tick()
        assert wait.phase == 'wait' and not wait.stream and not wait.commands
        assert trace.guard.transition.anchor is None and mission.home is None
    arm = tick()
    assert arm.phase == 'arm' and arm.stream and not arm.commands
    assert trace.guard.transition.home[0] == mission.home == (0., 0., 0., 0.)
    assert trace.guard.transition.clocks == (now, now)
    commands = [command for _ in range(3) for command in tick().commands]
    assert commands == ['offboard', 'arm']
    assert tick(armed=True).phase == 'climb'
    first = tick(armed=True, landed=False)
    assert first.phase == 'climb' and first.stream and first.setpoint[2] == pytest.approx(-.01)
    for centimetres in range(1, 36):
        out = tick(centimetres / 100, armed=True, landed=False)
        assert out.phase == 'climb' and out.stream and not out.commands
        assert out.setpoint[2] == pytest.approx(-(centimetres + 1) / 100)
    assert trace.guard.transition.handed_off
