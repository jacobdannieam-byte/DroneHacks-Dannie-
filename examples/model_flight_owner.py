"""Separate exact-approval entry point for the Classical-world PINN experiment."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import sys

import numpy as np

from examples.calibration_trial_owner import (_run, _read, OWNER_ID, PARAMETER_OVERRIDES,
                                             verify_recorder_ready)
from examples.gazebo_container import isolated, sha
from examples.h1_observation import ClockMapping
from refnav.calibration_trial import CalibrationPlan
from refnav.dynamics import DroneState
from refnav.model_flight import DecisionLog, ModelFlight, ModelSelector, REFERENCE

ROOT = Path(__file__).resolve().parents[1]
PROFILE = 'scenarios/model_flight/h1_pinn_prefix.json'
SOURCES = ('simulator/gazebo_trial/make_drone_16_model.py',
           'simulator/gazebo_trial/course_world_sdf.py', 'simulator/gazebo_trial/make_gazebo_world.py',
           'simulator/gazebo_trial/models/drone_16_with_oak_d_s2/model.sdf',
           'simulator/gazebo_trial/models/oak_d_s2_camera/model.sdf',
           'simulator/px4_changes/drone_16_settings.sh')


def geometry_review(raw):
    """Static full corridor check, with floor contact explicitly excluded from obstacles.

    This is a simulator design reservation, not a measured probabilistic uncertainty.
    .35 m horizontal body/design envelope + .10 m extra reserve; the lattice
    discretization error is subtracted from the nearest sampled distance.
    """
    sys.path.insert(0, str(ROOT/'drone_code/team_nav'))
    from team_nav.course_map import load_course_map, shapes_blocking_between_heights, distance_to_shapes_metres
    config = raw['model_flight']; plan = CalibrationPlan(**raw['plan'])
    if config['registration_enabled'] is not False or config['fallback_allowed'] is not False:
        raise ValueError('Registration and Classical fallback must be off')
    if sha(config['participant_map']) != config['participant_map_sha256']:
        raise ValueError('Participant map changed')
    course = load_course_map(config['participant_map'])
    low, high = np.array(plan.minimum_offset_m), np.array(plan.maximum_offset_m)
    north = np.linspace(low[0], high[0], math.ceil((high[0]-low[0])/.01)+1)
    east = np.linspace(-config['cross_track_limit_m'], config['cross_track_limit_m'], 6)
    points = np.array([(e+config['route_east_per_north']*n+course.launch_mark_centre_metres[0], n+course.launch_mark_centre_metres[1]) for n in north for e in east])
    shapes = shapes_blocking_between_heights(course, 0., -low[2]+.20)
    rooms, _ = distance_to_shapes_metres(points, shapes)
    margin = float(min(rooms)) - .45 - math.sqrt(2)*.005
    if margin <= 0:
        raise ValueError('Full launch/transit/landing corridor fails participant-map reserve')
    for point in config['waypoints_ned_m']:
        if not np.all(np.asarray(point) >= config['command_low_ned_m']) or not np.all(np.asarray(point) <= config['command_high_ned_m']):
            raise ValueError('Waypoint outside command corridor')
    return dict(scope='simulator static prior; not observed FREE or general D3 certification',
        minimum_remaining_horizontal_margin_m=margin, envelope_m=.35, additional_design_reserve_m=.10,
        sample_cover_error_m=math.sqrt(2)*.005, floor='authored flat support plane; landing contact permitted',
        height_slab_m=[0.,-low[2]+.20], registration_enabled=False,
        source_sha256={name:sha(ROOT/name) for name in SOURCES})


def approval_for(path, identity, contract_path, *, now=None):
    approval = _read(path); contract = _read(contract_path)
    now = datetime.now(timezone.utc) if now is None else now
    expires = datetime.fromisoformat(approval['expires_utc'].replace('Z','+00:00'))
    if (approval.get('mode') != 'hybrid_model_trial' or expires.tzinfo is None or expires <= now
        or any(approval.get(k) != v for k,v in identity.items())
        or approval.get('recording_contract_sha256') != sha(contract_path)
        or approval.get('parameter_overrides') != PARAMETER_OVERRIDES
        or approval.get('trial_id') != 'hybrid-pinn-classical-prefix-001'
        or not all(approval.get(k) is True for k in ('isolation_reviewed','recovery_reviewed','recording_ready',
            'allow_failed_simulator_termination','experimental_model_control_reviewed','static_prior_corridor_reviewed'))
        or not all(isinstance(approval.get(k),str) and approval[k].strip() for k in ('reviewed_by','recovery_operator'))):
        raise ValueError('Exact current Hybrid model-control approval required; C1 approval is invalid')
    if (contract['trial_id'] != approval['trial_id'] or contract['profile_sha256'] != identity['profile_sha256']
        or contract['owner_id'] != OWNER_ID or contract['trial_kind'] != 'stopping'
        or contract['review_status'] != 'reviewed' or contract['frame_id'] != 'px4_local_ned'
        or contract['topics']['events'] != '/qualification/events'):
        raise ValueError('Model trial recorder binding invalid')
    return approval, expires, contract


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('profile','world','vehicle','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--revision',required=True);parser.add_argument('--image-sha256',required=True)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--disarmed-check',action='store_true');group.add_argument('--approval',type=Path)
    parser.add_argument('--authorize-only',action='store_true')
    parser.add_argument('--recording-contract',type=Path)
    parser.add_argument('--recorder-pid',type=int);parser.add_argument('--bag-pid',type=int)
    parser.add_argument('--recorder-ready',type=Path)
    args=parser.parse_args()
    if not isolated() or not re.fullmatch('[0-9a-f]{40}',args.revision) or not re.fullmatch('sha256:[0-9a-f]{64}',args.image_sha256):
        raise ValueError('Isolated pinned SITL required')
    if args.profile.resolve() != (ROOT/PROFILE).resolve():
        raise ValueError('Only the bounded PINN prefix profile is supported')
    raw=_read(args.profile);config=raw['model_flight'];plan=CalibrationPlan(**raw['plan'])
    mapping=ClockMapping(**raw['clock_mapping']);geometry=geometry_review(raw)
    identity=dict(revision=args.revision,image_sha256=args.image_sha256,world_sha256=sha(args.world),
        vehicle_sha256=sha(args.vehicle),profile_sha256=sha(args.profile),ros_domain_id=int(os.environ['ROS_DOMAIN_ID']),
        checkpoint_sha256=config['checkpoint_sha256'],participant_map_sha256=config['participant_map_sha256'],
        registration_enabled=False,command_selection='physics_informed_residual_model',fallback_allowed=False)
    approval=expires=contract=ready=None
    if args.authorize_only and args.approval is None:
        raise ValueError('Authorize-only requires explicit approval')
    if args.approval:
        approval,expires,contract=approval_for(args.approval,identity,args.recording_contract)
        if not args.authorize_only:
            if None in (args.recorder_pid,args.bag_pid,args.recorder_ready):
                raise ValueError('Both live recorders required')
            ready=verify_recorder_ready(args.recorder_ready,contract,approval,args.recorder_pid)
    selector=ModelSelector(config)  # Invalid/missing weights cannot fall back to D0.
    def preview(obs,now):
        # Actual disarmed state only. Ground hypotheses are expected to fail the
        # airborne corridor; never fabricate an airborne state to admit flight.
        state=DroneState(REFERENCE,obs.position_stamp_ns/1e9,obs.position,obs.velocity,obs.heading_rad,0.)
        try:
            _,decision=selector.choose(state,config['waypoints_ned_m'][0],now)
            return dict(command_authority=False,decision=decision)
        except ValueError as error:
            return dict(command_authority=False,ground_state_rejected=str(error))
    with args.output.open('x') as out:
        if args.authorize_only:
            json.dump(dict(identity,authorized_artifact=True,geometry=geometry),out,indent=2);return 0
        log=DecisionLog(args.output.with_suffix('.decisions.jsonl'))
        try:
            driver=ModelFlight(plan,config,log,selector=selector)
            report,code=_run(plan,mapping,approval=approval,expires=expires,contract=contract,identity=identity,
                seconds=20.,recorder_pid=args.recorder_pid,bag_pid=args.bag_pid,recorder_ready=ready,
                driver=driver,preview=preview)
            report.update(mode='hybrid_model_trial' if approval else 'hybrid_model_disarmed',geometry=geometry,
                          model_command_count=driver.model_commands,waypoints_reached=driver.waypoint)
            json.dump(report,out,indent=2,allow_nan=False)
            return code
        finally:log.close()


if __name__=='__main__':
    raise SystemExit(main())
