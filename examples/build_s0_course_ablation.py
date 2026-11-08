"""Participant-route inputs for the explicitly unqualified S0 simulator experiment.

No Classic follower, simulator truth query, arming, or measured-bound fabrication.
"""
import argparse
from dataclasses import asdict
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import numpy as np

from examples.build_disarmed_shadow_profiles import diagnostic_profile
from refnav.reasoning.schema import MissionGoal
from simulator.gazebo_trial.make_hybrid_profile import assemble

ROOT = Path(__file__).resolve().parents[1]
SCOPE = 's0_participant_map_unqualified_sitl'


def ground_alignment(probe, probe_sha256):
    """Fit translation only from synchronized stationary native estimates.

    Authored axis rotation is unchanged. This is observed frame alignment, not
    a change to sensor positions, floor geometry or qualification uncertainty.
    """
    last = probe['last_observations']
    now = last['/clock']['sim_ns']
    visual = last['/gazebo_hybrid/simulated_estimator/odometry']
    capture = visual['capture_ns']
    local = last['/fmu/out/vehicle_local_position']['evidence'][0]
    if (visual['header_frame'] != 'odom' or visual['body_frame'] != 'base_link'
            or not 0 <= now-capture <= 250_000_000
            or not capture <= visual['receipt_ns'] <= now
            or not local['valid'] or not 0 <= now-local['stamp_ns'] <= 100_000_000
            or not 0 <= local['speed_mps'] <= .05):
        raise ValueError('fresh stationary native/visual ground alignment required')
    rows = probe['px4_clock_samples']
    reset = last['/fmu/out/vehicle_odometry']['evidence'][0]['reset']
    samples = []
    for row in rows:
        evidence = row['evidence'][0]
        stamp = evidence['stamp_ns']
        point = np.asarray(row['position'], dtype=float)
        if (evidence['valid'] is not True or evidence['reset'] != reset or evidence['pose_frame'] != 1
                or point.shape != (3,) or not np.isfinite(point).all()
                or not stamp <= row['receipt_ns'] <= now):
            raise ValueError('invalid native ground alignment sample')
        if samples and stamp <= samples[-1][0]:
            raise ValueError('nonadvancing native ground alignment samples')
        samples.append((stamp, point))
    recent = [(stamp, point) for stamp, point in samples if now-500_000_000 <= stamp <= now]
    if (len(recent) < 3 or recent[-1][0]-recent[0][0] < 400_000_000
            or np.linalg.norm(np.ptp(np.array([point for _, point in recent]), axis=0)) > .02):
        raise ValueError('native ground alignment has not settled')
    before = [(stamp, point) for stamp, point in samples if stamp <= capture]
    after = [(stamp, point) for stamp, point in samples if stamp >= capture]
    if not before or not after or after[0][0]-before[-1][0] > 20_000_000:
        raise ValueError('visual capture lacks synchronized native bracket')
    t0, p0 = before[-1]
    t1, p1 = after[0]
    native = p0 if t0 == t1 else p0+(p1-p0)*((capture-t0)/(t1-t0))
    optical_pose = np.asarray(visual['position'], dtype=float)
    if optical_pose.shape != (3,) or not np.isfinite(optical_pose).all():
        raise ValueError('invalid visual ground alignment position')
    translation = optical_pose - native*np.array([1., -1., -1.])
    return dict(method='stationary-native-to-visual-translation-v1', probe_sha256=probe_sha256,
        visual_capture_ns=capture, native_bracket_ns=[t0,t1], native_reset_counter=reset[0],
        native_position_ned_m=native.tolist(), visual_position_odom_m=optical_pose.tolist(),
        ned_origin_in_map_m=translation.tolist(), stationary_span_ns=recent[-1][0]-recent[0][0],
        native_position_range_m=np.ptp(np.array([point for _, point in recent]), axis=0).tolist(),
        source='paired native estimated poses, not simulator truth or altered sensor data')


def participant_geometry(map_path):
    """Load participant-authored geometry; caller applies the native/map transform."""
    package = str(ROOT / 'drone_code/team_nav')
    if package not in sys.path:
        sys.path.insert(0, package)
    from team_nav.course_map import load_course_map
    return load_course_map(map_path)


def course_document(map_path, mission_path, reference):
    course = participant_geometry(map_path)
    from team_nav.mission import load_mission
    from team_nav.route_planner import plan_route
    mission = load_mission(mission_path)
    route = plan_route(course, mission)
    points = route.waypoints_metres
    if not 2 <= len(points) <= 32:
        raise ValueError('participant route needs 2..32 stages')
    if len(route.checkpoints_cleared) != len(mission.checkpoints):
        raise ValueError('participant route does not cross every mission checkpoint')
    stages = []
    launch_x, launch_y = course.launch_mark_centre_metres
    for index, point in enumerate(points):
        local = [float(point[1]-launch_y), float(launch_x-point[0]), float(point[2]-.04)]
        stages.append(dict(stage_id=f'course_{index:02}', kind='waypoint',
                           position_m=local, arrival_m=.15, timeout_s=120.,
                           semantic=asdict(MissionGoal()), metric_fallback=True))
    stages[-1].update(kind='landing', landing_site_review='experimental-participant-pad-not-qualified')
    return dict(reference={k: reference[k] for k in ('frame_id', 'clock_id', 'epoch')},
                route_review='User-requested isolated simulation ablation; participant route prior, not observed clearance',
                recovery={'failure_limit': 4, 'no_progress_s': 20., 'observation_s': 2.}, stages=stages,
                participant_route=dict(map_sha256=sha256(Path(map_path).read_bytes()).hexdigest(),
                    mission_sha256=sha256(Path(mission_path).read_bytes()).hexdigest(),
                    checkpoint_waypoints=route.checkpoints_cleared,
                    length_m=route.length_metres, minimum_center_room_m=route.least_room_metres,
                    minimum_room_near=route.least_room_near,
                    policy_scope='route objectives + real G0 observation suggestions + D5/D0 selection; no Classic follower',
                    referee_credit='Only independent native referee crossings count; local arrival is not referee credit'))


def build(probe_path, output_dir, *, map_path, mission_path):
    probe_path, output_dir = Path(probe_path), Path(output_dir)
    map_path, mission_path = Path(map_path).resolve(), Path(mission_path).resolve()
    raw = probe_path.read_bytes()
    probe = json.loads(raw)
    profile = diagnostic_profile(probe, sha256(raw).hexdigest())
    aligned = ground_alignment(probe, sha256(raw).hexdigest())
    profile['alignment']['ned_origin_in_map_m'] = aligned['ned_origin_in_map_m']
    profile['alignment']['provenance'] = 'stationary-native-visual-ground-alignment'
    course_map = participant_geometry(map_path)
    profile['scope'] = SCOPE
    # The native odometry header/pose remains odom (north/left/up). Do not
    # relabel it as participant-map coordinates. Static geometry has its own
    # explicit transform below; route goals use the same native local frame.
    profile['strategy'] = json.loads((ROOT/'scenarios/strategies/hybrid_existing.json').read_text())
    now = json.loads(raw)['last_observations']['/clock']['sim_ns']
    profile['clock']['valid_until_ns'] = now + 600_000_000_000
    course = course_document(map_path, mission_path, profile['spatial']['reference'])
    profile['objective_m'] = course['stages'][0]['position_m']
    profile['diagnostic_assumptions'].append(
        'Experimental route map uses source-authored alignment; measured covariance is retained, not reduced to pass arrival.')
    output_dir.mkdir(parents=True, exist_ok=False)
    h1, mission, launcher = (output_dir/name for name in ('h1.json', 'course.json', 's0-experiment.json'))
    for path, data in ((h1, profile), (mission, course)):
        path.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    settings = assemble(h1, mission, launcher, mode='hybrid_shadow', profile_id='s0-course-ablation-'+sha256(raw).hexdigest()[:12])
    settings.update(experiment_scope=SCOPE, participant_map_file=str(map_path),
        participant_map_sha256=sha256(map_path.read_bytes()).hexdigest(), nominal_obstacle_reserve_m=.02,
        course_from_odom_yaw_rad=math.pi/2,
        course_from_odom_translation_m=[*map(float, course_map.launch_mark_centre_metres), .04],
        ground_alignment=aligned,
        runtime={'plan_hz': 5., 'semantic_hz': 2., 'budget_s': .05, 'lease_s': .25},
        experiment_assumptions={'qualification': 'not measured; simulator-only experiment',
            'route_geometry': 'participant static map, not fresh sensor FREE',
            'native_origin': 'source-authored NED origin; verify fresh preflight/reset alignment',
            'obstacle_reserve_m': '.02 is a nominal design assumption, not a braking bound',
            'g0': 'live observations only; log influence, fallback and no-event explicitly',
            'scans_enabled': False})
    launcher.write_text(json.dumps(settings, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return launcher


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map', type=Path, default=ROOT/'drone_code/team_nav/course_maps/phone_scan_2026-11-06.json')
    parser.add_argument('--mission', type=Path, default=ROOT/'drone_code/team_nav/missions/dronehacks_2026_side_gap_first.json')
    args = parser.parse_args()
    print(build(args.probe, args.output, map_path=args.map, mission_path=args.mission))


if __name__ == '__main__':
    main()
