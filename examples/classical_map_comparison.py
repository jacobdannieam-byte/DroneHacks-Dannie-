"""Isolated, offline route/predictor ablation; never produces executable receipts.

Archived Classical route/follower and D5 candidates see the same participant map,
route goal and synthetic state. No simulator truth, ROS, registration or arming.
This is a component comparison, not a closed-loop policy qualification.
"""
import argparse
from dataclasses import asdict
from hashlib import sha256
import json
from pathlib import Path, PurePosixPath
import sys
import tarfile
from time import perf_counter

import numpy as np

from refnav.dynamics import DroneState, DynamicsModel, LocalNED, PositionSetpoint
from refnav.dynamics.residual import ResidualDynamicsModel, ResidualMLP
from refnav.predictive_planner import _generate
from refnav.trajectory_evaluation import FrameAlignment, MetricGoal
from refnav.world_model.spatial_input import SpatialReference
from examples.waypoint_look_preview import LookPreview


def digest(path):
    h = sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def verify_baseline(manifest):
    root = Path(manifest['artifact_root'])
    actual = {name: digest(root / name) for name in manifest['artifact_sha256']}
    if actual != manifest['artifact_sha256']:
        raise ValueError('Protected Classical evidence differs from its manifest')
    return actual


def copy_archived_package(archive, destination):
    """Copy regular participant files only, never extract arbitrary archive paths."""
    destination.mkdir(parents=True, exist_ok=False)
    with tarfile.open(archive) as source:
        for member in source:
            parts = PurePosixPath(member.name).parts
            if parts[:2] != ('drone_code', 'team_nav') or not member.isfile():
                continue
            if '..' in parts or any(':' in p or '\\' in p for p in parts):
                raise ValueError('Unsafe archived participant path')
            target = destination.joinpath(*parts[2:])
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('xb') as out, source.extractfile(member) as inp:
                out.write(inp.read())


def enu_ned(point):
    return (float(point[1]), float(point[0]), -float(point[2]))


def evaluate(model, state, schedule, goal, course_map):
    """Common diagnostic map check, deliberately separate from native D3 authority."""
    from team_nav.course_map import shapes_blocking_between_heights, distance_to_shapes_metres
    prediction_start = perf_counter()
    predictions = model.rollout(state, schedule, dt=.02)
    prediction_ms = (perf_counter() - prediction_start) * 1000
    points = np.asarray([enu_ned(state.position_m)] + [enu_ned(p.state.position_m) for p in predictions])
    # Same conservative vertical slab and horizontal body/margin as archived Classical.
    low, high = float(points[:, 2].min() - .20), float(points[:, 2].max() + .20)
    shapes = shapes_blocking_between_heights(course_map, low, high)
    samples = np.concatenate([np.linspace(a[:2], b[:2], max(2, int(np.ceil(np.linalg.norm(b[:2] - a[:2]) / .025)) + 1))
                              for a, b in zip(points, points[1:])])
    rooms, _ = distance_to_shapes_metres(samples, shapes)
    peak = max(np.linalg.norm(state.velocity_mps), *(np.linalg.norm(p.state.velocity_mps) for p in predictions))
    margin = float(min(rooms) - .35)
    reason = ('vertical_domain' if low < 0 or high > course_map.wall_top_height_metres else
              'participant_map_collision' if margin < 0 else 'predicted_speed_limit' if peak > .50 else 'diagnostic_eligible')
    return dict(reason=reason, predicted_peak_speed_mps=float(peak), sampled_map_margin_m=margin,
                terminal_error_m=float(np.linalg.norm(points[-1] - goal)),
                endpoint_enu_m=points[-1].tolist(), prediction_ms=prediction_ms)


def run(manifest_path, output, predictors):
    manifest = json.loads(manifest_path.read_text())
    before = verify_baseline(manifest)
    output.mkdir(parents=True, exist_ok=False)
    package = output / 'archived_participant'
    copy_archived_package(Path(manifest['artifact_root']) / 'source.tar', package)
    sys.path.insert(0, str(package.resolve()))
    from team_nav.course_map import load_course_map
    from team_nav.mission import load_mission
    from team_nav.route_planner import plan_route
    from team_nav.route_follower import RouteFollower
    map_file = package / 'course_maps/phone_scan_2026-11-06.json'
    mission_file = package / 'missions/dronehacks_2026_side_gap_first.json'
    for file, key in ((map_file, 'course_maps/phone_scan_2026-11-06.json'),
                      (mission_file, 'missions/dronehacks_2026_side_gap_first.json')):
        if digest(file) != manifest['archived_configuration_sha256']['drone_code/team_nav/' + key]:
            raise ValueError('Archived map/mission identity mismatch')
    course = load_course_map(map_file)
    route = plan_route(course, load_mission(mission_file))
    (output / 'route.json').write_text(json.dumps(asdict(route), default=lambda x: x.tolist(), indent=2))
    from team_nav.stopping_spots import find_stopping_spots
    from team_nav.clearance_grid import make_clearance_grid
    stops = find_stopping_spots(route.waypoints_metres)
    looks = []
    grids = []
    # Archived Classical's 5 cm height-slab grid. Store prior occupancy separately
    # from sensor observations; never feed these cells to H1 as measured FREE.
    for height in sorted(set(float(p[2]) for p in route.waypoints_metres)):
        grid = make_clearance_grid(course, height - .20, height + .20)
        name = f'prior-grid-{len(grids)}.npz'
        np.savez_compressed(output / name, occupied=grid.room_metres < .35,
                            room_m=grid.room_metres, origin_enu_m=grid.lowest_corner_metres)
        grids.append(dict(file=name, sha256=digest(output / name), height_m=height,
                          cell_size_m=.05, evidence='participant_map_prior', observed_free=False))
    for index, point in enumerate(route.waypoints_metres[:-1]):
        direction = route.waypoints_metres[index + 1] - point
        heading = float(np.arctan2(direction[0], direction[1]))
        stop_allowed = bool(stops[index]) and index not in {row[0] for row in route.stops_near_boulders}
        looks.append(dict(waypoint=index, enu_m=point.tolist(), stop_allowed=stop_allowed,
                          targets=LookPreview().targets(heading, stationary=True, clearance_valid=True,
                                  fresh_depth=True, stop_allowed=stop_allowed),
                          scope='preview only; booleans are synthetic prerequisites, not native evidence'))
    (output / 'look-previews.json').write_text(json.dumps(looks, indent=2))
    training = json.loads((predictors / 'comparison.json').read_text())['training']
    models = [('Hybrid D0', DynamicsModel(), None)]
    for seed in (11, 29, 47):
        file = predictors / f'physics_{seed}.npz'
        identity = digest(file)
        if identity != training[f'physics_{seed}']['sha256']:
            raise ValueError('Frozen PINN checkpoint identity mismatch')
        models.append((f'Hybrid PINN {seed}', ResidualDynamicsModel(network=ResidualMLP.load(file)), identity))
    ned = LocalNED('offline_course_ned', 0, 'offline')
    reference = SpatialReference(frame_id='offline_course_enu', epoch=0, clock_id='offline')
    alignment = FrameAlignment(ned, reference, 100_000_000, 0, 0., (0., 0., 0.), 0., 0., 'offline_exact_frame')
    rows = []
    for leg, (start, end) in enumerate(zip(route.waypoints_metres, route.waypoints_metres[1:])):
        direction = (end - start) / np.linalg.norm(end - start)
        for speed in (0., .20):
            state = DroneState(ned, .1, enu_ned(start), enu_ned(direction * speed), 0., 0.)
            follower = RouteFollower((start, end))
            target = follower.next_target(start, .1)
            classical = PositionSetpoint(ned, .1, 1.1, enu_ned(target), 0.)
            options = tuple(_generate(state, MetricGoal(reference, tuple(end)), alignment))
            for name, model, identity in [('Classical', DynamicsModel(), None), *models]:
                began = perf_counter()
                candidates = [('classical_follower', (classical,) * 50)] if name == 'Classical' else [(c.candidate_id, c.schedule) for c in options]
                diagnostics = [dict(candidate_id=key, **evaluate(model, state, schedule, end, course)) for key, schedule in candidates]
                eligible = [d for d in diagnostics if d['reason'] == 'diagnostic_eligible']
                selected = min(eligible, key=lambda d: d['terminal_error_m']) if eligible else None
                rows.append(dict(leg=leg, speed_mps=speed, variant=name, checkpoint_sha256=identity,
                                 state=asdict(state), goal_enu_m=end.tolist(), candidates=diagnostics,
                                 selected=selected, elapsed_ms=(perf_counter() - began) * 1000))
    after = verify_baseline(manifest)
    result = dict(scope='offline map-assisted candidate ablation; synthetic states; no closed-loop claims',
                  baseline_unchanged=before == after, baseline_revision=manifest['revision'],
                  baseline_manifest_sha256=digest(manifest_path), baseline_artifact_sha256=after,
                  participant_map_sha256=digest(map_file), mission_sha256=digest(mission_file),
                  registration_enabled=False, command_authority=False, native_ready=False,
                  prior_grids=grids, look_preview_sha256=digest(output / 'look-previews.json'),
                  score='minimum terminal goal error among diagnostic eligible candidates',
                  limitations=['Classical held target forecast uses D0 as common diagnostic model, not measured PX4.',
                               'D5 candidates and frozen predictors are real; this diagnostic scorer is not native D3.',
                               'Sampled Classical map margins exclude native uncertainty, reaction and stopping bounds.',
                               'No new observed FREE cells, no prior lease, no executable proposal receipts.',
                               'No closed-loop completion, contact, recovery or flight timing metrics.'],
                  rows=rows)
    (output / 'results.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    summary = {name: dict(cases=sum(r['variant'] == name for r in rows),
                         eligible=sum(r['variant'] == name and r['selected'] is not None for r in rows))
               for name in ('Classical', *(m[0] for m in models))}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, default=Path('artifacts/strategy_comparison/classical_baseline.json'))
    parser.add_argument('--predictors', type=Path, default=Path('artifacts/goal_comparison/predictors'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.baseline, args.output, args.predictors)
