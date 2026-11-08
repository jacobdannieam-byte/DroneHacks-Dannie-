"""Read-only CDR reconstruction of likely native launch obstacle vetoes."""

import argparse
from dataclasses import replace
import glob
import json
import math
from pathlib import Path
import sqlite3
import struct
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2] / 'policy-gates-integration'
sys.path.insert(0, str(ROOT))

from examples.s0_course_experiment import load_experiment
from examples.s0_experiment_geometry import SWEEP_STEP_M, _samples
from onboard.adapters.geometry import R_ENU_NED, rot_z
from refnav.world_model.spatial_input import PoseBuffer, PoseSample, decode_rectified_depth
from refnav.world_model.spatial_volume import Cell, RollingVolume


class CDR:
    def __init__(self, data):
        if data[:4] != b'\x00\x01\x00\x00':
            raise ValueError('expected little-endian CDR')
        self.data, self.offset = data, 4

    def read(self, fmt, align):
        self.offset += (-(self.offset - 4)) % align
        result = struct.unpack_from('<' + fmt, self.data, self.offset)
        self.offset += struct.calcsize('<' + fmt)
        return result

    def string(self):
        size, = self.read('I', 4)
        start = self.offset
        self.offset += size
        return self.data[start:self.offset - 1].decode()

    def header(self):
        sec, nsec = self.read('iI', 4)
        return sec * 10**9 + nsec, self.string()


def rows(db, topic, start, end):
    return db.execute('''select messages.timestamp, messages.data from messages join topics
        on messages.topic_id=topics.id where topics.name=? and messages.timestamp between ? and ?
        order by messages.timestamp, messages.id''', (topic, start, end)).fetchall()


def depth_support(center, image, calibration, policy, path, floor_offset):
    """Reconstruct qualifying axial returns from the exact classifier tile ROI."""
    _, depth, pose = image
    rotation = pose.rotation @ np.asarray(calibration.R_body_optical)
    body_offset = np.asarray(calibration.t_body_optical_m)
    origin = np.asarray(pose.position_m) + pose.rotation @ body_offset
    camera = (center - origin) @ rotation
    distance = math.dist(center, pose.position_m) + math.sqrt(3) * policy.resolution_m / 2
    error = calibration.depth_error_m
    uncertainty = (pose.position_error_m + 2 * math.sin(pose.angle_error_rad / 2) * distance
                   + (calibration.extrinsic_position_error_m or 0.)
                   + 2 * math.sin((calibration.extrinsic_angle_error_rad or 0.) / 2)
                   * (distance + math.hypot(*body_offset)))
    half = (policy.resolution_m / 2 + uncertainty) * np.abs(rotation).sum(axis=0)
    lower, upper = camera - half, camera + half
    near, far = lower[2], upper[2]
    fx, fy, cx, cy = calibration.K
    projected = np.asarray([p[:2] / z for p in (lower, upper) for z in (near, far)]) * (fx, fy) + (cx, cy)
    limits = (depth.shape[1] - 1, depth.shape[0] - 1)
    low = np.floor(np.clip(projected.min(axis=0) - .5, 0, limits)).astype(int) // 8 * 8
    high = np.floor(np.clip(projected.max(axis=0) + 1.5, 0, limits)).astype(int) // 8 * 8 + 8
    ys, xs = np.mgrid[low[1]:min(high[1], depth.shape[0]), low[0]:min(high[0], depth.shape[1])]
    ranges = depth[ys, xs]
    qualified = (np.isfinite(ranges) & (ranges > policy.min_depth_m) & (ranges < policy.max_depth_m)
                 & (ranges - error <= far) & (ranges + error >= near))
    z = ranges[qualified]
    if not len(z):
        return {'pixels_overlapping_axial_interval': 0}
    rays = np.stack(((xs[qualified] - cx) / fx * z, (ys[qualified] - cy) / fy * z, z), axis=1)
    returns = rays @ rotation.T + origin
    floor_z = returns[:, 2] + floor_offset
    within_voxel = np.all(abs(returns - center) <= policy.resolution_m / 2, axis=1)
    direction = path[1] - path[0]
    delta = returns - path[0]
    fraction = np.clip(delta @ direction / (direction @ direction), 0., 1.)
    distance = np.linalg.norm(delta - fraction[:, None] * direction, axis=1)
    return dict(pixels_overlapping_axial_interval=len(z), endpoints_in_nominal_voxel=int(within_voxel.sum()),
                endpoint_height_above_authored_floor_m=[float(floor_z.min()), float(floor_z.max())],
                closest_endpoint_to_centerline_m=float(distance.min()),
                pose_inflation_at_cell_m=float(uncertainty))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resolution', type=float)
    args = parser.parse_args()
    folder = Path(__file__).resolve().parent
    route = ROOT / 'drone_code/team_nav/course_maps/phone_scan_2026-11-06.json'
    runtime = load_experiment(folder / 's0-profile/s0-experiment.json', route)
    replay = runtime.loop.replay
    spatial, alignment = replay.hybrid.spatial, replay.hybrid.alignment
    policy = replay.world._policy
    if args.resolution is not None:
        policy = replace(policy, resolution_m=args.resolution)
    transform = rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED
    course_rotation = rot_z(runtime.geometry.course_from_odom_yaw_rad)
    course_origin = runtime.geometry.course_from_odom_translation_m
    end = 61_208_000_000
    start = end - policy.occupied_age_ns - 500_000_000
    path, = glob.glob(str(folder / 'observations/*.db3'))
    db = sqlite3.connect(Path(path).resolve().as_uri() + '?mode=ro&immutable=1', uri=True)
    poses = PoseBuffer(80_000_000, 3., 4.)
    for receipt, raw in rows(db, '/gazebo_hybrid/simulated_estimator/odometry', start - 200_000_000, end):
        reader = CDR(raw)
        stamp, frame = reader.header()
        body = reader.string()
        assert frame == spatial.reference.frame_id and body == spatial.reference.body_frame
        x, y, z, qx, qy, qz, qw = reader.read('7d', 8)
        if stamp <= receipt:
            poses.push(PoseSample(spatial.reference, stamp, receipt, True, (x, y, z), (qw, qx, qy, qz),
                                  spatial.position_error_m, spatial.angle_error_rad))
    volume = RollingVolume(policy)
    commands = json.loads((folder / 'experiment.json').read_text())
    target = np.asarray(commands['last_issue']['setpoint'][:3])
    home = (target[0], target[1], target[2] + .76)
    raw_state = []
    for receipt, raw in rows(db, '/fmu/out/vehicle_odometry', end - 240_000_000, end):
        _, stamp, frame, x, y, z = struct.unpack_from('<QQB3x3f', raw, 4)
        assert frame == 1
        raw_state.append((receipt, stamp * 1000, (x, y, z)))
    image_frames = {}
    checks = {}
    first_hit_captures = {}
    reach_radius = runtime.geometry.body_radius_m + runtime.geometry.reserve_m + SWEEP_STEP_M / 2
    for received, raw in rows(db, '/simulator_only/oak/depth/image', start, end):
        reader = CDR(raw)
        capture, frame = reader.header()
        height, width = reader.read('II', 4)
        encoding = reader.string()
        bigendian, = reader.read('B', 1)
        step, size = reader.read('II', 4)
        assert frame == spatial.calibration.camera_frame
        decoded = decode_rectified_depth(reader.data[reader.offset:reader.offset + size], encoding=encoding,
                                         width=width, height=height, step=step, bigendian=bool(bigendian))
        pose = poses.at(capture, end)
        if pose is None or not 0 <= received - capture <= 250_000_000:
            continue
        volume.apply(volume.prepare(decoded, spatial.calibration, pose), capture)
        image_frames[capture] = (received, decoded, pose)
        for check in (61_108_000_000, 61_156_000_000, 61_208_000_000):
            if received > check or any(poses.at(stamp, check) is None for stamp in pose.bracket_ns):
                continue
            position = next(pos for receipt, _, pos in reversed(raw_state) if receipt <= check)
            odom = _samples(np.asarray((position, target)) @ transform.T + alignment.ned_origin_in_map_m)
            snap = volume.snapshot(spatial.reference, check, capture, str(capture), spatial.calibration.calibration_id,
                                   True, received_ns=received)
            reason = runtime.geometry.sweep(snap, check).check((position, target), alignment, support_center_ned=home)
            row = dict(check_ns=check, latest_frame_ns=capture, native_position_ned=position,
                       map_path=[odom[0].tolist(), odom[-1].tolist()], refusal=reason)
            checks[check] = row
            if reason != 'live_occupied_voxel_intersection':
                continue
            first_hit_captures.setdefault(check, capture)
            indices = np.argwhere(snap.cells_at(check, snap.reference) == Cell.OCCUPIED)
            centers = (indices + snap.origin_index + .5) * policy.resolution_m
            distances = np.linalg.norm(np.maximum(np.abs(odom[:, None] - centers[None, :]) - policy.resolution_m / 2,
                                                 0.), axis=2).min(axis=0)
            exact = (abs(centers[:, 2] + course_origin[2]) > policy.resolution_m / 2
                     ) | (np.linalg.norm(centers[:, :2] - (transform @ home + alignment.ned_origin_in_map_m)[:2], axis=1)
                          > reach_radius + policy.resolution_m / math.sqrt(2))
            hits = []
            for cell in np.flatnonzero(exact & (distances <= reach_radius)):
                hit = centers[cell]
                hits.append(dict(odom_center=hit.tolist(), course_center=(course_rotation @ hit + course_origin).tolist(),
                                 supporting_capture_ns=int(snap._stamps[tuple(indices[cell])]),
                                 distance_m=float(distances[cell])))
            row['hits'] = hits
    for check, row in checks.items():
        for hit in row.get('hits', ()):
            hit['raw_ray_support'] = depth_support(np.asarray(hit['odom_center']),
                image_frames[hit['supporting_capture_ns']], spatial.calibration, policy,
                np.asarray(row['map_path']), course_origin[2])
    print(json.dumps(dict(scope='read_only_reconstruction_not_exact_owner_callback_replay',
                          resolution_m=policy.resolution_m, frames=sorted(image_frames),
                          body_sweep_radius_m=reach_radius, target_ned=target.tolist(),
                          first_hit_capture_by_check_ns=first_hit_captures, checks=list(checks.values())), indent=2))


if __name__ == '__main__':
    main()
