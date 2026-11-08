"""Swept obstacle checks from the authored participant map and live occupied voxels."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import numpy as np

from onboard.adapters.geometry import R_ENU_NED, rot_x, rot_y, rot_z
from refnav.world_model.spatial_volume import Cell
from simulator.gazebo_trial.course_world_sdf import NET_HEIGHT_METRES, world_sdf


EXPERIMENT_EVIDENCE = 'nominal_participant_map_with_live_occupied_veto'
SWEEP_STEP_M = .02


def _samples(points):
    """Samples plus an outer bound covering the continuous line between them."""
    points = np.asarray(points, dtype=float)
    rows = [points[0]]
    for start, end in zip(points, points[1:]):
        count = max(1, math.ceil(math.dist(start, end) / SWEEP_STEP_M))
        if len(rows) + count > 1024:
            raise ValueError('experimental sweep exceeds the local query budget')
        rows.extend(start + (end - start) * (index / count) for index in range(1, count + 1))
    return np.asarray(rows)


@dataclass(frozen=True)
class AuthoredObstacles:
    low_xy: tuple
    high_xy: tuple
    boxes: tuple
    cylinders: tuple
    body_radius_m: float
    reserve_m: float
    course_from_odom_yaw_rad: float
    course_from_odom_translation_m: tuple

    @classmethod
    def from_course(cls, course, radius_m, reserve_m, *, course_from_odom_yaw_rad, course_from_odom_translation_m):
        if not .10 <= radius_m <= .60 or not .005 <= reserve_m <= .10:
            raise ValueError('finite explicit nominal vehicle and obstacle reserve required')
        # This deterministic conversion consumes only the participant map. The
        # running simulator and independent referee supply no planner geometry.
        parsed = ET.fromstring(world_sdf(course)).find('world')
        boxes, cylinders = [], []
        for model in parsed.findall('model'):
            if model.get('name') in ('floor', 'safety_net'):
                continue
            if model.findtext('pose') not in (None, '0 0 0 0 0 0'):
                raise ValueError('unsupported participant obstacle transform')
            for collision in model.findall('./link/collision'):
                link = next(link for link in model.findall('link') if collision in list(link))
                raw_pose = collision.findtext('pose') or link.findtext('pose') or '0 0 0 0 0 0'
                x, y, z, roll, pitch, yaw = map(float, raw_pose.split())
                box = collision.findtext('geometry/box/size')
                cylinder = collision.find('geometry/cylinder')
                if box:
                    boxes.append(((x, y, z), rot_z(yaw) @ rot_y(pitch) @ rot_x(roll),
                                  np.asarray(tuple(map(float, box.split()))) / 2))
                elif cylinder is not None and roll == pitch == 0.:
                    cylinders.append(((x, y, z), float(cylinder.findtext('radius')),
                                      float(cylinder.findtext('length')) / 2))
                else:
                    raise ValueError('unsupported authored obstacle shape')
        if not boxes or len(boxes) + len(cylinders) > 256:
            raise ValueError('expected bounded full-course obstacle geometry')
        return cls(tuple(course.lowest_corner_metres), tuple(course.highest_corner_metres),
                   tuple(boxes), tuple(cylinders), radius_m, reserve_m,
                   course_from_odom_yaw_rad, course_from_odom_translation_m)

    def sweep(self, metric, now_ns):
        return _CourseSweep(self, metric, now_ns)


class _CourseSweep:
    def __init__(self, course, metric, now_ns):
        self.course = course
        self.metric = metric
        self.resolution = metric.policy.resolution_m
        self.occupied_indices = np.argwhere(metric.cells_at(now_ns, metric.reference) == Cell.OCCUPIED)
        self.occupied = (self.occupied_indices + metric.origin_index + .5) * self.resolution

    def check(self, positions_ned, alignment, *, support_center_ned=None, witness=None):
        positions = np.asarray(positions_ned, dtype=float)
        if positions.ndim != 2 or positions.shape[1:] != (3,) or not len(positions) or not np.isfinite(positions).all():
            return 'invalid_experimental_sweep'
        transform = rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED
        odom_points = _samples(positions @ transform.T + alignment.ned_origin_in_map_m)
        points = (odom_points @ rot_z(self.course.course_from_odom_yaw_rad).T
                  + self.course.course_from_odom_translation_m)
        radius = self.course.body_radius_m + self.course.reserve_m + SWEEP_STEP_M / 2
        support = support_center_ned is not None
        if support:
            support_center = np.asarray(support_center_ned, dtype=float)
            if (support_center.shape != (3,) or not np.isfinite(support_center).all()
                    or np.any(np.linalg.norm(positions[:, :2] - support_center[:2], axis=1) > .15)):
                return 'experimental_support_outside_intended_column'
        if (np.any(points[:, :2] - radius <= self.course.low_xy)
                or np.any(points[:, :2] + radius >= self.course.high_xy)
                or np.any(points[:, 2] + radius >= NET_HEIGHT_METRES)
                or (not support and np.any(points[:, 2] - radius <= 0.))
                or (support and np.any(points[:, 2] < 0.))):
            return 'participant_enclosure_intersection'
        for center, rotation, half in self.course.boxes:
            delta = np.maximum(np.abs((points - center) @ rotation) - half, 0.)
            if np.any(np.einsum('ij,ij->i', delta, delta) <= radius ** 2):
                return 'participant_box_intersection'
        for center, cylinder_radius, half_height in self.course.cylinders:
            delta = points - center
            horizontal = np.maximum(np.linalg.norm(delta[:, :2], axis=1) - cylinder_radius, 0.)
            vertical = np.maximum(np.abs(delta[:, 2]) - half_height, 0.)
            if np.any(horizontal ** 2 + vertical ** 2 <= radius ** 2):
                return 'participant_cylinder_intersection'
        margin = radius + self.resolution / 2
        local_ids = np.flatnonzero(np.all((self.occupied >= odom_points.min(axis=0) - margin)
                                        & (self.occupied <= odom_points.max(axis=0) + margin), axis=1))
        local = self.occupied[local_ids]
        if support:
            # Only the floor-intersecting voxel layer in this intentional support
            # column may include expected floor returns. Voxel provenance cannot
            # distinguish another return in the same layer. All authored geometry
            # and every other live voxel still veto.
            floor_z = -self.course.course_from_odom_translation_m[2]
            center_odom = transform @ support_center + alignment.ned_origin_in_map_m
            floor_cell = np.abs(local[:, 2] - floor_z) <= self.resolution / 2
            near_pad = np.linalg.norm(local[:, :2] - center_odom[:2], axis=1) <= radius + self.resolution / math.sqrt(2)
            keep = ~(floor_cell & near_pad)
            local_ids, local = local_ids[keep], local[keep]
        if len(local):
            delta = np.maximum(np.abs(odom_points[:, None] - local[None, :]) - self.resolution / 2, 0.)
            squared = np.einsum('ijk,ijk->ij', delta, delta)
            if np.any(squared <= radius ** 2):
                if witness is not None:
                    distances = squared.min(axis=0)
                    hits = np.flatnonzero(distances <= radius ** 2)
                    nearest = hits[np.argsort(distances[hits], kind='stable')[:4]]
                    witness.update(metric_capture_ns=self.metric.capture_ns, metric_received_ns=self.metric.received_ns,
                                   metric_query_ns=self.metric.query_ns, observation_id=self.metric.observation_id,
                                   calibration_id=self.metric.calibration_id, map_frame=self.metric.reference.frame_id,
                                   map_epoch=self.metric.reference.epoch, clock_id=self.metric.reference.clock_id,
                                   resolution_m=self.resolution,
                                   sweep_start_ned=positions[0].tolist(), sweep_end_ned=positions[-1].tolist(),
                                   sweep_sample_count=len(odom_points), swept_radius_m=radius, support_column=support,
                                   intersecting_cell_count=len(hits), voxels=[dict(
                                       center_map_m=local[index].tolist(),
                                       capture_ns=int(self.metric._stamps[tuple(self.occupied_indices[local_ids[index]])]),
                                       minimum_sampled_aabb_distance_m=math.sqrt(distances[index])) for index in nearest])
                return 'live_occupied_voxel_intersection'
        return None
