"""Read-only planner contract for map corrections; disabled unless explicitly enabled.

Only route reasoning may use this pose. Collision evidence remains in its original
local frame. This consumer never publishes a target or changes an estimator.
"""
import math


def current_map_pose(fix, current_vio_pose, *, enabled=False, map_version,
                     reference_frame, reference_epoch, vio, now_ns, max_age_ns):
    if not enabled or fix is None or fix.schema != 1 or fix.status not in ('accepted', 'propagated'):
        return None
    if (fix.map_version != map_version or fix.reference_frame != reference_frame
            or fix.reference_epoch != reference_epoch or fix.vio != vio
            or fix.capture_ns is None or not 0 <= now_ns - fix.capture_ns <= max_age_ns
            or fix.reference_from_vio is None):
        return None
    errors = (fix.translation_uncertainty_m, fix.rotation_uncertainty_rad)
    if any(value is None or not math.isfinite(value) or value < 0 for value in errors):
        return None
    return fix.reference_from_vio.compose(current_vio_pose), errors

class ShadowMapConsumer:
    """H2 diagnostic adapter with a pinned reviewed map identity and epoch.

    Reports the current gravity-aligned body pose; roll/pitch are not in H2's
    motion state. The output never enters a command receipt or collision map.
    """
    def __init__(self, sidecar, *, map_version, reference_frame, reference_epoch,
                 vio, max_age_ns, enabled=False):
        self.sidecar = sidecar
        self.expected = dict(map_version=map_version, reference_frame=reference_frame,
            reference_epoch=reference_epoch, vio=vio, max_age_ns=max_age_ns, enabled=enabled)

    def __call__(self, state, alignment, now_ns):
        import numpy as np
        from .map_registration import SE3
        if not self.expected['enabled'] or alignment.map != self.expected['vio']:
            return None
        snapshot = self.sidecar.snapshot(now_ns)
        # H2 state is north/east/down; the spatial frame is east/north/up,
        # followed by the reviewed yaw and translation in FrameAlignment.
        angle = alignment.map_from_enu_yaw_rad
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array(((c,-s,0), (s,c,0), (0,0,1)))
        north, east, down = state.position_m
        position = rotation @ np.array((east,north,-down)) + alignment.ned_origin_in_map_m
        yaw = angle + math.pi / 2 - state.yaw_rad
        c, s = math.cos(yaw), math.sin(yaw)
        pose = SE3(((c,-s,0), (s,c,0), (0,0,1)), position)
        return current_map_pose(snapshot.localization, pose, now_ns=now_ns, **self.expected)
