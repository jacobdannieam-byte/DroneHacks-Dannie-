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
