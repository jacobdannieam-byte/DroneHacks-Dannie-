"""Read-only first-segment distance proposal from fresh depth evidence.

Adapted from soham-stages free_distance_seen: infinity is bounded by the judged
cross-section. The existing mission owner must independently validate any proposal.
"""
import math


def first_segment_metres(distance_metres, judged_out_to_metres, requested_metres,
                         stopping_distance_metres, *, capture_seconds, now_seconds,
                         max_age_seconds, tracking_healthy):
    values = (judged_out_to_metres, requested_metres, stopping_distance_metres,
              capture_seconds, now_seconds, max_age_seconds)
    if not tracking_healthy or distance_metres is None:
        return 0.0
    if not all(math.isfinite(value) and value >= 0 for value in values):
        return 0.0
    if not 0 <= now_seconds - capture_seconds <= max_age_seconds:
        return 0.0
    if math.isnan(distance_metres) or distance_metres < 0:
        return 0.0
    free_metres = min(distance_metres, judged_out_to_metres)
    return min(requested_metres, max(0.0, free_metres - stopping_distance_metres))


def swept_first_segment(metric, start_metres, end_metres, *, reference, now_ns,
                        horizontal_radius_metres, half_height_metres, margin_metres):
    """Validate the whole segment with an enclosing sphere, including both ends.

    The sphere encloses a cylindrical body at every attitude. This deliberately
    overestimates clearance needs; a clear centre ray alone cannot pass it.
    Unknown, expired, outside-map and wrong-epoch evidence all withhold approval.
    """
    bounds = (horizontal_radius_metres, half_height_metres)
    if not all(math.isfinite(value) and value > 0 for value in bounds):
        raise ValueError('Measured positive vehicle radius and half-height are required.')
    radius = math.hypot(*bounds)
    result = metric.check_sweep((start_metres, end_metres), reference=reference,
        now_ns=now_ns, vehicle_radius_m=radius, margin_m=margin_metres)
    return result.status == 'verified_free', result
