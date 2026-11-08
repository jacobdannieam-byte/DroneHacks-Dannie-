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
