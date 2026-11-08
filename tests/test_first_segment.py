"""Unknown, stale and obstructed depth cannot authorize a forward proposal."""
import math
import pytest
from refnav.first_segment import first_segment_metres

@pytest.mark.parametrize('distance,age,healthy,expected', [
    (None, .1, True, 0), (math.inf, .1, True, .7),
    (.5, .1, True, .2), (math.inf, .6, True, 0),
    (math.inf, -.1, True, 0), (math.inf, .1, False, 0),
    (math.nan, .1, True, 0), (.1, .1, True, 0)])
def test_proposals(distance, age, healthy, expected):
    assert first_segment_metres(distance, 1., 2., .3, capture_seconds=1.,
        now_seconds=1.+age, max_age_seconds=.5, tracking_healthy=healthy) == pytest.approx(expected)


def test_swept_body_rejects_jamb_unknown_floor_and_expired_evidence():
    from test_p2_spatial_evidence import volume_snapshot, REF, T
    from refnav.first_segment import swept_first_segment
    metric = volume_snapshot()
    args = dict(reference=REF, now_ns=T, horizontal_radius_metres=.16,
                half_height_metres=.1, margin_metres=.01)
    start, end = (-.375, .125, .125), (.375, .125, .125)
    assert swept_first_segment(metric, start, end, **args)[0]
    assert not swept_first_segment(metric, start, end, **(args | {'margin_metres': .23}))[0]
    assert not swept_first_segment(metric, start, (1.125,.125,.125), **args)[0]
    assert not swept_first_segment(metric, start, end, **(args | {'now_ns': T + metric.policy.free_age_ns + 1}))[0]
    assert not swept_first_segment(metric, start, (.375,.125,-.8), **args)[0]
    assert not swept_first_segment(metric, start, (3.,.125,.125), **args)[0]
