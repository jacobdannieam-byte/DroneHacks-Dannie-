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
