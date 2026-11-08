"""Planner contract uses current odometry and rejects stale or mismatched fixes."""
from types import SimpleNamespace
import numpy as np
from refnav.world_model.map_registration import SE3
from refnav.world_model.planner_map_input import current_map_pose


def test_current_pose_and_unavailable_contract():
    fix = SimpleNamespace(schema=1, status='accepted', map_version='map', reference_frame='course',
        reference_epoch=2, vio='vio2', capture_ns=100,
        reference_from_vio=SE3(np.eye(3), (1, 0, 0)),
        translation_uncertainty_m=.1, rotation_uncertainty_rad=.02)
    current = SE3(np.eye(3), (5, 0, 0))
    args = dict(map_version='map', reference_frame='course', reference_epoch=2,
                vio='vio2', now_ns=110, max_age_ns=20)
    assert current_map_pose(fix, current, **args) is None
    result = current_map_pose(fix, current, enabled=True, **args)
    assert result[0].translation_m == (6., 0., 0.)
    for key, value in [('map_version', 'wrong'), ('reference_frame', 'wrong'),
                       ('reference_epoch', 3), ('vio', 'vio3'), ('now_ns', 99), ('now_ns', 121)]:
        assert current_map_pose(fix, current, enabled=True, **(args | {key: value})) is None
    fix.translation_uncertainty_m = float('nan')
    assert current_map_pose(fix, current, enabled=True, **args) is None
