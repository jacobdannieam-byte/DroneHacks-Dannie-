from types import SimpleNamespace as Msg

import pytest

from examples.calibration_recovery_audit import audit
from examples.calibration_trial_probe import Fixture
from examples.h1_observation import RESET_FIELDS
from refnav.core import NAV_AUTO_LAND, NAV_OFFBOARD


def evidence():
    identity = dict(trial_kind='authority_expiry', trial_id='r1', owner_id='/calibration_trial_owner',
                    profile_sha256='a' * 64)
    last, expiry, land, disarm = 2_500_000, 2_750_000, 4_200_000, 5_500_000
    owner = dict(identity, result='planned_authority_expiry', events=[], input_fault=None, recovery_operator='operator',
        recovery_lease=dict(last_output_ros_ns=last * 1000, last_output_px4_us=last, last_output_wall_ns=last * 1000,
                            deadline_wall_ns=expiry * 1000, expiry_detected_ros_ns=expiry * 1000,
                            expiry_detected_wall_ns=expiry * 1000))
    operator = dict(trial_id='r1', operator='operator', command='px4-commander land', exit_code=0,
                    owner_exited=True, idle_motion_graph=True, fresh_healthy_in_lane=True, status_px4_us=4_000_000,
                    land_issue_ros_ns=land * 1000, land_issue_wall_ns=land * 1000)
    statuses = [Msg(timestamp=t, nav_state=nav, arming_state=armed)
                for t, nav, armed in ((1_000_000, 0, 1), (1_500_000, NAV_OFFBOARD, 2),
                    (2_000_000, NAV_OFFBOARD, 2), (2_500_000, NAV_OFFBOARD, 2), (3_000_000, NAV_OFFBOARD, 2),
                    (3_500_000, 2, 2), (4_000_000, 2, 2), (4_500_000, NAV_AUTO_LAND, 2),
                    (5_000_000, NAV_AUTO_LAND, 2), (disarm, NAV_AUTO_LAND, 1))]
    recording = dict(identity, provenance_scope='reviewed_calibration', recording_complete=True,
                     recording_faults=[], pending_clock_samples=0, clock_mapping=vars(Fixture().mapping),
                     status_reports=[dict(source_stamp_us=m.timestamp, nav_state=m.nav_state,
                                          arming_state=m.arming_state, receipt_wall_ns=m.timestamp * 1000)
                                     for m in statuses])
    raw = dict(commands=[Msg(command=176, timestamp=1_000_000, param1=1., param2=6.),
                         Msg(command=400, timestamp=1_100_000, param1=1., param2=0.)],
               targets=[Msg(timestamp=t, position=[float('nan')] * 3, velocity=(0., 0., 0.),
                            acceleration=[float('nan')] * 3, yaw=float('nan'), yawspeed=float('nan'))
                        for t in range(1_000_000, last + 1, 100_000)],
               modes=[Msg(timestamp=t, position=False, velocity=True) for t in range(1_000_000, last + 1, 100_000)],
               status=statuses, landed=[Msg(timestamp=disarm, landed=True)],
               positions=[Msg(timestamp_sample=t, x=0., y=0., z=-1. if t < land else 0., vx=0., vy=0., vz=0.,
                              xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
                              **dict.fromkeys(RESET_FIELDS, 0)) for t in range(1_100_000, disarm + 1, 100_000)])
    return owner, recording, operator, raw


def test_stationary_recovery_joins_raw_outputs_and_native_transitions():
    report = audit(*evidence(), position_mode=2)
    assert report['result'] == 'PASS' and report['review_status'] == 'unreviewed'
    assert report['position_fallback_px4_us'] == 3_500_000


@pytest.mark.parametrize('fault', ['expiry', 'replay', 'no_position', 'stale', 'force_disarm', 'speed', 'no_land',
                                   'attitude_mode', 'acceleration', 'unexpected_recovery_mode', 'synthetic'])
def test_stationary_recovery_refuses_unsupported_claims(fault):
    owner, recording, operator, raw = evidence()
    if fault == 'expiry':
        owner['recovery_lease']['expiry_detected_wall_ns'] -= 1
    if fault == 'replay':
        raw['targets'].append(raw['targets'][-1])
        raw['modes'].append(raw['modes'][-1])
    if fault == 'no_position':
        raw['status'][5].nav_state = NAV_AUTO_LAND
    if fault == 'stale':
        operator['land_issue_wall_ns'] += 4_000_000_000
    if fault == 'force_disarm':
        raw['commands'].append(Msg(command=400, timestamp=4_100_000, param1=0., param2=21196.))
    if fault == 'speed':
        raw['positions'][-2].vz = .51
    if fault == 'no_land':
        raw['landed'].clear()
    if fault == 'attitude_mode':
        raw['modes'][-1].attitude = True
    if fault == 'acceleration':
        raw['targets'][-1].acceleration[0] = .2
    if fault == 'unexpected_recovery_mode':
        raw['status'][-2].nav_state = 5
    if fault == 'synthetic':
        operator['synthetic'] = True
    with pytest.raises(ValueError):
        audit(owner, recording, operator, raw, position_mode=2)
