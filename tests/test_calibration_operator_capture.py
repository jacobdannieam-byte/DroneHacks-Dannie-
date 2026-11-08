from datetime import datetime, timezone
import json
from types import SimpleNamespace

import pytest

from examples import calibration_operator_capture as helper


def test_dry_run_keeps_hash_bound_synthetic_artifacts(tmp_path):
    assert helper.dry_run(tmp_path)
    receipt = json.loads((tmp_path / 'operator-land.json').read_text())
    assert receipt['synthetic'] is True
    assert receipt['operator_preflight_sha256'] == helper.sha(tmp_path / 'operator-preflight.json')
    assert receipt['console_log_sha256'] == helper.sha(tmp_path / 'operator-console.log')
    assert 'no console process was started' in (tmp_path / 'operator-console.log').read_text()
    with pytest.raises(FileExistsError):
        helper.dry_run(tmp_path)


def test_capture_refuses_before_transport_and_keeps_failed_attempt(tmp_path):
    owner = dict(trial_id='r1', recovery_operator='reviewer')
    refused = tmp_path / 'refused'
    refused.mkdir()
    assert not helper.capture(refused, owner, lambda: None, lambda _: pytest.fail('transport called'), synthetic=True)
    assert json.loads((refused / 'operator-land.json').read_text())['result'] == 'refused_preflight'

    failed = tmp_path / 'failed'
    failed.mkdir()
    sample = dict(owner_exited=True, idle_motion_graph=True, fresh_healthy_in_lane=True,
                  status_px4_us=11, ros_ns=12_000, wall_ns=13_000)

    def timeout(log):
        log.write(b'partial console output')
        raise helper.subprocess.TimeoutExpired(helper.CONSOLE, 2.)

    assert not helper.capture(failed, owner, lambda: sample, timeout, synthetic=True)
    receipt = json.loads((failed / 'operator-land.json').read_text())
    assert receipt['exit_code'] == 2
    assert receipt['land_issue_ros_ns'] == sample['ros_ns']
    assert receipt['console_log_sha256'] == helper.sha(failed / 'operator-console.log')


@pytest.mark.parametrize('delay,ros,deadline', [(10_000_001, 100, 100_000_000), (2, 99, 100_000_000), (2, 100, 1002)])
def test_late_rewound_or_expired_issue_pair_never_calls_transport(tmp_path, delay, ros, deadline):
    sample = dict(ros_ns=100, wall_ns=1000)
    wall = iter((1000 + delay - 1, 1000 + delay))

    def guard(checked):
        return helper.issue_pair(checked, lambda: ros, lambda: next(wall), deadline_wall_ns=deadline)

    assert not helper.capture(tmp_path, {}, lambda: sample, lambda _: pytest.fail('transport called'),
                              synthetic=True, issue_check=guard)
    assert json.loads((tmp_path / 'operator-preflight.json').read_text())['refused_final_issue'] is True


def test_successful_issue_pair_records_resampled_values():
    wall = iter((1001, 1002))
    assert helper.issue_pair(dict(ros_ns=100, wall_ns=1000), lambda: 101, lambda: next(wall),
                            deadline_wall_ns=1003) == dict(ros_ns=101, wall_ns=1002, wall_sample_before_ns=1001)


def test_handoff_closes_at_thirty_seconds_before_any_native_setup(monkeypatch):
    expiry_ns = 5000
    fake_owner = dict(recovery_lease=dict(expiry_detected_wall_ns=expiry_ns))
    monkeypatch.setattr(helper, 'authorize', lambda _: (
        fake_owner, None, None, datetime(2099, 1, 1, tzinfo=timezone.utc)))
    monkeypatch.setattr(helper, 'monotonic_ns', lambda: expiry_ns + 30_000_000_000)
    monkeypatch.setattr(helper, 'sha', lambda _: 'fixture')
    monkeypatch.setattr(helper, 'verify_landing_speed', lambda: pytest.fail('native setup called'))
    evidence = helper.Path('/evidence')
    args = SimpleNamespace(output_dir=SimpleNamespace(resolve=lambda: evidence),
        owner=SimpleNamespace(resolve=lambda: evidence / 'calibration_owner.json'),
        authorization=SimpleNamespace(resolve=lambda: evidence / 'calibration_authorization.json'),
        approval=None)
    with pytest.raises(ValueError, match='lease or operator approval expired'):
        helper.execute_reviewed(args)


def test_execute_authorization_requires_isolation_before_any_live_setup(monkeypatch):
    monkeypatch.setattr(helper, 'isolated', lambda: False)
    with pytest.raises(ValueError, match='loopback-only'):
        helper.authorize(SimpleNamespace())


def test_operator_approval_must_bind_helper_hash_and_named_operator(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '55')
    monkeypatch.setattr(helper, 'isolated', lambda: True)
    profile, approval_path = tmp_path / 'profile.json', tmp_path / 'approval.json'
    profile.write_text('{}')
    approval_path.write_text('{}')
    identity = dict.fromkeys(helper.IDENTITY, 'fixture')
    identity.update(profile_sha256=helper.sha(profile), ros_domain_id=55)
    plan, mapping = SimpleNamespace(trial_kind='authority_expiry'), SimpleNamespace(fixture=True)
    monkeypatch.setattr(helper, 'load_profile', lambda _: (dict(geometry_evidence_sha256='fixture'), plan, mapping))
    owner = dict(identity, mode='calibration_trial', owner_id=helper.OWNER_ID, trial_kind='authority_expiry',
                 result='planned_authority_expiry', input_fault=None, events=[], recovery_operator='operator', trial_id='r1')
    owner_path = tmp_path / 'owner.json'
    owner_path.write_text(json.dumps(owner))
    authorization = tmp_path / 'authorization.json'
    authorization.write_text(json.dumps(dict(identity, authorized_artifact=True, trial_id='r1',
                                             approval_sha256=helper.sha(approval_path))))
    approval = dict(operator_console_land_authorized=True, operator_capture_sha256=helper.sha(helper.Path(helper.__file__)),
                    recovery_operator='operator', trial_id='r1')
    monkeypatch.setattr(helper, 'load_approval', lambda *a, **kw: (
        approval, datetime(2099, 1, 1, tzinfo=timezone.utc), dict(clock_mapping=vars(mapping))))
    args = SimpleNamespace(owner=owner_path, profile=profile, approval=approval_path, authorization=authorization,
                           recording_contract=tmp_path / 'contract.json', operator='operator')
    assert helper.authorize(args)[0] == owner
    args.operator = 'wrong'
    with pytest.raises(ValueError, match='named recovery operator'):
        helper.authorize(args)
    args.operator = 'operator'
    approval['operator_capture_sha256'] = '0' * 64
    with pytest.raises(ValueError, match='named recovery operator'):
        helper.authorize(args)
