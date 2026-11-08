from dataclasses import replace
from datetime import datetime, timezone
from hashlib import sha256
import json
from pathlib import Path

import pytest

from examples.calibration_trial_owner import NativeInputs, PARAMETER_OVERRIDES, load_approval, load_profile
from examples.calibration_trial_probe import Fixture
from examples.gazebo_container import sha
from refnav.core import NAV_AUTO_LAND, NAV_OFFBOARD


def test_exact_trial_stages_and_ordered_receipts():
    fixture = Fixture()
    fixture.complete()
    assert [intent.command for intent in fixture.outputs if intent.command] == ['offboard', 'arm', 'land']
    brake, land = fixture.trial.events
    assert (brake['event'], land['event']) == ('brake_command', 'land_command')
    assert brake['stamp_ns'] < land['stamp_ns']
    assert brake['px4_issue_us'] == fixture.mapping.source_stamp_us(brake['stamp_ns'])
    assert fixture.step(nav=NAV_OFFBOARD, armed=True, landed=False) is None


def test_c1_profile_matches_recorder_and_settles_before_land():
    folder = Path(__file__).resolve().parents[1] / 'scenarios' / 'calibration'
    path = folder / 'c1_stop_020.json'
    _, plan, mapping = load_profile(path)
    contract = json.loads((folder / 'c1_recorder.json').read_text())
    # The isolated deployment reads immutable Git archive bytes (LF).
    assert contract['profile_sha256'] == sha256(path.read_bytes().replace(b'\r\n', b'\n')).hexdigest()
    assert contract['trial_kind'] == plan.trial_kind == 'stopping'
    assert contract['clock_mapping'] == vars(mapping)
    fixture = Fixture()
    fixture.trial.plan = plan
    fixture.reach_hover()
    assert fixture.trial.phase == 'climb'
    active = dict(nav=NAV_OFFBOARD, armed=True, landed=False)
    assert fixture.step(position=(0., 0., -1.), **active).velocity_ned_mps == (.2, 0., 0.)
    assert fixture.step(position=(1.49, 0., -1.), velocity=(.2, 0., 0.), **active).event is None
    assert fixture.step(position=(1.5, 0., -1.), velocity=(.2, 0., 0.), **active).event == 'brake_command'
    for _ in range(6):
        assert fixture.step(position=(1.6, 0., -1.), **active).command is None
    assert fixture.step(position=(1.6, 0., -1.), **active).command == 'land'


@pytest.mark.parametrize('faster_clock', ['ros_ns', 'wall_ns'])
def test_brake_timeout_uses_normal_landing_and_preserves_trial_failure(faster_clock):
    fixture = Fixture()
    fixture.trial.plan = replace(fixture.trial.plan, brake_timeout_s=4.)
    fixture.start_transit()
    active = dict(position=(1.5, 0., -1.), velocity=(.2, 0., 0.), nav=NAV_OFFBOARD, armed=True, landed=False)
    assert fixture.step(**active).event == 'brake_command'
    for _ in range(19):
        setattr(fixture, faster_clock, getattr(fixture, faster_clock) + 100_000_000)
        out = fixture.step(**active)
        assert out.command is None and out.event is None
    setattr(fixture, faster_clock, getattr(fixture, faster_clock) + 100_000_000)
    out = fixture.step(**active)
    assert out.command == 'land' and out.velocity_ned_mps is None
    fixture.step(position=(1.6, 0., -.5), nav=NAV_AUTO_LAND, armed=True, landed=False)
    assert fixture.step(position=(1.6, 0., 0.), nav=NAV_AUTO_LAND) is None
    assert fixture.trial.result == 'brake_settle_deadline'
    assert [event['event'] for event in fixture.trial.events] == ['brake_command', 'land_command']


def test_hover_land_has_no_horizontal_targets_and_no_braking_claim():
    fixture = Fixture('hover_land')
    fixture.reach_hover()
    assert fixture.outputs[-1].command == 'land'
    assert all(intent.velocity_ned_mps is None or intent.velocity_ned_mps[:2] == (0., 0.)
               for intent in fixture.outputs)
    fixture.step(position=(0., 0., -.5), nav=NAV_AUTO_LAND, armed=True, landed=False)
    fixture.step(nav=NAV_AUTO_LAND)
    assert fixture.trial.result == 'complete'
    assert [event['event'] for event in fixture.trial.events] == ['land_command']
    assert fixture.trial.events[0]['stamp_ns'] < fixture.trial.native_land_ack_ns < fixture.trial.native_disarm_ns


def test_landing_speed_and_deadline_fail_even_with_native_disarm():
    fixture = Fixture('hover_land')
    fixture.reach_hover()
    fixture.step(position=(0., 0., -.5), velocity=(0., 0., .6), nav=NAV_AUTO_LAND, armed=True, landed=False)
    fixture.step(nav=NAV_AUTO_LAND)
    assert fixture.trial.result == 'speed_limit_during_land'
    assert fixture.trial.max_observed_speed_mps == .6

    fixture = Fixture('hover_land')
    fixture.reach_hover()
    for _ in range(299):
        fixture.step(position=(0., 0., -.5), nav=NAV_AUTO_LAND, armed=True, landed=False)
    fixture.step(nav=NAV_AUTO_LAND)
    assert fixture.trial.result == 'landing_deadline'


@pytest.mark.parametrize('fault,result', [
    ({'admitted': False}, 'owner_admission_lost'),
    ({'nav': 0}, 'native_authority_lost'),
    ({'armed': False}, 'native_authority_lost'),
    ({'position': (4.01, 0., -1.)}, 'lane_exceeded'),
])
def test_loss_yields_without_publishing_and_never_replays(fault, result):
    fixture = Fixture()
    fixture.start_transit()
    state = dict(position=(.1, 0., -1.), nav=NAV_OFFBOARD, armed=True, landed=False)
    assert fixture.step(**(state | fault)) is None
    assert fixture.trial.result == result
    assert fixture.step(**state) is None
    assert not fixture.trial.events


def test_stale_clock_and_unconfirmed_publication_cannot_replay():
    fixture = Fixture()
    fixture.start_transit()
    fixture.wall_ns += 250_000_000
    assert fixture.step() is None
    assert fixture.trial.result == 'clock_stalled_or_rewound'
    fixture = Fixture()
    assert fixture.step(publish=False) is not None
    assert fixture.step() is None
    assert fixture.trial.result == 'publication_unconfirmed'


def test_overspeed_and_operator_land_use_only_owned_normal_land():
    for velocity, operator, reason in [((.4, .4, 0.), False, 'speed_limit'), ((0., 0., 0.), True, 'operator_land')]:
        fixture = Fixture()
        fixture.start_transit()
        if operator:
            fixture.trial.request_land()
        intent = fixture.step(position=(.1, 0., -1.), velocity=velocity, nav=NAV_OFFBOARD,
                              armed=True, landed=False)
        assert intent.command == 'land' and intent.velocity_ned_mps is None
        assert fixture.trial.recovery_reason == reason
        assert fixture.step(position=(.1, 0., -1.), nav=NAV_OFFBOARD, armed=False) is None
        assert fixture.trial.result == 'disarmed_before_landing_ack'


def test_invalid_trial_geometry_speed_and_timing_refused():
    plan = Fixture().trial.plan
    for change in (dict(north_brake_m=5.), dict(actual_speed_limit_mps=.51),
                   dict(trial_wall_limit_s=121.), dict(minimum_offset_m=(-.5, -.5, -.8))):
        with pytest.raises(ValueError):
            replace(plan, **change)


def test_native_clock_deferral_preserves_first_receipt_and_rejects_rewind():
    mapping = Fixture().mapping
    inputs = NativeInputs(mapping, 1.)
    inputs.offer('authority', 1_004_000, 1_004_000, (True, NAV_OFFBOARD), 1_000_000_000, 2_000_000_000)
    assert 'authority' not in inputs.samples
    inputs.advance(1_004_000_000, 2_006_000_000)
    assert inputs.samples['authority'] == (1_004_000_000, 2_000_000_000, (True, NAV_OFFBOARD))
    inputs.offer('authority', 1_004_000, 1_004_000, (False, 0), 1_008_000_000, 2_010_000_000)
    assert inputs.samples['authority'][2] == (True, NAV_OFFBOARD)
    inputs.offer('authority', 1_003_000, 1_003_000, (False, 0), 1_008_000_000, 2_011_000_000)
    assert inputs.fault == 'px4_source_rewind'


def test_supervised_release_pins_trial_contract_and_deployment(tmp_path):
    profile_sha = 'c' * 64
    identity = dict(profile_sha256=profile_sha, revision='d' * 40, world_sha256='e' * 64)
    contract = tmp_path / 'contract.json'
    contract.write_text(json.dumps(dict(review_status='reviewed', trial_id='trial', owner_id='/calibration_trial_owner',
        profile_sha256=profile_sha, frame_id='px4_local_ned', trial_kind='hover_land',
        topics={'events': '/qualification/events'})))
    approval = tmp_path / 'approval.json'
    now = datetime(2026, 1, 1, tzinfo=timezone.utc)
    fields = dict(identity, mode='calibration_trial', trial_id='trial', reviewed_by='reviewer',
                  recovery_operator='operator', isolation_reviewed=True, recovery_reviewed=True, recording_ready=True,
                  allow_failed_simulator_termination=True,
                  parameter_overrides=PARAMETER_OVERRIDES,
                  expires_utc='2026-01-01T01:00:00Z', recording_contract_sha256=sha(contract))
    approval.write_text(json.dumps(fields))
    assert load_approval(approval, identity, contract, now=now, trial_kind='hover_land')[0] == fields
    with pytest.raises(ValueError):
        load_approval(approval, dict(identity, world_sha256='f' * 64), contract, now=now, trial_kind='hover_land')
    with pytest.raises(ValueError):
        load_approval(approval, identity, contract, now=now.replace(hour=1), trial_kind='hover_land')
    with pytest.raises(ValueError):
        load_approval(approval, identity, contract, now=now, trial_kind='stopping')
    contract.write_text('{}')
    with pytest.raises(ValueError):
        load_approval(approval, identity, contract, now=now, trial_kind='hover_land')
