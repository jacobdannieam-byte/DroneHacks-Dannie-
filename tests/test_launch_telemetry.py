from types import SimpleNamespace as S

from examples.h1_observation import ClockMapping
from refnav.core import Vehicle
from refnav.launch_telemetry import accept_attitude


def clock():
    return ClockMapping('px4', 'ros', 1000, 0, 1, 10**12, 'fixture')


def message(**changes):
    return S(**({'q': (1., 0., 0., 0.), 'quat_reset_counter': 0, 'timestamp_sample': 1000} | changes))


def test_source_capture_survives_duplicate_receipt_without_freshening():
    vehicle = Vehicle()
    assert accept_attitude(vehicle, message(), clock(), 2_000_000) is None
    assert vehicle.attitude_capture_ns == 1_001_000
    assert accept_attitude(vehicle, message(), clock(), 3_000_000) is None
    assert vehicle.attitude_capture_ns == 1_001_000
    assert vehicle.attitude_received_ns == 3_000_000


def test_rewound_conflicting_future_or_invalid_attitude_removes_admission():
    for bad in (message(timestamp_sample=999), message(q=(0., 1., 0., 0.)),
                message(quat_reset_counter=1), message(timestamp_sample=4000),
                message(q=(2., 0., 0., 0.)), message(quat_reset_counter=True)):
        vehicle = Vehicle()
        assert accept_attitude(vehicle, message(), clock(), 2_000_000) is None
        assert accept_attitude(vehicle, bad, clock(), 3_000_000) == 'launch_attitude_invalid'
        assert vehicle.q is None and vehicle.attitude_capture_ns is None


def test_new_sample_preserves_explicit_reset_for_guard_to_reject_epoch_change():
    vehicle = Vehicle()
    assert accept_attitude(vehicle, message(), clock(), 2_000_000) is None
    assert accept_attitude(vehicle, message(timestamp_sample=1001, quat_reset_counter=1), clock(), 2_000_000) is None
    assert vehicle.attitude_reset_counter == 1
