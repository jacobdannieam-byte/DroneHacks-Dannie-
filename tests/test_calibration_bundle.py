from pathlib import Path
import pytest
from examples.verify_calibration_bundle import verify


def test_exact_deployment_contracts_match():
    root = Path(__file__).resolve().parents[1]
    assert len(verify(lambda name: (root / name).read_bytes())) == 3


def test_profile_line_ending_change_is_not_silently_normalized():
    root = Path(__file__).resolve().parents[1]
    def read(name):
        data = (root / name).read_bytes()
        if name.endswith('c1_stop_020.json'):
            return data.replace(b'\n', b'\r\n')
        return data
    with pytest.raises(ValueError, match='profile/recorder'):
        verify(read)
