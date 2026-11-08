import io
import tarfile

import pytest

from examples.classical_map_comparison import copy_archived_package, digest, enu_ned, verify_baseline
from examples.waypoint_look_preview import LookPreview
import math


def test_baseline_tamper_is_rejected(tmp_path):
    source = tmp_path / 'baseline.txt'
    source.write_text('protected evidence')
    manifest = dict(artifact_root=str(tmp_path), artifact_sha256={'baseline.txt': digest(source)})
    assert verify_baseline(manifest) == manifest['artifact_sha256']
    source.write_text('changed')
    with pytest.raises(ValueError, match='Protected'):
        verify_baseline(manifest)


def test_archive_copy_cannot_escape_or_overwrite(tmp_path):
    archive = tmp_path / 'source.tar'
    with tarfile.open(archive, 'w') as out:
        member = tarfile.TarInfo('drone_code/team_nav/../../../escape.py')
        member.size = 1
        out.addfile(member, io.BytesIO(b'x'))
    destination = tmp_path / 'copy'
    with pytest.raises(ValueError, match='Unsafe'):
        copy_archived_package(archive, destination)
    assert not (tmp_path / 'escape.py').exists()
    with pytest.raises(FileExistsError):
        copy_archived_package(archive, destination)


def test_frame_conversion_round_trip_and_known_axes():
    assert enu_ned((1, 2, 3)) == (2, 1, -3)
    assert enu_ned(enu_ned((1, 2, 3))) == (1, 2, 3)


@pytest.mark.parametrize('missing', ['stationary', 'clearance_valid', 'fresh_depth', 'stop_allowed'])
def test_look_refuses_missing_prerequisites(missing):
    flags = dict(stationary=True, clearance_valid=True, fresh_depth=True, stop_allowed=True)
    flags[missing] = False
    assert LookPreview().targets(0., **flags) == ()


def test_look_is_small_returns_to_heading_and_needs_new_observations():
    rows = LookPreview().targets(3., stationary=True, clearance_valid=True, fresh_depth=True, stop_allowed=True)
    assert rows[-1]['yaw_rad'] == pytest.approx(3.)
    assert max(abs(r['offset_rad']) for r in rows) == pytest.approx(math.pi / 9)
    assert rows[-1]['earliest_completion_s'] < 7.
    assert all(r['requires_new_captured_depth'] and not r['command_authority'] for r in rows)
    with pytest.raises(ValueError):
        LookPreview(offsets_rad=(math.tau, 0.))
