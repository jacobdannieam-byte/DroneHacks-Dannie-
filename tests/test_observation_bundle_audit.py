import sqlite3

import pytest

from examples.audit_observation_bundle import audit, digest
from simulator.gazebo_trial.recording_gate import bag_snapshot


def bundle(tmp_path):
    bag = tmp_path / 'observations'
    bag.mkdir()
    db_path = bag / 'observations_0.db3'
    topics = ['/clock', '/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_local_position']
    with sqlite3.connect(db_path) as db:
        db.execute('create table topics(id integer, name text)')
        db.execute('create table messages(id integer, topic_id integer, timestamp integer)')
        for index, topic in enumerate(topics):
            db.execute('insert into topics values (?,?)', (index, topic))
            db.executemany('insert into messages values (?,?,?)',
                           [(index*2, index, 1), (index*2+1, index, 30_000_000_001)])
    (bag / 'metadata.yaml').write_text('synthetic fixture, not native evidence')
    manifest = {'raw_files': {str(p): {'bytes': p.stat().st_size, 'sha256': digest(p)}
                              for p in bag.iterdir()}}
    report = bag_snapshot(bag, topics, finalized=True, minimum_duration_ns=30_000_000_000)
    return manifest, report, bag


def test_integrity_only(tmp_path):
    result = audit(*bundle(tmp_path))
    assert result['byte_and_sqlite_audit'] == 'PASS'
    assert 'no decoded telemetry' in result['qualification']


def test_modified_raw_bytes_rejected(tmp_path):
    manifest, report, bag = bundle(tmp_path)
    (bag / 'metadata.yaml').write_text('replaced')
    with pytest.raises(ValueError, match='mismatch'):
        audit(manifest, report, bag)


def test_forged_summary_rejected(tmp_path):
    manifest, report, bag = bundle(tmp_path)
    report['topics']['/clock']['count'] += 1
    with pytest.raises(ValueError, match='disagree'):
        audit(manifest, report, bag)


def test_additional_unexamined_segment_rejected(tmp_path):
    manifest, report, bag = bundle(tmp_path)
    (bag / 'observations_1.db3').write_bytes(b'not examined')
    with pytest.raises(ValueError, match='unsplit'):
        audit(manifest, report, bag)


def test_unmanifested_database_rejected(tmp_path):
    manifest, report, bag = bundle(tmp_path)
    del manifest['raw_files'][str(bag / 'observations_0.db3')]
    with pytest.raises(ValueError, match='absent from manifest'):
        audit(manifest, report, bag)


@pytest.mark.parametrize('suffix', ['_1.db3', '_0.db3-wal'])
def test_recording_files_created_during_audit_rejected(tmp_path, monkeypatch, suffix):
    manifest, report, bag = bundle(tmp_path)
    def changed_snapshot(*args, **kwargs):
        result = bag_snapshot(*args, **kwargs)
        (bag / ('observations' + suffix)).write_bytes(b'new concurrent data')
        return result
    monkeypatch.setattr('examples.audit_observation_bundle.bag_snapshot', changed_snapshot)
    with pytest.raises(ValueError, match='file set changed'):
        audit(manifest, report, bag)
