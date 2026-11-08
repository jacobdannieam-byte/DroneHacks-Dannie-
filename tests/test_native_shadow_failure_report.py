import json
from types import SimpleNamespace

from examples import hybrid_launch, hybrid_native_shadow


def test_failed_native_observation_writes_json_and_returns_nonzero(tmp_path, monkeypatch):
    runtime = SimpleNamespace(records=[], record_count=0,
                              timings=SimpleNamespace(summary=lambda: {}), status=lambda: {})
    failure = {'reason': 'sim_clock_future_or_overload', 'cause': 'source_ahead_of_callback_limit',
               'source_publish_us': 151_000, 'source_sample_us': 149_000,
               'callback_ros_ns': 100_000_000, 'callback_wall_ns': 901_000_000,
               'mapped_capture_upper_ns': 151_000_000, 'last_delivered_clock_ns': 100_000_000}
    evidence = {'fault': failure['reason'], 'interrupted': False, 'command_graph': [],
                'observation_error': {'type': 'ValueError', 'reason': failure['reason']},
                'clock_ingestion': {'failure': failure}}
    monkeypatch.setattr(hybrid_launch, 'configured', lambda path: ('hybrid_shadow', runtime))
    monkeypatch.setattr(hybrid_native_shadow, 'run', lambda *args, **kwargs: evidence)
    path = tmp_path/'failed-native.json'
    assert hybrid_launch.main(['--config', 'fixture.json', '--output', str(path)]) == 1
    report = json.loads(path.read_text())
    assert report['observations']['clock_ingestion']['failure'] == failure
    assert report['observations']['observation_error']['type'] == 'ValueError'
