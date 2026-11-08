from copy import deepcopy
import json

import pytest

from examples.build_disarmed_shadow_profiles import build, diagnostic_profile
from examples.hybrid_launch import configured


def probe():
    return {'tracking_source': 'simulated_estimator', 'disarmed': True, 'landed': True,
            'healthy': True, 'advancing_clock': True, 'missing': [], 'graph': {'issues': []},
            'px4_clock_samples': [{'position_variance': [.001, .001, .03]}],
            'last_observations': {
                '/clock': {'sim_ns': 20_000_000_000},
                '/simulator_only/oak/depth/camera_info': {'frame': 'oak_depth_optical_frame',
                    'width': 512, 'height': 320, 'k': [317., 0., 256., 0., 317., 160., 0., 0., 1.],
                    'p': [317., 0., 256., 0., 0., 317., 160., 0., 0., 0., 1., 0.], 'd': [0.]*5},
                '/tf_static': {'evidence': [{'frame': 'base_link', 'child_frame': 'oak_depth_optical_frame',
                    'valid': True, 'translation_m': [.131, 0., 0.], 'rotation_xyzw': [.5,-.5,.5,-.5]}]},
                '/fmu/out/vehicle_odometry': {'evidence': [{'reset': [6]}]},
                '/fmu/out/vehicle_local_position': {'evidence': [{'reset': [2,1,1,1,1]}]},
                '/fmu/in/vehicle_visual_odometry': {'evidence': [{'reset': [0]}]}}}


@pytest.mark.parametrize('field', ['disarmed', 'landed', 'healthy', 'advancing_clock'])
def test_builder_refuses_invalid_preflight(field):
    data = probe()
    data[field] = False
    with pytest.raises(ValueError):
        diagnostic_profile(data, 'a'*64)


def test_builder_refuses_missing_graph_changed_rig_and_nonfinite_covariance():
    data = probe()
    data.pop('graph')
    with pytest.raises(ValueError):
        diagnostic_profile(data, 'a'*64)
    data = probe()
    data['last_observations']['/tf_static']['evidence'][0]['translation_m'][0] = .5
    with pytest.raises(ValueError):
        diagnostic_profile(data, 'a'*64)
    data = probe()
    data['px4_clock_samples'][0]['position_variance'].append(float('nan'))
    with pytest.raises(ValueError):
        diagnostic_profile(data, 'a'*64)


def test_generated_launchers_are_shadow_only_without_landing_approval(tmp_path):
    source = tmp_path/'probe.json'
    source.write_text(json.dumps(probe()))
    outputs = build(source, tmp_path/'profiles')
    assert len(outputs) == 4
    from pathlib import Path
    for output in outputs:
        mode, runtime = configured(output)
        assert mode == 'hybrid_shadow'
        assert runtime.course.stages[-1].landing_site_review is None
        assert not runtime.executor.gateway.enabled
        assert '/simulator_only/oak/depth/image' in runtime.topics
        runtime.close()
        settings = json.loads(Path(output).read_text())
        settings['strategy'] = 'hybrid_sitl'
        Path(output).write_text(json.dumps(settings))
        with pytest.raises(ValueError, match='landing site'):
            configured(output)
