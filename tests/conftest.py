"""Repository import path and isolated semantic reasoning fixtures; no ROS setup."""
import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.fixture(scope='session')
def reasoning_replay():
    from examples.semantic_world_model_demo import replay
    return replay()[1]


@pytest.fixture
def reasoning_snapshot(reasoning_replay):
    return copy.deepcopy(reasoning_replay[1]['snapshot'])


@pytest.fixture
def reasoning_context(reasoning_snapshot):
    from refnav.reasoning import WorldReference, build_context
    return build_context(reasoning_snapshot, now=.2, reference=WorldReference())
