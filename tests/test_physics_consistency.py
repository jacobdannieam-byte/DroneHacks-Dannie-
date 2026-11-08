"""The optional response-law loss changes training without changing model size."""
import numpy as np
import pytest

from examples.train_adaptive_dynamics_d1 import fit, _forward
from refnav.dynamics.residual import SHAPES


def test_physics_loss_shrinks_unsupported_response_correction():
    sample = (np.zeros((8, 17), dtype=np.float32), np.ones((8, 4), dtype=np.float32),
              np.full((8, 4), .5, dtype=np.float32))
    plain, _, _ = fit(sample, sample, seed=11)
    physics, _, _ = fit(sample, sample, seed=11, physics_strength=.5)
    for name, shape in SHAPES.items():
        assert plain[name].shape == physics[name].shape == shape
    x = np.zeros((8, 17), dtype=np.float32)
    assert np.linalg.norm(_forward(x, physics)[-1]) < np.linalg.norm(_forward(x, plain)[-1])
    with pytest.raises(ValueError):
        fit(sample, sample, physics_strength=float('nan'))
