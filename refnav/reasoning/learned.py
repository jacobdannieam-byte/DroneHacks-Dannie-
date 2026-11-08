"""Optional compact per-candidate MLP inference; no training truth enters this file.

Features are normalized physical/evidence quantities, never IDs or list positions.
NumPy is imported only when G2 is requested. Invalid checkpoints fail closed.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from .selection import components

FEATURE_VERSION = 2
FEATURE_COUNT = 16
HIDDEN_WIDTH = 16  # Our own small ablation size, 289 trainable parameters.
MAX_CHECKPOINT_BYTES = 100_000


def features(context, options):
    rows = []
    entities = {entity.entity_id: entity for entity in context.entities}
    for candidate in options:
        entity = entities[candidate.entity_id]
        rows.append((*components(candidate), min(1., entity.age_s / context.stale_after_s),
                     min(1., entity.observations / 8), float(entity.geometry.height_m is None),
                     min(1., entity.geometry.width_m / 4), min(1., len(entity.relations) / 4),
                     min(1., len(entity.evidence) / 8), float(bool(context.omitted_entities)),
                     *(max(-1., min(1., value / 6)) for value in candidate.relative_geometry_metres)))
    return np.asarray(rows, dtype=float).reshape((-1, FEATURE_COUNT))


class GoalMLP:
    def __init__(self, document, identity):
        if document.get('feature_version') != FEATURE_VERSION:
            raise ValueError('Unsupported goal feature schema; retrain the checkpoint.')
        self.identity = identity
        self.weights = {}
        shapes = {'w1': (FEATURE_COUNT, HIDDEN_WIDTH), 'b1': (HIDDEN_WIDTH,),
                  'w2': (HIDDEN_WIDTH,), 'b2': (), 'mean': (FEATURE_COUNT,), 'scale': (FEATURE_COUNT,)}
        for name, shape in shapes.items():
            value = np.asarray(document[name], dtype=float)
            if value.shape != shape or not np.isfinite(value).all() or np.max(np.abs(value)) > 100:
                raise ValueError('Invalid goal checkpoint weights; use a compatible finite checkpoint.')
            if name == 'scale' and np.min(value) < .001:
                raise ValueError('Invalid feature normalization scale.')
            self.weights[name] = np.frombuffer(value.tobytes(), dtype=float).reshape(shape)

    @classmethod
    def load(cls, path, expected_hash):
        if not path or not expected_hash:
            raise ValueError('G2 requires a checkpoint path and SHA-256 identity.')
        with Path(path).open('rb') as stream:
            payload = stream.read(MAX_CHECKPOINT_BYTES + 1)
        identity = hashlib.sha256(payload).hexdigest()
        if len(payload) > MAX_CHECKPOINT_BYTES or identity != expected_hash:
            raise ValueError('Goal checkpoint size or SHA-256 does not match configuration.')
        try:
            return cls(json.loads(payload), identity)
        except (KeyError, TypeError) as error:
            raise ValueError('Incomplete goal checkpoint; retrain using the supplied runner.') from error

    def score(self, context, options):
        weights = self.weights
        values = (features(context, options) - weights['mean']) / weights['scale']
        hidden = np.tanh(values @ weights['w1'] + weights['b1'])
        return tuple(map(float, hidden @ weights['w2'] + weights['b2']))
