"""Validated Hybrid experiment selection; Classical remains an external system.

Execution authority is deliberately not a strategy parameter. Existing owner
and gateway admission checks still apply to every resulting assessment.
"""
from dataclasses import asdict, dataclass
from hashlib import sha256
import json
from pathlib import Path
import re

from refnav.reasoning.selection import ReasonerConfig

VARIANTS = {
    'hybrid_existing': 'G0',
    'hybrid_information_gain': 'G1',
    'hybrid_learned_ranker': 'G2',
    'hybrid_physics_informed': 'G1',
}


@dataclass(frozen=True)
class StrategyConfig:
    identifier: str = 'hybrid_existing'
    ranker_checkpoint: str | None = None
    ranker_sha256: str | None = None
    predictor_checkpoint: str | None = None
    predictor_sha256: str | None = None
    schema_version: int = 1

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version != 1 or self.identifier not in VARIANTS:
            raise ValueError('Unknown Hybrid strategy or schema version')
        for path, digest, required in (
            (self.ranker_checkpoint, self.ranker_sha256, self.identifier == 'hybrid_learned_ranker'),
            (self.predictor_checkpoint, self.predictor_sha256, self.identifier == 'hybrid_physics_informed'),
        ):
            if required:
                if not isinstance(path, str) or not path or not isinstance(digest, str) or not re.fullmatch('[0-9a-f]{64}', digest):
                    raise ValueError('Experimental models require a path and SHA256')
            elif path is not None or digest is not None:
                raise ValueError('Model configuration does not belong to this strategy')

    @property
    def identity(self):
        return sha256(json.dumps(asdict(self), sort_keys=True, separators=(',', ':')).encode()).hexdigest()

    def reasoner(self):
        return ReasonerConfig(VARIANTS[self.identifier], self.ranker_checkpoint, self.ranker_sha256)

    def load_predictor(self):
        """Load once, before processing sensor inputs; return explicit D0 fallback."""
        if self.identifier != 'hybrid_physics_informed':
            return None, None, None
        try:
            from refnav.dynamics.residual import ResidualDynamicsModel, ResidualMLP, MAX_ARCHIVE_BYTES
            path = Path(self.predictor_checkpoint)
            if path.stat().st_size > MAX_ARCHIVE_BYTES:
                raise ValueError('checkpoint exceeds byte budget')
            if sha256(path.read_bytes()).hexdigest() != self.predictor_sha256:
                raise ValueError('checkpoint hash mismatch')
            model = ResidualDynamicsModel(network=ResidualMLP.load(path))
            return model, 'physics:' + self.predictor_sha256, None
        except (OSError, ValueError, TypeError, ImportError) as error:
            return None, None, 'predictor_load_fallback:' + type(error).__name__
