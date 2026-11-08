"""Explicit per-stream owner deadlines; simulation cadence is not hardware proof."""
from dataclasses import dataclass

@dataclass(frozen=True)
class OwnerTiming:
    authority_ns: int = 300_000_000
    landed_ns: int = 300_000_000
    position_ns: int = 300_000_000

    def limit(self, key):
        return {'authority': self.authority_ns, 'landed': self.landed_ns,
                'position': self.position_ns}[key]

    def fresh(self, key, stamp, now, error=0):
        return 0 <= now-stamp and now-stamp+error <= self.limit(key)

    @classmethod
    def for_runtime(cls, runtime):
        if runtime.adapter.simulated_estimator:
            # Match independently observed 2 Hz status and 1 Hz landed cadence.
            # Slow authority detection MUST be included in the stopping budget.
            return cls(authority_ns=750_000_000, landed_ns=1_500_000_000)
        return cls()

    def validate_stopping_budget(self, envelope):
        if envelope is None or envelope.reaction_s < self.authority_ns / 1e9:
            raise ValueError('stopping reaction budget excludes owner authority detection deadline')
