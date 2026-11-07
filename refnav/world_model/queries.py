"""Read-only semantic/conditional-affordance queries. No path planner or safety approval."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VehicleEnvelope:
    width_m: float
    height_m: float
    tracking_margin_m: float
    localization_margin_m: float

    def __post_init__(self):
        if not all(math.isfinite(v) and v >= 0 for v in vars(self).values()) or min(self.width_m, self.height_m) <= 0:
            raise ValueError('vehicle dimensions must be positive; margins finite and nonnegative')


def entities(state, now=None, semantic_type=None):
    return [e for e in state.snapshot(now)['entities'] if semantic_type is None or e['semantic_type'] == semantic_type]


def openings(state, now=None, envelope=None):
    result = []
    for entity in entities(state, now, 'opening'):
        g = entity['geometry']
        missing = list(entity['missing_evidence'])
        wide = tall = None
        if envelope is None:
            missing += ['vehicle_propeller_envelope', 'localization_margin', 'controller_tracking_margin']
        else:
            margin = envelope.tracking_margin_m + envelope.localization_margin_m + g['uncertainty_m']
            wide = g['width_m'] > envelope.width_m + 2 * margin
            tall = None if g['height_m'] is None else g['height_m'] > envelope.height_m + 2 * margin
        if entity['status'] in ('stale', 'uncertain'):
            missing.append('fresh_consistent_observation')
        if entity['status'] == 'invalidated':
            traversability = 'blocked'
            missing.append('unobstructed_reobservation')
        elif wide is False or tall is False:
            traversability = 'insufficient_clearance'
        else:
            traversability = 'unknown'
        # Even hypothetical complete dimensions do not certify a swept path.
        missing.append('validated_swept_path')
        result.append({**entity, 'semantic_type': 'candidate_passage', 'width_m': g['width_m'],
                       'height_m': g['height_m'], 'geometrically_wide_enough': wide,
                       'geometrically_tall_enough': tall, 'fully_observed_enough': False,
                       'localization_margin_configured': envelope is not None,
                       'tracking_margin_configured': envelope is not None, 'traversability': traversability,
                       'missing_evidence': sorted(set(missing))})
    return result
