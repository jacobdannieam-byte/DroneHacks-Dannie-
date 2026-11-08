"""Synthetic inspection episodes for offline comparison, never imported by runtime.

Hidden occluders and measurement outcomes belong to the evaluator. A policy sees
only the same bounded semantic observations and calibrated camera hypotheses.
This small environment measures evidence acquisition, not flight completion.
"""
from copy import deepcopy
from dataclasses import replace
from functools import lru_cache
import random

from examples.unified_spatial_replay import calibration, REFERENCE
from refnav.reasoning import build_context, WorldReference
from refnav.reasoning.evaluation import populated_world
from refnav.reasoning.inspection import InspectionInputs, Attempt, candidates

# Our own fixed experiment splits; entire layouts/runs belong to only one split.
SPLITS = {'train': range(100), 'validation': range(100, 130), 'test': range(300, 360)}
EPISODE_STEPS = 6


@lru_cache(maxsize=1)
def template():
    return populated_world(1).semantic_snapshot(.8)


class InspectionEpisode:
    def __init__(self, layout, regime='nominal'):
        self.layout, self.regime = layout, regime
        rng = random.Random(93000 + layout)
        self.source = deepcopy(template())
        opening = self.source['entities'][0]
        self.source.update(frame_id=REFERENCE.frame_id, clock_id=REFERENCE.clock_id,
                           epoch=0, timestamp=.8, query_timestamp=.8)
        self.source['entities'] = []
        self.truth = {}
        count = 2 + layout % 7
        for index in range(count):
            entity = deepcopy(opening)
            entity.update(entity_id=f'opening_{index}', frame_id=REFERENCE.frame_id,
                          epoch=0, clock_id=REFERENCE.clock_id, relations=[],
                          geometric_confidence=rng.uniform(.35, .95), semantic_confidence=.7)
            entity['geometry'].update(center=[rng.uniform(2., 4.), rng.uniform(-1.8, 1.8), 1.2],
                                      width_m=rng.uniform(.6, 1.6), height_m=None,
                                      uncertainty_m=rng.uniform(.01, .4))
            self.source['entities'].append(entity)
            # Occlusion is hidden; width/uncertainty are noisy observable correlates.
            visible = rng.random() > .20
            self.truth[entity['entity_id']] = (visible, rng.uniform(.6, 1.5))
        self.source['total_entities'] = count
        self.attempts, self.previous = (), None
        self.acquired = set()
        self.random = random.Random(170000 + layout)
        self._apply_regime()

    def _apply_regime(self):
        for entity in self.source['entities']:
            if self.regime == 'stale':
                entity['last_observed'] = -10.
                entity['source_timestamp'] = -10.
                entity['evidence'] = []
            elif self.regime == 'contradiction':
                entity['contradictions'] = 1
            elif self.regime == 'dead_end':
                self.truth[entity['entity_id']] = (False, 1.)
            elif self.regime == 'missing_support':
                entity['evidence'] = []
        if self.regime == 'reset':
            self.source['epoch'] = 1
        if self.regime == 'omissions':
            self.source['total_entities'] += 100

    def observe(self):
        context = build_context(self.source, now=.8, reference=WorldReference(
            REFERENCE.frame_id, REFERENCE.epoch, REFERENCE.clock_id))
        inputs = InspectionInputs(context.context_reference, REFERENCE, 'synthetic-observation',
                                  800_000_000, calibration(), (0., 0., 1.2),
                                  ((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.)),
                                  800_000_000, 'synthetic-level-camera', attempts=self.attempts,
                                  previous_entity_id=self.previous)
        return context, inputs

    def expected_measurement(self, candidate):
        """Training/evaluation only: hidden occlusion and an analytic observation model."""
        visible, _ = self.truth[candidate.entity_id]
        if not visible or candidate.entity_id in self.acquired:
            return 0.
        return min(.95, .15 + .8 * candidate.information_gain_heuristic)

    def act(self, entity_id):
        context, inputs = self.observe()
        option = next((c for c in candidates(context, inputs) if c.entity_id == entity_id), None)
        if option is None:
            return False
        success = self.random.random() < self.expected_measurement(option)
        self.attempts = (*self.attempts, Attempt(entity_id, .8, 'acquired' if success else 'failed'))[-32:]
        self.previous = entity_id
        if success:
            self.acquired.add(entity_id)
            entity = next(e for e in self.source['entities'] if e['entity_id'] == entity_id)
            entity['geometry']['height_m'] = self.truth[entity_id][1]
            entity['missing_evidence'] = [m for m in entity['missing_evidence']
                                          if m not in ('upper_boundary', 'lower_boundary')]
        return success
