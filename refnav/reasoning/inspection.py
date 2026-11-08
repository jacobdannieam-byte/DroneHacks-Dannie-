"""Bound inspection hypotheses to observed geometry, camera and a fresh pose.

These immutable records supplement schema v1; they never declare a path safe.
The trajectory evaluator still checks every predicted swept vehicle volume.
"""
from dataclasses import dataclass
from hashlib import sha256
import math

from .schema import canonical, token, finite

# Our own conservative experiment limits, not measured aircraft capability.
MAX_STEP_METRES = .5
STANDOFF_METRES = .8
MAX_ATTEMPTS = 3
HISTORY_LIMIT = 32
HISTORY_SECONDS = 10.


@dataclass(frozen=True)
class Attempt:
    entity_id: str
    stamp_seconds: float
    outcome: str

    def __post_init__(self):
        if (not token(self.entity_id) or not finite(self.stamp_seconds)
                or self.outcome not in ('failed', 'acquired')):
            raise ValueError('Supply a grounded attempt with a finite time and observed outcome.')


@dataclass(frozen=True)
class InspectionInputs:
    context_reference: str
    reference: object
    observation_id: str
    capture_ns: int
    calibration: object
    position_metres: tuple
    rotation_map_optical: tuple
    state_capture_ns: int
    alignment_provenance: str
    camera_offset_metres: tuple = (0., 0., 0.)
    stage_id: str = 'inspection'
    attempts: tuple = ()
    previous_entity_id: str | None = None
    schema_version: int = 1

    def __post_init__(self):
        from refnav.world_model.spatial_input import DepthCalibration, SpatialReference, stamp_ok
        from refnav.world_model.observations import rotation_ok
        if (type(self.schema_version) is not int or self.schema_version != 1
                or not isinstance(self.reference, SpatialReference) or not isinstance(self.calibration, DepthCalibration)
                or not all(token(v) for v in (self.context_reference, self.observation_id,
                                              self.alignment_provenance, self.stage_id))
                or not stamp_ok(self.capture_ns) or not stamp_ok(self.state_capture_ns)
                or type(self.position_metres) is not tuple or len(self.position_metres) != 3
                or not all(finite(v) and abs(v) <= 1000 for v in self.position_metres)
                or type(self.camera_offset_metres) is not tuple or len(self.camera_offset_metres) != 3
                or not all(finite(v) and abs(v) <= 2 for v in self.camera_offset_metres)
                or type(self.rotation_map_optical) is not tuple
                or any(type(row) is not tuple for row in self.rotation_map_optical)
                or not rotation_ok(self.rotation_map_optical)
                or type(self.attempts) is not tuple or len(self.attempts) > HISTORY_LIMIT
                or not all(isinstance(a, Attempt) for a in self.attempts)
                or (self.previous_entity_id is not None and not token(self.previous_entity_id))):
            raise ValueError('Invalid immutable inspection inputs; rebuild from the current observation.')

    @property
    def identity(self):
        return sha256(canonical(self).encode()).hexdigest()


@dataclass(frozen=True)
class InspectionCandidate:
    entity_id: str
    position_metres: tuple
    mode: str
    missing_evidence: tuple
    information_gain_heuristic: float
    movement_cost: float
    risk_proxy: float
    mission_support: float
    failed_attempts: int
    switching: float
    relative_geometry_metres: tuple
    yaw_delta_map_rad: float = 0.


def attempt_count(context, inputs, entity_id):
    recent = [a for a in inputs.attempts if a.entity_id == entity_id
              and 0 <= context.context_created_at - a.stamp_seconds <= HISTORY_SECONDS]
    failures = 0
    for attempt in recent:
        failures = failures + 1 if attempt.outcome == 'failed' else 0
    return failures


def candidates(context, inputs):
    """One bounded viewpoint per eligible opening; visibility remains a heuristic.

    Optical axes are right, down, forward. Project the known center only: missing
    top/bottom boundaries are observation objectives, never invented geometry.
    """
    if inputs.context_reference != context.context_reference or not context.context_valid:
        return ()
    if ((inputs.reference.frame_id, inputs.reference.epoch, inputs.reference.clock_id) !=
            (context.reference.frame_id, context.reference.epoch, context.reference.clock_id)
            or any(a.stamp_seconds > context.context_created_at for a in inputs.attempts)
            or any(a.stamp_seconds > b.stamp_seconds for a, b in zip(inputs.attempts, inputs.attempts[1:]))):
        return ()
    if context.mission_goal.intent == 'reach_region':
        return ()
    result = []
    for entity in context.entities:
        failures = attempt_count(context, inputs, entity.entity_id)
        if (entity.semantic_type != 'opening' or entity.status in ('invalidated', 'stale', 'uncertain')
                or entity.age_s > context.stale_after_s or entity.contradictions
                or not any(e.kind == 'support' for e in entity.evidence)
                or entity.traversability != 'unknown' or failures >= MAX_ATTEMPTS):
            continue
        candidate = _viewpoint(context, inputs, entity, failures)
        if candidate is not None:
            result.append(candidate)
    return tuple(sorted(result, key=lambda c: c.entity_id))


def _viewpoint(context, inputs, entity, failures):
    delta = tuple(b - a for a, b in zip(inputs.position_metres, entity.geometry.center))
    distance = math.hypot(*delta)
    optical = tuple(sum(inputs.rotation_map_optical[i][j] * delta[i] for i in range(3)) for j in range(3))
    if optical[2] <= STANDOFF_METRES:
        return None
    fx, fy, cx, cy = inputs.calibration.K
    u, v = fx * optical[0] / optical[2] + cx, fy * optical[1] / optical[2] + cy
    yaw_delta = 0.
    if not (0 <= u < inputs.calibration.width and 0 <= v < inputs.calibration.height):
        # A bounded horizontal turn can bring a grounded target into view. This
        # is a visibility hypothesis; D3 must still check the stationary volume.
        forward = tuple(row[2] for row in inputs.rotation_map_optical)
        yaw_delta = math.atan2(delta[1], delta[0]) - math.atan2(forward[1], forward[0])
        yaw_delta = math.atan2(math.sin(yaw_delta), math.cos(yaw_delta))
        if abs(yaw_delta) > .45 or not 0 <= v < inputs.calibration.height:
            return None
        import numpy as np
        from onboard.adapters.geometry import rot_z
        turn = rot_z(yaw_delta)
        offset = np.asarray(inputs.camera_offset_metres)
        turned_delta = np.asarray(delta) + offset - turn @ offset
        turned_optical = (turn @ np.asarray(inputs.rotation_map_optical)).T @ turned_delta
        if turned_optical[2] <= STANDOFF_METRES:
            return None
        u, v = fx * turned_optical[0] / turned_optical[2] + cx, fy * turned_optical[1] / turned_optical[2] + cy
        if not (0 <= u < inputs.calibration.width and 0 <= v < inputs.calibration.height):
            return None
    step = min(MAX_STEP_METRES, max(0., distance - STANDOFF_METRES * 2))
    if yaw_delta:
        step = 0.
    point = tuple(a + step * d / distance - offset for a, d, offset in
                  zip(inputs.position_metres, delta, inputs.camera_offset_metres))
    targets = tuple(m for m in entity.missing_evidence if m in
                    ('upper_boundary', 'lower_boundary', 'fresh_consistent_observation'))
    if not targets:
        return None
    centrality = max(0., 1 - abs(u - cx) / max(cx, 1))
    gain = min(1., len(targets) / 2) * centrality / (1 + entity.geometry.uncertainty_m)
    return InspectionCandidate(entity.entity_id, point, 'yaw' if yaw_delta else ('move' if step else 'reobserve'), targets,
                               gain, max(step / MAX_STEP_METRES, abs(yaw_delta) / .45),
                               min(1., entity.geometry.uncertainty_m + 1 - entity.geometric_confidence),
                               float(entity.entity_id == context.mission_goal.target_entity_id), failures,
                               float(inputs.previous_entity_id not in (None, entity.entity_id)), delta, yaw_delta)


def make_inputs(context, snapshot, state, alignment, calibration, **history):
    """Convert the existing North/East/Down pose into the calibrated map camera pose."""
    import numpy as np
    from onboard.adapters.geometry import R_ENU_NED, rot_z
    from refnav.trajectory_evaluation import state_capture_ns
    transform = rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED
    # NED body uses forward/right/down; calibration uses forward/left/up.
    body = transform @ rot_z(state.yaw_rad) @ np.diag((1., -1., -1.))
    camera = transform @ state.position_m + alignment.ned_origin_in_map_m + body @ calibration.t_body_optical_m
    rotation = body @ calibration.R_body_optical
    return InspectionInputs(context.context_reference, snapshot.metric.reference, snapshot.metric.observation_id,
                            snapshot.metric.capture_ns, calibration, tuple(map(float, camera)),
                            tuple(tuple(map(float, row)) for row in rotation), state_capture_ns(state, alignment),
                            alignment.provenance, tuple(map(float, body @ calibration.t_body_optical_m)), **history)
