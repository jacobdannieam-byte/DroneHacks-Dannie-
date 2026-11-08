"""Controlled-SITL clearance from a reviewed participant survey and live exclusion lease."""

from dataclasses import asdict, dataclass, field, replace
from hashlib import sha256
import json
import math
import re
from time import monotonic_ns
from typing import Callable

import numpy as np

from onboard.adapters.geometry import rot_z
from refnav.stopping_envelope import StoppingEnvelope
from refnav.world_model.spatial_input import MAX_LOCAL_M, NS, SpatialReference, stamp_ok, vector


SCOPE = 'controlled_sitl_participant_map'
AUTHORIZED_STATIC = 'authorized_static'
MAX_CERTIFICATE_NS = 300 * NS
MAX_EXCLUSION_NS = 60 * NS


def _digest(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def _identifier(value):
    return isinstance(value, str) and 0 < len(value.strip()) <= 128


def _interval(start, end, limit):
    return stamp_ok(start) and stamp_ok(end) and 0 < end - start <= limit


def _json_digest(value):
    return sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def alignment_identity(alignment):
    """Bind the authored alignment, including reset epoch, independently of its live sample stamp."""
    return _json_digest({name: value for name, value in asdict(alignment).items() if name != 'capture_ns'})


@dataclass(frozen=True)
class ClearanceCertificate:
    certificate_sha256: str
    scope: str
    participant_map_sha256: str
    review_id: str
    reference: SpatialReference
    alignment_sha256: str
    bounds_min_m: tuple
    bounds_max_m: tuple
    map_from_prior_yaw_rad: float
    prior_origin_in_map_m: tuple
    position_error_m: float
    angle_error_rad: float
    valid_from_ns: int
    valid_until_ns: int
    stopping_evidence_sha256: str

    def __post_init__(self):
        if (self.scope != SCOPE or not isinstance(self.reference, SpatialReference)
                or not _identifier(self.review_id)
                or not all(_digest(value) for value in (self.certificate_sha256, self.participant_map_sha256,
                                                       self.alignment_sha256, self.stopping_evidence_sha256))
                or not _interval(self.valid_from_ns, self.valid_until_ns, MAX_CERTIFICATE_NS)):
            raise ValueError('reviewed scoped certificate, reference and bounded validity required')
        for name in ('bounds_min_m', 'bounds_max_m', 'prior_origin_in_map_m'):
            point = vector(getattr(self, name), 3)
            if max(map(abs, point)) > MAX_LOCAL_M:
                raise ValueError('prior geometry outside local coordinates')
            object.__setattr__(self, name, point)
        scalars = (self.map_from_prior_yaw_rad, self.position_error_m, self.angle_error_rad)
        if (any(lo >= hi for lo, hi in zip(self.bounds_min_m, self.bounds_max_m))
                or not all(type(value) in (int, float) and math.isfinite(value) for value in scalars)
                or not -math.pi <= self.map_from_prior_yaw_rad <= math.pi
                or not 0 <= self.position_error_m <= 1. or not 0 <= self.angle_error_rad <= .5):
            raise ValueError('bounded reviewed prior geometry and transform uncertainty required')

    @classmethod
    def from_json(cls, raw_bytes, *, expected_sha256):
        """Pin exact reviewed bytes. The deployment separately verifies the participant source and permission."""
        if (type(raw_bytes) is not bytes or not 0 < len(raw_bytes) <= 8192 or not _digest(expected_sha256)
                or sha256(raw_bytes).hexdigest() != expected_sha256):
            raise ValueError('certificate bytes do not match the reviewed identity')

        def unique_object(pairs):
            result = dict(pairs)
            if len(result) != len(pairs):
                raise ValueError('duplicate certificate key')
            return result

        try:
            fields = json.loads(raw_bytes, object_pairs_hook=unique_object)
            fields['reference'] = SpatialReference(**fields['reference'])
            return cls(certificate_sha256=expected_sha256, **fields)
        except (TypeError, KeyError, UnicodeError, RecursionError) as error:
            raise ValueError('invalid certificate document') from error

    def contains_segments(self, mapped_points, radius):
        """Enclose each continuous inflated segment within the one reviewed prior box."""
        points = (mapped_points - self.prior_origin_in_map_m) @ rot_z(self.map_from_prior_yaw_rad)
        before, after = (points[:-1], points[1:]) if len(points) > 1 else (points, points)
        distance = np.maximum(np.linalg.norm(before, axis=1), np.linalg.norm(after, axis=1))
        inflation = radius + self.position_error_m + 2 * math.sin(self.angle_error_rad / 2) * distance
        low = np.nextafter(np.minimum(before, after) - inflation[:, None], -np.inf)
        high = np.nextafter(np.maximum(before, after) + inflation[:, None], np.inf)
        return np.all((low >= self.bounds_min_m) & (high <= self.bounds_max_m), axis=1)


@dataclass(frozen=True)
class ExclusionLease:
    lease_id: str
    certificate_sha256: str
    valid_from_ns: int
    valid_until_ns: int
    wall_from_ns: int
    wall_until_ns: int

    def __post_init__(self):
        if (not _identifier(self.lease_id) or not _digest(self.certificate_sha256)
                or not _interval(self.valid_from_ns, self.valid_until_ns, MAX_EXCLUSION_NS)
                or not _interval(self.wall_from_ns, self.wall_until_ns, MAX_EXCLUSION_NS)):
            raise ValueError('separate bounded exclusion lease required on both clocks')


@dataclass(frozen=True)
class MapAssistedClearance:
    certificate: ClearanceCertificate
    lease: ExclusionLease
    verify_exclusion: Callable[[ExclusionLease], bool]
    stopping_envelope: StoppingEnvelope
    monotonic_ns: Callable[[], int] = monotonic_ns
    identity: tuple = field(init=False)

    def __post_init__(self):
        certificate, lease = self.certificate, self.lease
        if (not isinstance(certificate, ClearanceCertificate) or not isinstance(lease, ExclusionLease)
                or not isinstance(self.stopping_envelope, StoppingEnvelope)
                or lease.certificate_sha256 != certificate.certificate_sha256
                or not certificate.valid_from_ns <= lease.valid_from_ns < lease.valid_until_ns <= certificate.valid_until_ns
                or certificate.stopping_evidence_sha256 != self.stopping_envelope.evidence_sha256
                or not callable(self.verify_exclusion) or not callable(self.monotonic_ns)):
            raise ValueError('certificate, live exclusion and measured stopping evidence must agree')

        identity = (certificate.certificate_sha256, lease.lease_id,
                    _json_digest((asdict(certificate), asdict(lease), asdict(self.stopping_envelope))))
        object.__setattr__(self, 'identity', identity)

    def attach(self, inputs):
        """Retain the exact H1 metric and add this separate authority to the snapshot."""
        snapshot = inputs['snapshot']
        if snapshot.clearance is not None and snapshot.clearance is not self:
            raise ValueError('conflicting map-assisted authority')
        return {**inputs, 'snapshot': replace(snapshot, clearance=self)}

    @staticmethod
    def observation_reason(metric, alignment, now_ns):
        if (metric.reference != alignment.map or not metric.available or not stamp_ok(now_ns)
                or not all(stamp_ok(stamp) for stamp in (metric.capture_ns, metric.received_ns, metric.query_ns))
                or not metric.capture_ns <= metric.received_ns <= metric.query_ns <= now_ns
                or now_ns - metric.capture_ns > 250_000_000):
            return 'prior_live_observation_unavailable'
        return None

    def reason(self, alignment, now_ns):
        if (alignment.map != self.certificate.reference
                or alignment_identity(alignment) != self.certificate.alignment_sha256):
            return 'prior_alignment_mismatch'
        lease = self.lease
        if not stamp_ok(now_ns) or not lease.valid_from_ns <= now_ns < lease.valid_until_ns:
            return 'prior_ros_lease_expired_or_future'
        try:
            wall_ns = self.monotonic_ns()
            if not stamp_ok(wall_ns) or not lease.wall_from_ns <= wall_ns < lease.wall_until_ns:
                return 'prior_wall_lease_expired_or_future'
            if self.verify_exclusion(lease) is not True:
                return 'prior_exclusion_unverified'
            wall_ns = self.monotonic_ns()
            if not stamp_ok(wall_ns) or not lease.wall_from_ns <= wall_ns < lease.wall_until_ns:
                return 'prior_wall_lease_expired_or_future'
        except Exception:
            return 'prior_exclusion_unverified'
        return None


def clearance_sweep(snapshot, mapped_points, *, alignment, now_ns, vehicle_radius_m, margin_m):
    """Keep current occupied evidence as a veto and label any prior-backed volume separately."""
    metric, clearance = snapshot.metric, snapshot.clearance
    observed = metric.check_sweep(mapped_points, reference=alignment.map, now_ns=now_ns,
                                  vehicle_radius_m=vehicle_radius_m, margin_m=margin_m)
    if clearance is None or observed.status == 'occupied':
        return observed
    reason = clearance.observation_reason(metric, alignment, now_ns) or clearance.reason(alignment, now_ns)
    if reason is not None:
        return replace(observed, status='unknown', reason=reason)
    if not clearance.certificate.contains_segments(np.asarray(mapped_points), vehicle_radius_m + margin_m).all():
        return replace(observed, status='unknown', reason='outside_certified_volume')
    return (observed if observed.status == 'verified_free'
            else replace(observed, status=AUTHORIZED_STATIC, reason='prior_authorized_swept_volume'))
