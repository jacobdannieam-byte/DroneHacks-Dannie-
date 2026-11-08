"""Single-epoch, read-only map localization over validated capture-time depth."""

from dataclasses import dataclass, replace
import math

import numpy as np

from refnav.depth import DepthFrame

from .map_registration import RegistrationPolicy, RegistrationQuality, SE3, register
from .spatial_input import AlignedDepth, NS, SpatialReference, stamp_ok


@dataclass(frozen=True)
class ReviewedAnchor:
    reference_version: str
    reference_epoch: int
    vio: SpatialReference
    valid_from_ns: int
    reference_from_vio: SE3
    evidence_id: str

    def __post_init__(self):
        if (not isinstance(self.reference_version, str) or len(self.reference_version) != 64
                or any(c not in "0123456789abcdef" for c in self.reference_version)
                or not isinstance(self.vio, SpatialReference) or not isinstance(self.reference_from_vio, SE3)
                or type(self.reference_epoch) is not int or self.reference_epoch < 0
                or not stamp_ok(self.valid_from_ns)
                or not isinstance(self.evidence_id, str) or not 0 < len(self.evidence_id.strip()) <= 128):
            raise ValueError("localization requires a reviewed map, epoch and initial transform")


@dataclass(frozen=True)
class LocalizationPolicy:
    keyframe_interval_ns: int = 250_000_000
    hold_ns: int = 500_000_000
    max_gap_ns: int = NS
    prior_age_ns: int = 2 * NS
    drift_m_per_s: float = .05
    drift_rad_per_s: float = .02

    def __post_init__(self):
        limits = (self.keyframe_interval_ns, self.hold_ns, self.max_gap_ns, self.prior_age_ns)
        if (not all(stamp_ok(v) for v in limits) or not 0 < limits[0] <= limits[1] <= limits[2] <= limits[3] <= 5 * NS
                or not 0 < self.drift_m_per_s <= .5 or not 0 < self.drift_rad_per_s <= .25):
            raise ValueError("invalid localization cadence or drift bounds")


@dataclass(frozen=True)
class MapLocalization:
    """Diagnostic snapshot v1. Absent pose means no valid map fix for this capture."""

    schema: int
    map_id: str
    map_version: str
    reference_frame: str
    reference_epoch: int | None
    vio: SpatialReference
    capture_ns: int | None
    pose_bracket_ns: tuple
    observation_id: str | None
    calibration_id: str | None
    status: str
    reason: str
    reference_from_body: SE3 | None = None
    reference_from_vio: SE3 | None = None
    translation_uncertainty_m: float | None = None
    rotation_uncertainty_rad: float | None = None
    quality: RegistrationQuality | None = None
    last_success_ns: int | None = None
    anchor_evidence_id: str | None = None


def _body_cloud(frame, cap):
    cal = frame.calibration
    stride = max(1, math.ceil(math.sqrt(frame.depth_m.size / cap)))
    while math.ceil(cal.width / stride) * math.ceil(cal.height / stride) > cap:
        stride += 1
    depth = DepthFrame(frame.depth_m, cal.K, cal.R_body_optical, cal.t_body_optical_m, np.eye(3), stride=stride)
    return depth.points(rmin=0., rmax=10.00001)  # SpatialWorld already masked its calibrated range.


class MapLocalizer:
    """The existing observation owner supplies aligned depth and epoch/health loss events.

    Use one instance per reviewed epoch in a serial diagnostic consumer. The
    last correction is kept as a bounded prior after rejection, never emitted
    as a fresh fix. Explicit tracking loss or a time gap fences the instance.
    """

    def __init__(self, reference, vio, anchor=None, *, registration=RegistrationPolicy(),
                 policy=LocalizationPolicy()):
        if not isinstance(vio, SpatialReference) or (anchor is not None and not isinstance(anchor, ReviewedAnchor)):
            raise ValueError("expected a VIO reference and optional reviewed anchor")
        self.reference = reference
        self.vio = vio
        self.anchor = anchor
        self.registration = registration
        self.policy = policy
        self._prior = None if anchor is None else anchor.reference_from_vio
        self._last_capture = self._last_processed = self._last_attempt = self._last_success = None
        self._fit_error = None
        self._fit_position = self._calibration = None
        self._fence = None
        self._snapshot = self._make(None, "unavailable", "awaiting_capture")

    def _make(self, frame, status, reason, **kwargs):
        anchor = self.anchor
        return MapLocalization(1, self.reference.spec.map_id, self.reference.version,
                               self.reference.spec.frame_id, None if anchor is None else anchor.reference_epoch,
                               self.vio, None if frame is None else frame.capture_ns,
                               () if frame is None else frame.pose.bracket_ns,
                               None if frame is None else frame.observation_id,
                               None if frame is None else frame.calibration.calibration_id,
                               status, reason, last_success_ns=self._last_success,
                               anchor_evidence_id=None if anchor is None else anchor.evidence_id, **kwargs)

    def _publish(self, frame, status, reason, **kwargs):
        self._snapshot = self._make(frame, status, reason, **kwargs)
        return self._snapshot

    def invalidate(self, reason="tracking_loss"):
        """Fence this reviewed alignment after a health or epoch fault."""
        self._fence = reason
        self._prior = None
        self._snapshot = self._without_fix(self._snapshot, reason)
        return self._snapshot

    @staticmethod
    def _without_fix(snapshot, reason):
        return replace(snapshot, status="unavailable", reason=reason,
                       reference_from_body=None, reference_from_vio=None,
                       translation_uncertainty_m=None, rotation_uncertainty_rad=None)

    def snapshot(self, now_ns):
        if not stamp_ok(now_ns) or (self._last_processed is not None and now_ns < self._last_processed):
            raise ValueError("localization query precedes processing time")
        if self._last_success is not None and now_ns - self._last_success > self.policy.hold_ns:
            if self._snapshot.status in ("accepted", "propagated"):
                return self._without_fix(self._snapshot, "map_fix_expired")
        return self._snapshot

    def update(self, frame):
        if not isinstance(frame, AlignedDepth):
            raise ValueError("expected SpatialWorld aligned depth")
        if frame.reference != self.vio:
            if frame.reference.epoch >= self.vio.epoch:
                self.invalidate("vio_reference_mismatch")
            return self._make(frame, "unavailable", "vio_reference_mismatch")
        capture = frame.capture_ns
        if (not all(stamp_ok(v) for v in (capture, frame.received_ns, frame.processed_ns))
                or not capture <= frame.received_ns <= frame.processed_ns
                or frame.processed_ns - capture > self.policy.hold_ns):
            return self._publish(frame, "unavailable", "invalid_or_stale_capture")
        if self._last_processed is not None and frame.processed_ns < self._last_processed:
            self.invalidate("clock_rewind_requires_reanchor")
        if self._last_capture is not None and capture <= self._last_capture:
            return self._make(frame, "rejected", "out_of_order_capture")
        if self._last_capture is not None and capture - self._last_capture > self.policy.max_gap_ns:
            self.invalidate("capture_gap_requires_reanchor")
        self._last_capture, self._last_processed = capture, frame.processed_ns
        if self._fence is not None:
            return self._publish(frame, "unavailable", self._fence)
        anchor = self.anchor
        if not self.reference.spec.reviewed_scale:
            return self._publish(frame, "unavailable", "unreviewed_reference_scale")
        if anchor is None:
            return self._publish(frame, "unavailable", "missing_reviewed_anchor")
        if anchor.vio != self.vio or anchor.reference_version != self.reference.version or capture < anchor.valid_from_ns:
            return self._publish(frame, "unavailable", "anchor_reference_or_time_mismatch")
        prior_ns = anchor.valid_from_ns if self._last_success is None else self._last_success
        if capture - prior_ns > self.policy.prior_age_ns:
            self.invalidate("registration_prior_expired_requires_reanchor")
            return self._publish(frame, "unavailable", self._fence)

        cal = frame.calibration
        if self._calibration is None:
            self._calibration = cal
        elif cal != self._calibration:
            self.invalidate("calibration_changed_requires_reanchor")
            return self._publish(frame, "unavailable", self._fence)
        if cal.extrinsic_position_error_m is None or cal.extrinsic_angle_error_rad is None:
            return self._publish(frame, "unavailable", "unreviewed_extrinsic_uncertainty")
        vio_from_body = SE3(frame.pose.rotation, frame.pose.position_m)
        predicted = self._prior.compose(vio_from_body)
        if self._last_attempt is not None and capture - self._last_attempt < self.policy.keyframe_interval_ns:
            if self._snapshot.status not in ("accepted", "propagated"):
                return self._publish(frame, "unavailable", "awaiting_registration_keyframe")
            return self._publish_fix(frame, predicted, "propagated", "vio_between_keyframes", self._snapshot.quality)

        self._last_attempt = capture
        result = register(self.reference, _body_cloud(frame, self.registration.max_source_points),
                          predicted, self.registration)
        if result.reference_from_body is None:
            return self._publish(frame, "rejected", result.quality.reason, quality=result.quality)
        # Diagnostic error indicators, including an angular lever arm. They
        # are not certified covariance or collision-margin guarantees.
        point_error = result.quality.rmse_m + cal.depth_error_m + cal.extrinsic_position_error_m
        self._fit_error = (point_error + frame.pose.position_error_m,
                           point_error / .5 + cal.extrinsic_angle_error_rad + frame.pose.angle_error_rad)
        self._prior = result.reference_from_body.compose(vio_from_body.inverse())
        self._fit_position = result.reference_from_body.translation_m
        self._last_success = capture
        return self._publish_fix(frame, result.reference_from_body, "accepted", "registration_accepted", result.quality)

    def _publish_fix(self, frame, pose, status, reason, quality):
        age = (frame.processed_ns - self._last_success) / NS
        if age > self.policy.hold_ns / NS:
            return self._publish(frame, "unavailable", "map_fix_expired")
        position, angle = self._fit_error
        traveled = math.dist(self._fit_position, pose.translation_m)
        return self._publish(frame, status, reason, reference_from_body=pose, reference_from_vio=self._prior,
                             translation_uncertainty_m=(position + frame.pose.position_error_m
                                                        + age * self.policy.drift_m_per_s + traveled * angle),
                             rotation_uncertainty_rad=angle + frame.pose.angle_error_rad + age * self.policy.drift_rad_per_s,
                             quality=quality)
