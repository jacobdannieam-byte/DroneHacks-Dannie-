"""Reviewed ROS observation bindings for a local simulator. No truth or command translation."""

from dataclasses import dataclass
import re
from types import MappingProxyType

from examples.gazebo_sensor_contract import SIM_BINDINGS, SIM_DEPTH, SIM_ESTIMATOR, SIM_INFO
from examples.h1_observation import TF, TRACKING, PX4_TYPES
from examples.spatial_bag_input import TOPICS


OWNER_ROLES = ("vehicle_attitude", "vehicle_status", "vehicle_land_detected")


def _topic(name):
    return (isinstance(name, str) and len(name) <= 120
            and re.fullmatch(r"(/[A-Za-z_][A-Za-z_0-9]*)+", name) is not None)


class GazeboObservations:
    """Map same-schema ROS messages without rewriting stamps, frames or health."""

    def __init__(self, profile, sources, *, profile_id, owner_topics=None):
        expected = {**TOPICS, TF: "tf2_msgs/msg/TFMessage", TRACKING: "diagnostic_msgs/msg/DiagnosticArray",
                    **{topic: PX4_TYPES[role] for role, topic in profile.px4_topics.items()}}
        if not isinstance(profile_id, str) or not profile_id.strip() or len(profile_id) > 100:
            raise ValueError("reviewed simulator profile identity required")
        if set(sources) != set(expected):
            raise ValueError("every H1 source needs a binding")
        bindings = {}
        for canonical, fields in sources.items():
            name, kind, provenance = (fields[k] for k in ("topic", "type", "provenance"))
            sim = profile.tracking_source == SIM_ESTIMATOR
            permitted_sim_image = sim and (canonical, name) in (
                ("/camera/depth/image_raw", SIM_DEPTH), ("/camera/depth/camera_info", SIM_INFO))
            if (not _topic(name) or kind != expected[canonical]
                    or (sim and canonical in SIM_BINDINGS and SIM_BINDINGS[canonical] != name)
                    or not isinstance(provenance, str) or not provenance.strip() or len(provenance) > 240
                    or any(s in name.lower() for s in ("ground_truth", "groundtruth",
                                                      "referee", "/truth", "model_states", "/score"))
                    or ("simulator_only" in name.lower() and not permitted_sim_image)
                    or (name.startswith("/gazebo_hybrid/") and not sim)
                    or name in bindings or (name.startswith("/fmu/") and not canonical.startswith("/fmu/"))):
                raise ValueError(f"invalid observation binding: {canonical}")
            if (canonical in ("/clock", TF) or canonical.startswith("/fmu/")) and name != canonical:
                raise ValueError(f"canonical clock, static TF and PX4 interfaces required: {canonical}")
            bindings[name] = (canonical, kind)
        self.profile_id = profile_id
        self.simulated_estimator = profile.tracking_source == SIM_ESTIMATOR
        self.topics = MappingProxyType({name: kind for name, (_, kind) in bindings.items()})
        self._canonical = MappingProxyType({name: canonical for name, (canonical, _) in bindings.items()})
        self._native = MappingProxyType({canonical: name for name, canonical in self._canonical.items()})
        self.owner_topics = MappingProxyType(dict(owner_topics or {}))
        if owner_topics is not None and (set(owner_topics) != set(OWNER_ROLES)
                or any(not isinstance(name, str) or not re.fullmatch(rf"/fmu/out/{role}(?:_v[0-9]+)?", name)
                       for role, name in owner_topics.items())):
            raise ValueError("owner status, attitude and landed topics must match pinned PX4 schemas")

    def canonical(self, topic):
        return self._canonical[topic]

    def native(self, canonical):
        return self._native[canonical]

    @property
    def can_attach_owner(self):
        return bool(self.owner_topics)


@dataclass(frozen=True)
class IsolationApproval:
    """Reviewed trial identity plus a live deployment-owned isolation probe."""

    profile_id: str
    domain_id: int
    evidence_id: str
    verify_isolation: object

    def __post_init__(self):
        if (type(self.domain_id) is not int or not 1 <= self.domain_id <= 232
                or not all(isinstance(v, str) and 0 < len(v.strip()) <= 100
                           for v in (self.profile_id, self.evidence_id))
                or not callable(self.verify_isolation)):
            raise ValueError("isolated trial needs a reviewed identity, domain and live probe")

    def permits(self, adapter, domain):
        if adapter.profile_id != self.profile_id or domain != str(self.domain_id):
            return False
        try:
            return self.verify_isolation(self.profile_id, self.evidence_id) is True
        except Exception:
            # A missing/crashed deployment probe never grants authority.
            return False
