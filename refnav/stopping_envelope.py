"""Conservative whole-body stopping volume backed by a reviewed measurement.

The ball covers every braking direction, rather than assuming a straight stop.
No default measurement or inferred free space is supplied.
"""
from dataclasses import dataclass
import math
import re
import numpy as np
from onboard.adapters.geometry import R_ENU_NED, rot_z

@dataclass(frozen=True)
class StoppingEnvelope:
    horizontal_radius_m: float
    half_height_m: float
    max_speed_mps: float
    reaction_s: float
    braking_distance_m: float
    evidence_sha256: str

    def __post_init__(self):
        values=(self.horizontal_radius_m,self.half_height_m,self.max_speed_mps,self.reaction_s,self.braking_distance_m)
        if not all(math.isfinite(v) and v>0 for v in values) or self.max_speed_mps>.5:
            raise ValueError('positive measured envelope and speed at most 0.50 m/s required')
        if not re.fullmatch('[0-9a-f]{64}',self.evidence_sha256):
            raise ValueError('stopping measurement SHA256 required')
        if self.body_radius_m+self.stop_distance_m>2.:
            raise ValueError('stopping envelope exceeds supported metric query radius')

    @property
    def body_radius_m(self):
        return math.hypot(self.horizontal_radius_m,self.half_height_m)

    @property
    def stop_distance_m(self):
        return self.max_speed_mps*self.reaction_s+self.braking_distance_m

    def check(self, state, snapshot, alignment, bounds, now_ns):
        speed=math.hypot(*state.velocity_mps)
        if not math.isfinite(speed) or speed>self.max_speed_mps:
            return 'stopping_speed_outside_measurement'
        point=np.asarray(state.position_m)
        mapped=rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED @ point + alignment.ned_origin_in_map_m
        margin=(bounds.state_position_error_m+bounds.tracking_error_m+alignment.position_error_m
                +2*math.sin(alignment.angle_error_rad/2)*np.linalg.norm(point))
        radius=max(bounds.radius_m,self.body_radius_m)+self.stop_distance_m
        result=snapshot.metric.check_sweep((mapped,),reference=alignment.map,now_ns=now_ns,
            vehicle_radius_m=radius,margin_m=margin)
        return None if result.status=='verified_free' else 'stopping_volume_unverified:'+result.status
