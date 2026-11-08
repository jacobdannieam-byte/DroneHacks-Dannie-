"""Offline small-angle waypoint previews. No ROS publisher or clearance authority."""
from dataclasses import dataclass
import math

from refnav.dynamics.validation import wrap_yaw


@dataclass(frozen=True)
class LookPreview:
    offsets_rad: tuple = (-math.pi / 9, math.pi / 9, 0.)
    yaw_rate_limit_rad_s: float = .25
    dwell_s: float = .30

    def __post_init__(self):
        if (not 1 <= len(self.offsets_rad) <= 3 or self.offsets_rad[-1] != 0.
                or any(not math.isfinite(v) or abs(v) > math.pi / 6 for v in self.offsets_rad)
                or not 0 < self.yaw_rate_limit_rad_s <= .25 or not .2 <= self.dwell_s <= 1.):
            raise ValueError('Bounded look requires <=30 degree offsets, <=0.25 rad/s and return to heading')

    def targets(self, heading_rad, *, stationary, clearance_valid, fresh_depth, stop_allowed):
        if not math.isfinite(heading_rad):
            raise ValueError('Finite heading required')
        if not all((stationary, clearance_valid, fresh_depth, stop_allowed)):
            return ()
        previous, elapsed, rows = 0., 0., []
        for offset in self.offsets_rad:
            elapsed += abs(offset - previous) / self.yaw_rate_limit_rad_s + self.dwell_s
            rows.append(dict(yaw_rad=wrap_yaw(heading_rad + offset), offset_rad=offset,
                             earliest_completion_s=elapsed, requires_new_captured_depth=True,
                             command_authority=False))
            previous = offset
        return tuple(rows)
