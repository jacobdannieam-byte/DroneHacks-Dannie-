"""Simulator-specific launch geometry, separate from navigation map evidence.

The caller must bind this contract to the pinned world/vehicle and local frame.
It never writes FREE voxels and never authorizes route navigation or landing.
"""
from dataclasses import dataclass
import math
import re
import numpy as np


def directional_error(ned_error, ned_to_map):
    """Enclose a bounded NED error box after rotation, retaining every axis."""
    error=np.asarray(ned_error,dtype=float); rotation=np.asarray(ned_to_map,dtype=float)
    if error.shape!=(3,) or not np.isfinite(error).all() or np.any(error<0):
        raise ValueError('three finite nonnegative directional bounds required')
    if rotation.shape!=(3,3) or not np.isfinite(rotation).all() or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-8):
        raise ValueError('orthonormal frame transform required')
    return np.abs(rotation)@error


@dataclass(frozen=True)
class SimulatorLaunchReference:
    source_sha256: str
    frame_id: str
    epoch: int
    floor_z_m: float
    ceiling_z_m: float
    lower_xy_m: tuple
    upper_xy_m: tuple
    body_half_extent_m: tuple
    contact_height_m: float
    contact_tolerance_m: float
    obstacles: tuple = ()

    def __post_init__(self):
        if not re.fullmatch('[0-9a-f]{64}',self.source_sha256) or not self.frame_id or type(self.epoch) is not int or self.epoch<0:
            raise ValueError('pinned source and frame identity required')
        values=(*self.lower_xy_m,*self.upper_xy_m,*self.body_half_extent_m,
                self.floor_z_m,self.ceiling_z_m,self.contact_height_m,self.contact_tolerance_m)
        if len(self.lower_xy_m)!=2 or len(self.upper_xy_m)!=2 or len(self.body_half_extent_m)!=3 or not all(math.isfinite(v) for v in values):
            raise ValueError('finite launch geometry required')
        if (any(v<=0 for v in self.body_half_extent_m) or
                any(a>=b for a,b in zip(self.lower_xy_m,self.upper_xy_m)) or
                self.ceiling_z_m<=self.floor_z_m or not 0<=self.contact_tolerance_m<=.02 or
                self.contact_height_m<=0):
            raise ValueError('invalid launch geometry or contact allowance')
        for lo,hi in self.obstacles:
            if len(lo)!=3 or len(hi)!=3 or not all(math.isfinite(v) for v in (*lo,*hi)) or any(a>=b for a,b in zip(lo,hi)):
                raise ValueError('invalid obstacle box')

    def check(self, start, end, *, error_m, frame_id, epoch, source_sha256,
              phase, landed, contact_confirmed):
        if (frame_id,epoch,source_sha256)!=(self.frame_id,self.epoch,self.source_sha256):
            return 'launch_reference_mismatch'
        if phase not in ('wait','arm','climb','hover'):
            return 'launch_reference_has_no_route_authority'
        start=np.asarray(start,dtype=float); end=np.asarray(end,dtype=float)
        if start.shape!=(3,) or end.shape!=(3,) or not np.isfinite([start,end]).all():
            return 'invalid_launch_position'
        error=directional_error(error_m,np.eye(3))
        if end[2]<start[2]:
            return 'launch_requires_nondecreasing_motion'
        extent=np.asarray(self.body_half_extent_m)+error
        lo=np.minimum(start,end)-extent; hi=np.maximum(start,end)+extent
        if np.any(lo[:2]<self.lower_xy_m) or np.any(hi[:2]>self.upper_xy_m) or hi[2]>self.ceiling_z_m:
            return 'launch_side_or_ceiling_clearance'
        if lo[2]<self.floor_z_m:
            # Contact only constrains the floor-facing uncertainty. It does not
            # erase horizontal/upper uncertainty, obstacles or unsupported space.
            expected=self.floor_z_m+self.contact_height_m
            if not (phase in ('wait','arm','climb') and landed is True and contact_confirmed is True
                    and abs(start[2]-expected)<=self.contact_tolerance_m):
                return 'launch_floor_clearance'
            lo[2]=self.floor_z_m
        for obstacle_lo,obstacle_hi in self.obstacles:
            if np.all(hi>=obstacle_lo) and np.all(lo<=obstacle_hi):
                return 'launch_obstacle_clearance'
        return None


class BoundLaunchGuard:
    """Recheck a deployment-owned source binding on every startup tick."""
    def __init__(self, reference, source_valid, error_ned_m, transform, offset, *, alignment=None):
        self.reference=reference; self.source_valid=source_valid
        self.error=directional_error(error_ned_m,transform)
        self.transform=np.asarray(transform); self.offset=np.asarray(offset)
        self.alignment = alignment
        if self.offset.shape!=(3,) or not np.isfinite(self.offset).all() or not callable(source_valid):
            raise ValueError('live source binding and finite launch transform required')

    def check_output(self, mission, vehicle, inputs, output, *, phase):
        """Validate exact output with its originating phase, even across go transition.

        This is geometry only; it grants no authority for LAND or any command.
        The owner retains all command admission and deadline checks.
        """
        from types import SimpleNamespace
        if phase not in ('wait','arm','climb','hover'):
            return 'launch_reference_has_no_route_authority'
        if output.phase == 'land':
            return 'launch_reference_has_no_landing_authority'
        if not output.stream:
            return None
        if output.setpoint is None or len(output.setpoint)!=4:
            return 'invalid_launch_output'
        proxy=SimpleNamespace(home=mission.home,sp=output.setpoint,p=mission.p,phase=phase)
        return self.check_owner(proxy,vehicle,inputs)

    def check_owner(self, mission, vehicle, inputs):
        try:
            if self.source_valid() is not True: return 'launch_source_changed'
            if inputs is None or vehicle.pos is None: return 'launch_state_unavailable'
            alignment=inputs['alignment']
            if alignment.map.frame_id!=self.reference.frame_id or alignment.map.epoch!=self.reference.epoch:
                return 'launch_reference_mismatch'
            from onboard.adapters.geometry import R_ENU_NED, rot_z
            if (not np.allclose(rot_z(alignment.map_from_enu_yaw_rad)@R_ENU_NED,self.transform,atol=1e-10,rtol=0)
                    or not np.array_equal(np.asarray(alignment.ned_origin_in_map_m),self.offset)
                    or (self.alignment is not None and any(getattr(alignment,k)!=getattr(self.alignment,k)
                        for k in ('map','ned','provenance','position_error_m','angle_error_rad')))):
                return 'launch_transform_changed'
            # Preserve tracking and alignment uncertainty in every direction.
            bounds=inputs['vehicle']
            start=self.transform@np.asarray(vehicle.pos)+self.offset
            home=np.asarray(mission.home[:3] if mission.home is not None else vehicle.pos,dtype=float)
            command=np.asarray(mission.sp[:3] if mission.sp is not None else home,dtype=float)
            if not np.array_equal(command[:2],home[:2]):
                return 'launch_command_outside_home_column'
            target=home.copy()
            target[2]=min(command[2],home[2]-mission.p.height_m)
            end=self.transform@target+self.offset
            allowance=bounds.tracking_error_m+alignment.position_error_m
            allowance+=2*math.sin(alignment.angle_error_rad/2)*max(np.linalg.norm(vehicle.pos),np.linalg.norm(target))
            error=self.error+allowance
            # Validate the actual held command as well as the complete climb.
            # A downward correction is not covered by this launch contract.
            for finish in (self.transform@command+self.offset,end):
                reason=self.reference.check(start,finish,error_m=error,
                frame_id=alignment.map.frame_id,epoch=alignment.map.epoch,
                source_sha256=self.reference.source_sha256,phase=mission.phase,
                landed=vehicle.landed,contact_confirmed=vehicle.landed is True)
                if reason is not None: return reason
            return None
        except (ValueError,TypeError,AttributeError,KeyError,OSError):
            return 'launch_binding_invalid'
