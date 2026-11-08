"""Simulator-specific launch geometry, separate from navigation map evidence.

The caller must bind this contract to the pinned world/vehicle and local frame.
It never writes FREE voxels and never authorizes route navigation or landing.
"""
from dataclasses import dataclass
import math
import re
import numpy as np

from onboard.adapters.geometry import R_ENU_NED, R_FRD_FLU, rot_z
from refnav.depth import quat_to_mat
from refnav.trajectory_evaluation import state_capture_ns
from refnav.world_model.spatial_input import NS, stamp_ok


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
        reason, lo = self._space_reason(start, end, error)
        if reason:
            return reason
        if lo[2]<self.floor_z_m:
            expected=self.floor_z_m+self.contact_height_m
            if not (phase in ('wait','arm','climb') and landed is True and contact_confirmed is True
                    and abs(start[2]-expected)<=self.contact_tolerance_m):
                return 'launch_floor_clearance'
        return None

    def _space_reason(self, start, end, error):
        """All-attitude side, ceiling and obstacle checks, including at pad contact."""
        extent=np.asarray(self.body_half_extent_m)+error
        lo=np.minimum(start,end)-extent; hi=np.maximum(start,end)+extent
        if np.any(lo[:2]<self.lower_xy_m) or np.any(hi[:2]>self.upper_xy_m) or hi[2]>self.ceiling_z_m:
            return 'launch_side_or_ceiling_clearance', lo
        for obstacle_lo,obstacle_hi in self.obstacles:
            if np.all(hi>=obstacle_lo) and np.all(lo<=obstacle_hi):
                return 'launch_obstacle_clearance', lo
        return None, lo


@dataclass(frozen=True)
class ContactTransition:
    """Reviewed isolated-SITL model bounds with no operating defaults.

    ``relative_height_error_m`` bounds differential estimation from the fixed
    contact sample. ``max_downward_tracking_excursion_m`` independently bounds
    continuous downward tracking beyond the lowest checked center or command,
    from the oldest permitted state capture through effective command retention
    and the reviewed loss response. The horizon must cover that full response.
    Angular reach uses the oldest allowed attitude plus that horizon. Sampled
    speed and attitude checks can contradict those model bounds. They cannot
    establish them. The support tolerance applies to the designated pad contact.
    """

    evidence_id: str
    pad_center_xy_m: tuple
    pad_half_width_m: float
    start_tolerance_m: float
    support_band_m: float
    max_support_penetration_m: float
    max_contact_command_lead_m: float
    relative_height_error_m: float
    max_downward_tracking_excursion_m: float
    attitude_error_rad: float
    max_angular_rate_rad_s: float
    max_tilt_rad: float
    max_speed_mps: float
    max_climb_rate_mps: float
    max_center_rise_m: float
    max_horizontal_drift_m: float
    attitude_max_age_ns: int
    contact_dwell_ns: int
    control_horizon_ns: int
    max_ros_ns: int
    max_wall_ns: int

    def __post_init__(self):
        pad = tuple(self.pad_center_xy_m)
        if (not isinstance(self.evidence_id, str) or not self.evidence_id.strip() or len(self.evidence_id) > 128
                or len(pad) != 2 or not all(type(v) in (int, float) and math.isfinite(v) for v in pad)):
            raise ValueError('reviewed support pad and model evidence required')
        object.__setattr__(self, 'pad_center_xy_m', pad)
        limits = ((self.pad_half_width_m, 1.), (self.start_tolerance_m, .05), (self.support_band_m, .05),
                  (self.max_support_penetration_m, .02), (self.max_contact_command_lead_m, .15),
                  (self.relative_height_error_m, .05), (self.max_downward_tracking_excursion_m, .02),
                  (self.attitude_error_rad, .10), (self.max_angular_rate_rad_s, 1.), (self.max_tilt_rad, .20),
                  (self.max_speed_mps, .50), (self.max_climb_rate_mps, .20), (self.max_center_rise_m, 1.),
                  (self.max_horizontal_drift_m, .15))
        clocks = ((self.attitude_max_age_ns, 100_000_000), (self.contact_dwell_ns, NS), (self.control_horizon_ns, 2 * NS),
                  (self.max_ros_ns, 15 * NS), (self.max_wall_ns, 45 * NS))
        if (any(type(value) not in (int, float) or not 0 < value <= limit for value, limit in limits)
                or any(not stamp_ok(value) or not 0 < value <= limit for value, limit in clocks)
                or not 100_000_000 <= self.contact_dwell_ns or not NS <= self.control_horizon_ns
                or not self.start_tolerance_m <= self.support_band_m < self.max_center_rise_m):
            raise ValueError('invalid bounded contact-transition assumptions')


class _ContactSession:
    """Single-use contact anchor followed by an irreversible airborne handoff."""

    def __init__(self, contract, vertices, support, floor_contact_m):
        self.contract = contract
        self.vertices = np.array(vertices, dtype=float, copy=True)
        if (self.vertices.ndim != 2 or self.vertices.shape[1:] != (3,) or not 8 <= len(self.vertices) <= 1024
                or not np.isfinite(self.vertices).all() or self.vertices[:, 2].min() != -floor_contact_m):
            raise ValueError('exact supported collision vertices required')
        self.vertices.setflags(write=False)
        self.radii = np.linalg.norm(self.vertices, axis=1)
        self.support = np.array(support, dtype=bool, copy=True)
        if (self.support.shape != (len(self.vertices),) or not np.any(self.support) or np.all(self.support)
                or not np.array_equal(self.support, self.vertices[:, 2] == -floor_contact_m)):
            raise ValueError('every lowest collision vertex must be explicitly designated as reviewed support')
        self.support.setflags(write=False)
        self.anchor = self.clocks = self.last = self.mission = self.reset = self.command_z = None
        self.last_q = None
        self.last_position = None
        self.peak_rise = 0.
        self.home = self.rest = self.command_ns = None
        self.rest_mission = self.rest_reset = self.rest_position = None
        self.handed_off = False
        self.separated = False
        self.failure = None

    def revoke(self, reason):
        if self.anchor is not None and self.failure is None:
            self.failure = reason
        elif self.anchor is None and reason != 'launch_transition_contact_dwell':
            self.rest = None
        return reason

    def freshness_reason(self, vehicle, inputs, now_ns, wall_ns):
        if self.failure:
            return self.failure
        if not stamp_ok(now_ns) or not stamp_ok(wall_ns) or inputs is None:
            return 'launch_transition_clock_or_state_unavailable'
        if self.clocks is not None:
            if (now_ns < self.last[0] or wall_ns < self.last[1]
                    or (not self.handed_off and (now_ns - self.clocks[0] >= self.contract.max_ros_ns
                                                or wall_ns - self.clocks[1] >= self.contract.max_wall_ns))):
                return 'launch_transition_clock_or_deadline'
            maximum_gap = min(self.contract.control_horizon_ns, 250_000_000)
            if not self.handed_off and max(now_ns - self.last[0], wall_ns - self.last[1]) > maximum_gap:
                return 'launch_transition_control_gap'
        capture, receipt = vehicle.attitude_capture_ns, vehicle.attitude_received_ns
        local_capture = state_capture_ns(inputs['state'], inputs['alignment'])
        decision_ns = inputs['now_ns']
        if (not all(stamp_ok(v) for v in (capture, receipt, vehicle.landed_capture_ns, local_capture, decision_ns))
                or not capture <= receipt <= now_ns or now_ns - capture > self.contract.attitude_max_age_ns
                or not 0 <= now_ns - local_capture <= 50_000_000
                or not 0 <= now_ns - decision_ns <= 50_000_000
                or not 0 <= now_ns - vehicle.landed_capture_ns <= 1_500_000_000
                or type(vehicle.landed) is not bool
                or type(vehicle.attitude_reset_counter) is not int or not 0 <= vehicle.attitude_reset_counter <= 255):
            return 'launch_transition_telemetry_unverified'
        if self.reset is not None and self.reset != (vehicle.attitude_reset_counter, inputs['estimator_reset_counter']):
            return 'launch_transition_reset'
        if self.last is not None and (capture < self.last[2] or local_capture < self.last[3]):
            return 'launch_transition_sample_rewind'
        return None

    def _orientation(self, vehicle, transform):
        q = np.asarray(vehicle.q, dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or not .99 <= np.linalg.norm(q) <= 1.01:
            raise ValueError('fresh normalized PX4 quaternion required')
        rotation = transform @ quat_to_mat(*(q / np.linalg.norm(q))) @ R_FRD_FLU
        tilt = math.acos(float(np.clip(rotation[2, 2], -1., 1.)))
        c = self.contract
        reach = c.attitude_error_rad + c.max_angular_rate_rad_s * ((c.attitude_max_age_ns + c.control_horizon_ns) / NS)
        projected_z = self.vertices @ rotation[2]
        lower_z = projected_z - 2 * np.sin(reach / 2) * self.radii
        return q / np.linalg.norm(q), tilt + reach, projected_z, lower_z

    def check(self, ref, mission, vehicle, inputs, phase, start, command, end, error, transform, now_ns, wall_ns,
              *, commit):
        c = self.contract
        reason = self.freshness_reason(vehicle, inputs, now_ns, wall_ns)
        if reason:
            return reason
        for finish in (command, end):
            reason, _ = ref._space_reason(start, finish, error)
            if reason:
                return reason
        lo = min(start[2], command[2], end[2]) - ref.body_half_extent_m[2] - error[2]
        home = (tuple(mission.home) if mission.home is not None else (*vehicle.pos, vehicle.heading),
                mission.p.height_m, mission.p.climb_rate_mps)
        if self.anchor is not None and (self.mission is not mission or self.home != home):
            return 'launch_transition_owner_changed'
        if self.clocks is not None and phase == 'wait' and now_ns != self.clocks[0]:
            return 'launch_transition_restart_refused'
        if self.command_z is not None:
            command_rise = command[2] - self.command_z
            if command_rise < -1e-9:
                return 'launch_transition_descending_command'
            if command_rise > c.max_climb_rate_mps * (inputs['now_ns'] - self.command_ns) / NS + 1e-9:
                return 'launch_transition_command_rate'
        if self.handed_off:
            if commit:
                self.last = (now_ns, wall_ns, vehicle.attitude_capture_ns,
                             state_capture_ns(inputs['state'], inputs['alignment']))
                self.command_z, self.command_ns = command[2], inputs['now_ns']
            return None if lo > ref.floor_z_m else 'launch_floor_clearance_after_handoff'
        if phase == 'hover':
            return 'launch_transition_handoff_required'
        if not 0 < mission.p.climb_rate_mps <= c.max_climb_rate_mps:
            return 'launch_transition_climb_rate'
        state = inputs['state']
        local_capture = state_capture_ns(state, inputs['alignment'])
        # The reviewed H1 velocity and position must describe the same local state used by the owner.
        if (math.dist(state.position_m, vehicle.pos) > inputs['vehicle'].state_position_error_m
                or math.hypot(*state.velocity_mps) > c.max_speed_mps):
            return 'launch_transition_state_or_speed'
        if state.velocity_mps[2] * (c.control_horizon_ns + 50_000_000) / NS > c.max_downward_tracking_excursion_m:
            return 'launch_transition_descending_state'
        current = transform @ np.asarray(state.position_m) + np.asarray(inputs['alignment'].ned_origin_in_map_m)
        q, tilt, projected_z, lower_z = self._orientation(vehicle, transform)
        if tilt > c.max_tilt_rad:
            return 'launch_transition_attitude'
        if self.last_q is not None:
            angular_delta = 2 * math.acos(float(np.clip(abs(q @ self.last_q), 0., 1.)))
            attitude_dt = (vehicle.attitude_capture_ns - self.last[2]) / NS
            if angular_delta > 2 * c.attitude_error_rad + c.max_angular_rate_rad_s * attitude_dt:
                return 'launch_transition_angular_rate'
        anchor = self.anchor
        if anchor is None:
            expected = ref.floor_z_m + ref.contact_height_m
            if (phase != 'wait' or vehicle.armed is not False or vehicle.landed is not True
                    or abs(current[2] - expected) > c.start_tolerance_m
                    or abs(start[2] - expected) > c.start_tolerance_m
                    or math.dist(start[:2], c.pad_center_xy_m) > c.start_tolerance_m
                    or math.hypot(*state.velocity_mps) > .05
                    or abs(command[2] - start[2]) > c.start_tolerance_m):
                return 'launch_transition_contact_anchor_unverified'
            # At reviewed resting contact, the lowest support point fixes a one-sided
            # altitude reference. Use the worst attitude error on the support face.
            uncertainty = 2 * math.sin(c.attitude_error_rad / 2) * self.radii[self.support]
            anchor = (tuple(current), -float(np.min(projected_z[self.support] + uncertainty)))
        anchor_pos, contact_down = anchor
        rise = current[2] - anchor_pos[2]
        if (math.dist(current[:2], anchor_pos[:2]) > c.max_horizontal_drift_m
                or rise > c.max_center_rise_m):
            return 'launch_transition_drift_or_height'
        if self.last_position is not None:
            maximum_step = (c.max_speed_mps * (local_capture - self.last[3]) / NS
                            + 2 * inputs['vehicle'].state_position_error_m)
            if math.dist(current, self.last_position) > maximum_step:
                return 'launch_transition_position_jump'
        if rise < self.peak_rise - c.relative_height_error_m:
            return 'launch_transition_height_regression'
        footprint = np.maximum(np.abs(start[:2] - c.pad_center_xy_m), np.abs(command[:2] - c.pad_center_xy_m))
        if np.any(footprint + error[:2] + np.asarray(ref.body_half_extent_m[:2]) > c.pad_half_width_m):
            return 'launch_transition_outside_support_pad'
        full_clearance = lo > ref.floor_z_m
        separation = self.separated
        if not full_clearance:
            # Propagate the reviewed differential height error across this short control
            # horizon. Floor contact is possible only on the named pad-support face.
            relative_height = min(current[2], start[2], command[2]) - anchor_pos[2]
            lower = (contact_down + relative_height - c.relative_height_error_m
                     - c.max_downward_tracking_excursion_m + lower_z)
            if np.any(lower[~self.support] <= 0):
                return 'launch_transition_nonsupport_floor_intersection'
            support_clearance = float(lower[self.support].min())
            if support_clearance > 0:
                separation = True
            elif (separation or max(current[2], start[2]) - anchor_pos[2] > c.support_band_m
                    or command[2] - min(current[2], start[2]) > c.max_contact_command_lead_m
                    or support_clearance < -c.max_support_penetration_m):
                return 'launch_transition_support_clearance'
        if self.anchor is None:
            reset = (vehicle.attitude_reset_counter, inputs['estimator_reset_counter'])
            if (self.rest is None or self.rest_mission is not mission or self.rest_reset != reset
                    or math.dist(current, self.rest_position) > c.start_tolerance_m
                    or not 0 <= local_capture - self.rest[1] <= 100_000_000
                    or now_ns < self.rest[2] or wall_ns < self.rest[3]):
                self.rest = (local_capture, local_capture, now_ns, wall_ns)
                self.rest_mission, self.rest_reset, self.rest_position = mission, reset, tuple(current)
            self.rest = (self.rest[0], local_capture, self.rest[2], self.rest[3])
            if min(local_capture - self.rest[0], now_ns - self.rest[2], wall_ns - self.rest[3]) < c.contact_dwell_ns:
                return 'launch_transition_contact_dwell'
        if commit:
            if self.anchor is None:
                self.anchor, self.clocks, self.mission = anchor, (now_ns, wall_ns), mission
                self.reset = (vehicle.attitude_reset_counter, inputs['estimator_reset_counter'])
                self.home = home
            self.command_z, self.command_ns = command[2], inputs['now_ns']
            self.last = (now_ns, wall_ns, vehicle.attitude_capture_ns, local_capture)
            self.last_q = q
            self.last_position = tuple(current)
            self.peak_rise = max(self.peak_rise, rise)
            self.separated = separation or full_clearance
            self.handed_off = full_clearance
        return None


class BoundLaunchGuard:
    """Recheck a deployment-owned source binding on every startup tick."""
    def __init__(self, reference, source_valid, error_ned_m, transform, offset, *, alignment=None,
                 contact_transition=None, collision_vertices=None, support_vertices=None):
        self.reference=reference; self.source_valid=source_valid
        self.error=directional_error(error_ned_m,transform)
        self.transform=np.array(transform, copy=True); self.offset=np.array(offset, copy=True)
        self.transform.setflags(write=False); self.offset.setflags(write=False); self.error.setflags(write=False)
        self.alignment = alignment
        if self.offset.shape!=(3,) or not np.isfinite(self.offset).all() or not callable(source_valid):
            raise ValueError('live source binding and finite launch transform required')
        self.transition = None
        if contact_transition is not None:
            if not isinstance(contact_transition, ContactTransition):
                raise ValueError('typed launch transition required')
            center = np.array(contact_transition.pad_center_xy_m)
            width = contact_transition.pad_half_width_m
            if np.any(center - width < reference.lower_xy_m) or np.any(center + width > reference.upper_xy_m):
                raise ValueError('reviewed support pad outside isolated floor')
            self.transition = _ContactSession(contact_transition, collision_vertices, support_vertices,
                                              reference.contact_height_m)

    def check_output(self, mission, vehicle, inputs, output, *, phase, now_ns=None, wall_ns=None):
        """Validate exact output with its originating phase, even across go transition.

        This is geometry only; it grants no authority for LAND or any command.
        The owner retains all command admission and deadline checks.
        """
        if phase not in ('wait','arm','climb','hover'):
            return 'launch_reference_has_no_route_authority'
        if output.phase == 'land':
            return 'launch_reference_has_no_landing_authority'
        if not output.stream:
            return 'invalid_launch_output' if self.transition is not None and getattr(output, 'commands', ()) else None
        if output.setpoint is None or len(output.setpoint)!=4:
            return 'invalid_launch_output'
        return self._check(mission, vehicle, inputs, phase, output.setpoint, now_ns, wall_ns, commit=True)

    def check_owner(self, mission, vehicle, inputs, *, now_ns=None, wall_ns=None):
        return self._check(mission, vehicle, inputs, mission.phase, mission.sp, now_ns, wall_ns, commit=False)

    def freshness_reason(self, vehicle, inputs, *, now_ns, wall_ns):
        if self.transition is None:
            return None
        reason = self.transition.freshness_reason(vehicle, inputs, now_ns, wall_ns)
        return self.transition.revoke(reason) if reason else None

    def _check(self, mission, vehicle, inputs, phase, setpoint, now_ns, wall_ns, *, commit):
        try:
            reason = self._geometry_reason(mission, vehicle, inputs, phase, setpoint, now_ns, wall_ns, commit=commit)
        except (ValueError,TypeError,AttributeError,KeyError,OSError,ArithmeticError):
            reason = 'launch_binding_invalid'
        if reason and self.transition is not None:
            return self.transition.revoke(reason)
        return reason

    def _geometry_reason(self, mission, vehicle, inputs, phase, setpoint, now_ns, wall_ns, *, commit):
        if phase not in ('wait','arm','climb','hover'):
            return 'launch_reference_has_no_route_authority'
        try:
            if self.source_valid() is not True: return 'launch_source_changed'
            if inputs is None or vehicle.pos is None: return 'launch_state_unavailable'
            alignment=inputs['alignment']
            if alignment.map.frame_id!=self.reference.frame_id or alignment.map.epoch!=self.reference.epoch:
                return 'launch_reference_mismatch'
            if (not np.allclose(rot_z(alignment.map_from_enu_yaw_rad)@R_ENU_NED,self.transform,atol=1e-10,rtol=0)
                    or not np.array_equal(np.asarray(alignment.ned_origin_in_map_m),self.offset)
                    or (self.alignment is not None and any(getattr(alignment,k)!=getattr(self.alignment,k)
                        for k in ('map','ned','provenance','position_error_m','angle_error_rad')))):
                return 'launch_transform_changed'
            # Preserve tracking and alignment uncertainty in every direction.
            bounds=inputs['vehicle']
            start=self.transform@np.asarray(vehicle.pos)+self.offset
            home=np.asarray(mission.home[:3] if mission.home is not None else vehicle.pos,dtype=float)
            command=np.asarray(setpoint[:3] if setpoint is not None else home,dtype=float)
            if not np.isfinite((start, home, command)).all():
                return 'invalid_launch_position'
            if not np.array_equal(command[:2],home[:2]):
                return 'launch_command_outside_home_column'
            if (self.transition is not None and setpoint is not None and mission.home is not None
                    and (not math.isfinite(setpoint[3]) or setpoint[3] != mission.home[3])):
                return 'launch_transition_yaw_command_changed'
            target=home.copy()
            target[2]=min(command[2],home[2]-mission.p.height_m)
            end=self.transform@target+self.offset
            allowance=bounds.tracking_error_m+alignment.position_error_m
            allowance+=2*math.sin(alignment.angle_error_rad/2)*max(np.linalg.norm(vehicle.pos),np.linalg.norm(target))
            error=self.error+allowance
            mapped_command = self.transform @ command + self.offset
            if self.transition is not None:
                return self.transition.check(self.reference, mission, vehicle, inputs, phase, start, mapped_command,
                                             end, error, self.transform, now_ns, wall_ns, commit=commit)
            # Validate the actual held command as well as the complete climb.
            # A downward correction is not covered by this launch contract.
            for finish in (mapped_command,end):
                reason=self.reference.check(start,finish,error_m=error,
                frame_id=alignment.map.frame_id,epoch=alignment.map.epoch,
                    source_sha256=self.reference.source_sha256,phase=phase,
                landed=vehicle.landed,contact_confirmed=vehicle.landed is True)
                if reason is not None: return reason
            return None
        except (ValueError,TypeError,AttributeError,KeyError,OSError):
            return 'launch_binding_invalid'
