"""Depth evidence for local section traversal. No registration or simulator truth.

This conservative forward-volume guard supplements the existing map/body guard;
it does not certify unobserved rear space or authorize an observation rotation.
"""
import math
import numpy as np
from refnav.world_model.observations import Config, validate, project
from refnav.world_model.geometry import _planes


class SectionPolicy:
    def __init__(self, checkpoints):
        self.sections = [(str(name), int(index)-1) for name, _, index in checkpoints]
        if not self.sections or any(i < 0 for _, i in self.sections):
            raise ValueError('section checkpoints required')
        self.observation = None
        self.frame_identity = None
        self.fault = None
        self.last_accepted = None
        self.confirmations = 0
        self.key = None
        self.config = Config(max_age_s=.25, pose_sync_s=.03)

    def offer(self, observation):
        identity = observation.frame_id, observation.epoch, observation.clock_id
        if self.frame_identity is not None and identity != self.frame_identity:
            self.fault = 'section_reference_changed'
        if self.observation is not None and observation.stamp <= self.observation.stamp:
            self.fault = 'section_depth_rewind_or_duplicate'
        self.frame_identity = identity
        self.observation = observation

    def assess(self, waypoint, position, goal, now):
        section = next((n for n, end in self.sections if waypoint <= end), 'landing')
        key = section, waypoint
        if key != self.key:
            self.key, self.confirmations, self.last_accepted = key, 0, None
        result = dict(section=section, waypoint=waypoint, state='OBSERVE', allowed=False,
                      registration_enabled=False, reason='missing_depth')
        if self.fault:
            return dict(result, reason=self.fault)
        obs = self.observation
        if obs is None:
            return result
        reason = validate(obs, now, self.config)
        if reason:
            self.confirmations = 0
            return dict(result, reason=reason)
        direction = np.asarray(goal)-position
        direction[2] = 0
        length = np.linalg.norm(direction)
        if length < .02:
            direction = np.asarray(obs.R_world_body)[:, 0].copy()
            direction[2] = 0
        direction /= max(np.linalg.norm(direction), 1e-12)
        lateral = np.array([-direction[1], direction[0], 0.])
        # Existing .30 m body + .05 m lateral design reserve; extra .05 m
        # observation/discretization reserve is conservative, not calibrated sigma.
        stations = np.arange(.35, max(.8, min(length+.35, 1.2))+.001, .05)
        reserve = .05 + 3*obs.position_sigma_m + 1.2*obs.angle_sigma_rad
        half_width, half_height = .35+reserve+.10, .20+reserve+.05
        offsets = np.array([(x, y, z) for x in stations
                            for y in np.linspace(-half_width,half_width,math.ceil(2*half_width/.05)+1)
                            for z in np.linspace(-half_height,half_height,math.ceil(2*half_height/.05)+1)])
        world = np.asarray(position) + offsets[:,0,None]*direction + offsets[:,1,None]*lateral
        world[:,2] += offsets[:,2]
        rotation = np.asarray(obs.R_world_body) @ np.asarray(obs.R_body_cam)
        origin = np.asarray(obs.t_world_body) + np.asarray(obs.R_world_body) @ obs.t_body_cam
        optical = (world-origin) @ rotation
        fx, fy, cx, cy = obs.K
        z = optical[:,2]
        u = np.rint(fx*optical[:,0]/np.maximum(z,1e-9)+cx).astype(int)
        v = np.rint(fy*optical[:,1]/np.maximum(z,1e-9)+cy).astype(int)
        image = np.asarray(obs.depth_m)
        inside = (z>.1)&(u>=1)&(u<image.shape[1]-1)&(v>=1)&(v<image.shape[0]-1)
        clear = np.zeros(len(world), dtype=bool)
        indices = np.flatnonzero(inside)
        if len(indices):
            neighbours = np.stack([image[v[indices]+dy,u[indices]+dx] for dy in (-1,0,1) for dx in (-1,0,1)])
            clear[indices] = np.all(np.isfinite(neighbours)&(neighbours>z[indices]+.05)&(neighbours<self.config.max_depth_m),axis=0)
        coverage = float(clear.mean())
        result.update(observation_id=obs.observation_id, capture_s=obs.stamp,
                      forward_volume_coverage=coverage, sampled_volume_points=len(world),
                      observation_reserve_m=reserve, uncertainty_basis='3sigma_reported_pose_plus_design_reserve_not_hardware_bound')
        result['admitted_center_volume'] = dict(origin=np.asarray(position).tolist(),direction=direction.tolist(),
            lateral_limit_m=.10,vertical_limit_m=.05,forward_limit_m=float(stations[-1]-.35-reserve))
        if not clear.all():
            self.confirmations = 0
            return dict(result, reason='swept_forward_volume_unknown_or_blocked')
        points = project(obs, self.config).points()-np.asarray(position)
        along, across = points@direction, points@lateral
        # Restrict fitting to flight-height returns, so floor/ceiling points on
        # either side cannot impersonate two corridor walls.
        walls = list(_planes(points[np.abs(points[:,2])<.35],self.config))
        side_support = []
        tunnel = 'tunnel' in section.lower()
        for low in (.35,.6):
            band = (along>=low)&(along<low+.3)
            sides=set()
            for normal,offset,hits in walls:
                if abs(normal@lateral)<.95:
                    continue
                supported=hits[((hits@direction)>=low)&((hits@direction)<low+.3)]
                if len(supported)>=5 and np.ptp(supported[:,2])>=.4:
                    side=float(np.median(supported@lateral))
                    if abs(side)>half_width:
                        sides.add(1 if side>0 else -1)
            side_support.append(sides=={-1,1})
            if tunnel:
                center = band&(np.abs(across)<.4)
                # NED: roof negative, floor positive. Curved surfaces supported.
                roof=center&(points[:,2]<-half_height)
                floor=center&(points[:,2]>half_height)
                supported = all(np.sum(surface&(across>=left)&(across<left+.2))>=3
                                for surface in (roof,floor) for left in (-.3,-.1,.1))
                if not supported:
                    self.confirmations = 0
                    return dict(result, reason='tunnel_floor_or_roof_unobserved')
        if ('corridor' in section.lower() or tunnel) and not all(side_support):
            self.confirmations = 0
            return dict(result, reason='passage_side_surfaces_unconfirmed')
        if obs.observation_id != self.last_accepted:
            self.confirmations += 1
            self.last_accepted = obs.observation_id
        return dict(result, allowed=self.confirmations>=3,
                    state='TRAVERSE' if self.confirmations>=3 else 'VALIDATE',
                    reason='observed_volume' if self.confirmations>=3 else 'await_independent_frames',
                    tunnel_checked=tunnel, confirmations=self.confirmations)
