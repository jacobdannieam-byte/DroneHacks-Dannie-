"""Full Classical-course goals with PINN-selected commands, no route-follower fallback."""
from dataclasses import dataclass, asdict
import math
from pathlib import Path
import sys

import numpy as np

from refnav.calibration_trial import CalibrationPlan, CalibrationTrial
from refnav.model_flight import ModelFlight, ModelSelector, digest


@dataclass(frozen=True)
class FullCoursePlan(CalibrationPlan):
    def __post_init__(self):
        # Retain calibration lifecycle limits; only course extent and total budget
        # differ for the separately authorized full-course mission.
        CalibrationPlan(**dict(asdict(self), minimum_offset_m=(-.5,-.5,-1.05),
            maximum_offset_m=(4.,.5,.02),transit_timeout_s=20.,trial_ros_limit_s=60.,trial_wall_limit_s=120.))
        for key in ('origin_ned_m','minimum_offset_m','maximum_offset_m'):
            value=tuple(getattr(self,key))
            if len(value)!=3 or not all(math.isfinite(x) for x in value):raise ValueError('Invalid full-course bounds')
            object.__setattr__(self,key,value)
        if (not all(-8<=a<0<b<=8 for a,b in zip(self.minimum_offset_m,self.maximum_offset_m))
            or (self.transit_timeout_s,self.trial_ros_limit_s,self.trial_wall_limit_s)!=(360.,420.,600.)):
            raise ValueError('Full-course extent or deadline invalid')


def course_and_route(config):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'drone_code/team_nav'))
    from team_nav.course_map import load_course_map
    from team_nav.mission import load_mission
    from team_nav.route_planner import plan_route
    for key in ('participant_map','mission','other_boulder_map'):
        if digest(config[key])!=config[key+'_sha256']:raise ValueError('Full-course source identity mismatch: '+key)
    course=load_course_map(config['participant_map'])
    route=plan_route(course,load_mission(config['mission']),other_possible_boulders=load_course_map(config['other_boulder_map']).low_obstacles)
    launch=course.launch_mark_centre_metres
    goals=[(float(p[1]-launch[1]),float(p[0]-launch[0]),-float(p[2])) for p in route.waypoints_metres[1:]]
    if not np.allclose(goals,config['waypoints_ned_m'],atol=1e-10,rtol=0):raise ValueError('Goals do not match complete Classical route')
    return course,route


class FullCourseSelector(ModelSelector):
    def __init__(self, config, **kwargs):
        if config.get('registration_enabled') is not False or config.get('fallback_allowed') is not False:
            raise ValueError('Registration and fallback must remain disabled')
        super().__init__(config,**kwargs)
        self.course,self.route=course_and_route(config)
        from team_nav.course_map import shapes_blocking_between_heights, distance_to_shapes_metres
        self.distance=distance_to_shapes_metres
        self.blocking_shapes=shapes_blocking_between_heights
        # Entire approved navigation-height band plus the Classical body/slab margin.
        self.shapes=shapes_blocking_between_heights(self.course,.45,1.15)
        self.require_depth = False
        self.depth_admission = None

    def margin(self, points):
        points=np.asarray(points,dtype=float)
        xy=points[:,[1,0]]+self.course.launch_mark_centre_metres
        shapes=self.blocking_shapes(self.course,float((-points[:,2]).min())-.20,float((-points[:,2]).max())+.20)
        distances,_=self.distance(xy,shapes)
        return float(min(distances))-.35

    def evaluate(self,model,state,target,goal):
        predictions=model.rollout(state,target,dt=.02,steps=30)
        path=np.array([state.position_m]+[p.state.position_m for p in predictions])
        # Check both curved prediction and entire held-target displacement.
        segments=[*zip(path,path[1:]),(np.array(state.position_m),np.array(target.position_m))]
        samples=np.concatenate([np.linspace(a,b,max(2,math.ceil(float(np.linalg.norm(b-a))/.002)+1)) for a,b in segments])
        margin=self.margin(samples)-.001  # Lipschitz cover bound for <=2 mm sample spacing.
        inside=bool(np.all(samples>=self.config['command_low_ned_m']) and np.all(samples<=self.config['command_high_ned_m']))
        peak=max(math.hypot(*state.velocity_mps),*(math.hypot(*p.state.velocity_mps) for p in predictions))
        last=predictions[-1].state
        score=math.dist(last.position_m,goal)+.15*math.hypot(*last.velocity_mps)
        allowed=inside and margin>=0 and peak<=.35
        if self.require_depth:
            evidence = self.depth_admission
            observed = False
            if evidence is not None and evidence.get('allowed'):
                volume=evidence['admitted_center_volume'];direction=np.asarray(volume['direction'])
                lateral=np.array([-direction[1],direction[0],0.]);offset=samples-volume['origin']
                along=offset@direction
                observed=bool(np.all(along>=-.01) and np.all(along<=volume['forward_limit_m'])
                    and np.all(np.abs(offset@lateral)<=volume['lateral_limit_m'])
                    and np.all(np.abs(offset[:,2])<=volume['vertical_limit_m']))
            allowed = allowed and observed
        return dict(score=score,eligible=allowed,peak_speed_mps=peak,endpoint_ned_m=last.position_m,
            endpoint_velocity_mps=last.velocity_mps,reason='eligible' if allowed else 'map_clearance_height_or_speed',
            map_margin_m=margin,trajectory_ned_m=[p.state.position_m for p in predictions])


class FullCourseFlight(ModelFlight):
    def __init__(self,plan,config,log,*,selector=None):
        super().__init__(plan,config,log,selector=selector or FullCourseSelector(config))
        from refnav.section_policy import SectionPolicy
        self.section_policy = SectionPolicy(self.selector.route.checkpoints_cleared)
        self.selector.require_depth = True
        self.section_wait = None
        self.section_result = None
        self.turn_wait = None
        self.policy_hold = None
        self.prepared_section = None
        self.prepared_key = None

    def prepare_section(self,obs,ros_ns):
        """Read-only perception before the native command/publication timing fence.

        Decisions still check capture age and the current predicted path against
        the fixed observed volume; preparation never grants command authority.
        """
        if self.phase!='transit' or self.waypoint>=len(self.config['waypoints_ned_m']):
            return
        capture=self.section_policy.observation
        key=(self.waypoint,None if capture is None else capture.observation_id)
        if key!=self.prepared_key:
            self.prepared_section=self.section_policy.assess(self.waypoint,np.array(obs.position),
                self.config['waypoints_ned_m'][self.waypoint],ros_ns/1e9)
            self.prepared_key=key

    def hold_for_section(self,obs,ros_ns,reason,yaw_rate=None):
        from refnav.calibration_trial import Intent, ZERO
        self.policy_hold = dict(reason=reason,decision_ros_ns=ros_ns,position_ned_m=obs.position,
                                velocity_ned_mps=obs.velocity,heading_rad=obs.heading_rad,
                                yaw_rate_rad_s=yaw_rate,waypoint=self.waypoint)
        return self._issue(Intent(ZERO,yaw_rate_rad_s=yaw_rate),ros_ns)

    def admit_goal(self,obs,goal,ros_ns,wall_ns):
        from refnav.calibration_trial import Intent, ZERO
        heading = math.atan2(goal[1]-obs.position[1],goal[0]-obs.position[0])
        error = math.atan2(math.sin(heading-obs.heading_rad),math.cos(heading-obs.heading_rad))
        if math.dist(obs.position,goal)>.06 and abs(error)>.10:
            self._progress = None
            if self.turn_wait is None:
                self.turn_wait = ros_ns,wall_ns
            if max(ros_ns-self.turn_wait[0],wall_ns-self.turn_wait[1])>=15_000_000_000:
                return self._land('section_turn_timeout',ros_ns,wall_ns)
            capture = self.section_policy.observation
            if capture is None or not 0<=ros_ns/1e9-capture.stamp<=.25 or self.section_policy.fault:
                return self._land('section_turn_depth_unavailable',ros_ns,wall_ns)
            # Map-assisted experiment only: the existing circular body reservation
            # covers every yaw. This is not a claim of observed rear clearance.
            yaw = None
            if math.hypot(*obs.velocity)<=.05 and self.selector.margin([obs.position])>=0:
                yaw = math.copysign(.25,error)
            return self.hold_for_section(obs,ros_ns,'bounded_heading_alignment',yaw)
        self.turn_wait = None
        report = self.prepared_section
        if (report is None or report['waypoint']!=self.waypoint or self.section_policy.fault
            or not 0<=ros_ns/1e9-report.get('capture_s',-1)<=.25):
            report=dict(section='unconfirmed',waypoint=self.waypoint,allowed=False,
                        reason=self.section_policy.fault or 'await_fresh_prepared_depth',registration_enabled=False)
        self.section_result = report
        self.selector.depth_admission = report
        self.log.write(dict(kind='section_check',ros_ns=ros_ns,**report))
        if report['allowed']:
            self.section_wait = None
            return None
        self._progress = None
        if self.section_wait is None:
            self.section_wait = ros_ns,wall_ns
        if max(ros_ns-self.section_wait[0],wall_ns-self.section_wait[1])>=3_000_000_000:
            return self._land('section_depth_refused:'+report['reason'],ros_ns,wall_ns)
        return self.hold_for_section(obs,ros_ns,report['reason'])

    def published(self,intent,stamp_ns,px4_issue_us,*,wall_ns=None):
        result = super().published(intent,stamp_ns,px4_issue_us,wall_ns=wall_ns)
        if self.policy_hold is not None:
            self.log.write(dict(kind='section_hold_published',**self.policy_hold,
                                stamp_ns=stamp_ns,px4_issue_us=px4_issue_us))
            self.policy_hold = None
        return result

    def step(self,ros_ns,wall_ns,observation,*,admitted):
        if observation is not None and self.phase in ('climb','transit','brake'):
            if self.selector.margin([observation.position])<0:
                self._operator_land=True
        return CalibrationTrial.step(self,ros_ns,wall_ns,observation,admitted=admitted)

    def transit(self,obs,offset,vertical_target,ros_ns,wall_ns):
        distance=math.dist(obs.position,self.config['waypoints_ned_m'][self.waypoint])
        old=getattr(self,'_progress',None)
        if old is None or old[0]!=self.waypoint or distance<old[1]-.02:
            self._progress=(self.waypoint,distance,ros_ns,wall_ns)
        elif distance>.06 and max(ros_ns-old[2],wall_ns-old[3])>4_000_000_000:
            self.log.write(dict(kind='abort',reason='model_no_progress',ros_ns=ros_ns,fallback=False))
            return self._land('model_no_progress',ros_ns,wall_ns)
        return super().transit(obs,offset,vertical_target,ros_ns,wall_ns)
