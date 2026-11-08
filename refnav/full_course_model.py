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
        return dict(score=score,eligible=allowed,peak_speed_mps=peak,endpoint_ned_m=last.position_m,
            endpoint_velocity_mps=last.velocity_mps,reason='eligible' if allowed else 'map_clearance_height_or_speed',
            map_margin_m=margin,trajectory_ned_m=[p.state.position_m for p in predictions])


class FullCourseFlight(ModelFlight):
    def __init__(self,plan,config,log,*,selector=None):
        super().__init__(plan,config,log,selector=selector or FullCourseSelector(config))

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
