"""Closed-loop synthetic regression, optionally through private ROS PX4 messages.

No flight commands or simulator start service. Private transport is not a flight.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import numpy as np

from examples.audit_model_flight import audit
from refnav.calibration_trial import CalibrationPlan, Observation, NS
from refnav.core import NAV_AUTO_LAND, NAV_OFFBOARD
from refnav.dynamics import DroneState, DynamicsModel, PositionSetpoint
from refnav.dynamics.physics import NominalParameters
from refnav.model_flight import DecisionLog, ModelFlight, REFERENCE


def run(output, tau, transport=None):
    output.mkdir(exist_ok=False,parents=True)
    raw=json.loads(Path('scenarios/model_flight/h1_pinn_prefix.json').read_text())
    log=DecisionLog(output/'decisions.jsonl');driver=ModelFlight(CalibrationPlan(**raw['plan']),raw['model_flight'],log)
    plant=DynamicsModel(NominalParameters(velocity_tau_s=tau))
    pos=np.zeros(3);velocity=np.zeros(3);armed=False;nav=0;commands=[];states=[]
    for i in range(1200):
        ns=NS+i*50_000_000
        obs=Observation(tuple(map(float,pos)),tuple(map(float,velocity)),ns,ns,armed,bool(pos[2]>=0.),nav,0.)
        intent=driver.step(ns,ns,obs,admitted=True)
        if driver.phase=='done':break
        if intent is not None:
            if intent.command=='offboard':nav=NAV_OFFBOARD
            if intent.command=='arm':armed=True
            if intent.command=='land':nav=NAV_AUTO_LAND
            if intent.position_ned_m is not None:
                row=dict(timestamp=ns//1000,position=list(intent.position_ned_m),velocity=[math.nan]*3,yaw=intent.yaw_rad)
                if transport is not None:row=transport(row)
                commands.append(row)
                target=PositionSetpoint(REFERENCE,ns/NS,ns/NS+.6,tuple(row['position']),row['yaw'])
                predicted=plant.predict(DroneState(REFERENCE,ns/NS,tuple(pos),tuple(velocity),0.,0.),target,dt=.05)
                pos=np.array(predicted.state.position_m);velocity=np.array(predicted.state.velocity_mps)
            elif intent.velocity_ned_mps is not None:
                desired=np.array(intent.velocity_ned_mps) if armed else np.zeros(3)
                velocity+=(desired-velocity)*min(1.,.05/tau)
                pos+=velocity*.05
            driver.published(intent,ns,ns//1000,wall_ns=ns)
        if nav==NAV_AUTO_LAND:
            velocity=np.array([0.,0.,.3]);pos+=velocity*.05
            if pos[2]>=0:pos[2]=0.;velocity*=0;armed=False
        states.append(dict(time_s=ns/NS,position=pos.tolist(),velocity=velocity.tolist(),phase=driver.phase))
    log.close()
    rows=[json.loads(line) for line in (output/'decisions.jsonl').read_text().splitlines()]
    proof=audit(rows,commands,raw['model_flight'])
    summary=dict(scope='synthetic plant regression; not actual Gazebo flight',tau_s=tau,result=driver.result,
        recovery_reason=driver.recovery_reason,waypoints_reached=driver.waypoint,**proof,
        peak_speed_mps=max(math.hypot(*r['velocity']) for r in states),
        decision_p95_ms=float(np.percentile([r['elapsed_s']*1000 for r in rows if r['kind']=='decision'],95)))
    (output/'states.json').write_text(json.dumps(states,indent=2))
    (output/'summary.json').write_text(json.dumps(summary,indent=2))
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ros-private-transport',action='store_true');args=parser.parse_args()
    node=None;transport=None
    if args.ros_private_transport:
        import rclpy
        from px4_msgs.msg import TrajectorySetpoint
        from onboard.diagnostics.sitl_graph import inspect
        rclpy.init();node=rclpy.create_node('model_command_private_test')
        topic='/qualification/model_fixture/trajectory_setpoint';received=[]
        node.create_subscription(TrajectorySetpoint,topic,received.append,10)
        pub=node.create_publisher(TrajectorySetpoint,topic,10)
        def transport(row):
            before=len(received)
            from examples.calibration_trial_owner import position_messages
            from refnav.calibration_trial import Intent
            mode,message=position_messages(Intent(position_ned_m=tuple(row['position']),yaw_rad=row['yaw']),row['timestamp'])
            assert mode.position and not mode.velocity
            pub.publish(message)
            end=time.monotonic()+2
            while len(received)==before and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
            if len(received)!=before+1:raise RuntimeError('Private ROS delivery failed')
            m=received[-1];return dict(timestamp=m.timestamp,position=list(m.position),velocity=list(m.velocity),yaw=m.yaw)
        deadline=time.monotonic()+3
        while pub.get_subscription_count()<1 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if inspect(node,strict_motion=True)['issues']:raise RuntimeError('Flight command graph is not idle')
    try:
        results=[run(args.output/f'tau-{tau}',tau,transport) for tau in (.2,.35,.6)]
        if node is not None:
            graph=inspect(node,strict_motion=True)
            (args.output/'ros-graph.json').write_text(json.dumps(graph,indent=2))
            assert not graph['issues']
        print(json.dumps(results,indent=2))
        return int(any(r['result']!='complete' or r['model_command_proof']!='PASS' for r in results))
    finally:
        if node is not None:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':raise SystemExit(main())
