"""Independent decision replay and raw PX4 command correlation. Never publishes."""
import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sqlite3

from refnav.dynamics import DroneState, LocalNED
from refnav.model_flight import ModelSelector


def audit(rows, raw_commands, config):
    previous='0'*64; decisions={}; receipts={}; faults=[]; differences=0
    selector=ModelSelector(config,monotonic=lambda:0.)
    for source in rows:
        row=dict(source); identity=row.pop('sha256')
        raw=json.dumps(row,sort_keys=True,separators=(',',':'),allow_nan=False)
        if row['previous_sha256']!=previous or sha256(raw.encode()).hexdigest()!=identity:
            faults.append('decision_log_hash_chain')
        previous=identity
        if row['kind']=='abort':faults.append('model_abort')
        if row['kind']=='decision':
            number=row['sequence']
            if number in decisions or number!=len(decisions)+1: faults.append('decision_sequence')
            if row['model_sha256']!=config['checkpoint_sha256'] or row['fallback'] or row['registration_enabled']:
                faults.append('model_or_authority_identity')
            state=dict(row['state']);state['reference']=LocalNED(**state['reference'])
            target,replayed=selector.choose(DroneState(**state),row['goal_ned_m'],row['decision_ros_ns'])
            if row['selected']!=replayed['selected']:faults.append('model_selection_replay')
            if row['goal_ned_m']!=config['waypoints_ned_m'][row['waypoint']]:faults.append('goal_not_in_profile')
            for original,again in zip(row['candidates'],replayed['candidates']):
                if original['candidate_id']!=again['candidate_id'] or any(
                    not math.isclose(original[k]['score'],again[k]['score'],rel_tol=1e-6,abs_tol=1e-7)
                    for k in ('pinn','d0')):faults.append('candidate_score_replay')
            if len(row['candidates'])!=len(replayed['candidates']):faults.append('candidate_count')
            decisions[number]=(row,target)
            differences+=row['selected']!=row['d0_selected']
        if row['kind']=='published':
            number=row['sequence']
            if number in receipts or number not in decisions:faults.append('unbound_publication');continue
            decision,target=decisions[number]
            if (math.dist(row['position_ned_m'],target.position_m)>1e-9 or abs(row['yaw_rad']-target.yaw_rad)>1e-9
                or not 0<=row['stamp_ns']-decision['decision_ros_ns']<=50_000_000
                or row['model_sha256']!=config['checkpoint_sha256']):faults.append('published_command_differs')
            receipts[number]=row
    if set(decisions)!=set(receipts):faults.append('unpublished_model_decision')
    position_commands=[r for r in raw_commands if all(math.isfinite(x) for x in r['position'])]
    if len(position_commands)!=len(receipts):faults.append('raw_command_count')
    for command,receipt in zip(position_commands,receipts.values()):
        if (command['timestamp']!=receipt['px4_issue_us']
            or math.dist(command['position'],receipt['position_ned_m'])>1e-5
            or abs(command['yaw']-receipt['yaw_rad'])>1e-5
            or any(math.isfinite(x) for x in command['velocity'])):faults.append('raw_command_mismatch')
    if receipts:
        first=min(r['px4_issue_us'] for r in receipts.values());last=max(r['px4_issue_us'] for r in receipts.values())
        if any(first<=r['timestamp']<=last and not all(math.isfinite(x) for x in r['position']) for r in raw_commands):
            faults.append('nonmodel_command_during_model_authority')
    if len(receipts)<10:faults.append('insufficient_model_commands')
    return dict(model_command_proof='PASS' if not faults else 'FAIL',faults=faults,
                selected_commands=len(receipts),pinn_d0_different_selections=differences,
                interpretation='Model selected and raw PX4 position commands match; not proof of superior flight performance')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',type=Path,required=True);args=parser.parse_args()
    from rclpy.serialization import deserialize_message
    from px4_msgs.msg import TrajectorySetpoint
    root=args.evidence;bag=next((root/'observations').glob('*.db3'))
    connection=sqlite3.connect('file:'+str(bag)+'?mode=ro',uri=True)
    topic=connection.execute("select id from topics where name='/fmu/in/trajectory_setpoint'").fetchone()[0]
    commands=[]
    for data, in connection.execute('select data from messages where topic_id=? order by timestamp,id',(topic,)):
        msg=deserialize_message(data,TrajectorySetpoint)
        commands.append(dict(timestamp=msg.timestamp,position=list(msg.position),velocity=list(msg.velocity),yaw=msg.yaw))
    config=json.loads((root/'calibration_profile.json').read_text())['model_flight']
    rows=[json.loads(line) for line in (root/'model_owner.decisions.jsonl').read_text().splitlines()]
    report=audit(rows,commands,config)
    owner=json.loads((root/'model_owner.json').read_text())
    recorder=json.loads((root/'calibration_recording.json').read_text())
    bag_gate=json.loads((root/'recording_final.json').read_text())
    report['recording_complete']=recorder.get('recording_complete') is True and not recorder.get('recording_faults') and recorder.get('pending_clock_samples')==0 and bag_gate.get('passed') is True
    report['flight_pass']=(report['model_command_proof']=='PASS' and report['recording_complete']
        and owner['result']=='complete' and owner['waypoints_reached']==len(config['waypoints_ned_m'])
        and owner['model_command_count']==report['selected_commands']
        and owner['max_observed_speed_mps']<=.50 and owner['native_disarm_ns'] is not None)
    with (root/'model-command-audit.json').open('x') as out:json.dump(report,out,indent=2)
    print(json.dumps(report));return int(not report['flight_pass'])


if __name__=='__main__':raise SystemExit(main())
