import csv
import gzip
import json
import numpy as np
import pytest
from examples.prepare_real_pinn_data import prepare, yaw_rate, load


def write(path, rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    with gzip.open(path,'wt',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def test_body_rates_are_transformed_and_invalid_attitude_rejected():
    row={'pose_frame':'1',**{f'q[{i}]':v for i,v in enumerate([1,0,0,0])},
         **{f'angular_velocity[{i}]':v for i,v in enumerate([0,0,.2])}}
    assert yaw_rate(row)==pytest.approx(.2)
    row['q[0]']=2
    with pytest.raises(ValueError):yaw_rate(row)


def test_clock_rewind_is_quarantined_not_sorted(tmp_path):
    path=tmp_path/'data.gz'
    write(path,[{'timestamp':t,'received_unix_time_seconds':t/1e6} for t in (2000000,1000000)])
    values,rejected=load(path)
    assert not values and rejected==2


def test_complete_healthy_windows_export_with_provenance(tmp_path):
    source=tmp_path/'source';folder=source/'flight';folder.mkdir(parents=True)
    with gzip.open(folder/'session.json.gz','wt') as f:
        json.dump({'team_nav_overall_fingerprint':'abc','computer_name':'fixture'},f)
    streams={k:[] for k in ('vehicle_local_position','vehicle_status_v1','vehicle_land_detected','estimator_status_flags','vehicle_odometry','command')}
    for i in range(10):
        t=1000000+i*20000;base={'timestamp':t,'timestamp_sample':t,'received_unix_time_seconds':t/1e6}
        streams['vehicle_local_position'].append(dict(base,x=0,y=0,z=-.8,vx=0,vy=0,vz=0,heading=0,
            xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True,heading_good_for_control=True,dead_reckoning=False,xy_reset_counter=0))
        streams['vehicle_status_v1'].append(dict(base,arming_state=2,nav_state=14,failsafe=False))
        streams['vehicle_land_detected'].append(dict(base,landed=False,ground_contact=False))
        streams['estimator_status_flags'].append(dict(base,cs_ev_pos=True,cs_yaw_align=True))
        streams['vehicle_odometry'].append(dict(base,pose_frame=1,reset_counter=0,
            **{f'q[{j}]':v for j,v in enumerate([1,0,0,0])},**{f'angular_velocity[{j}]':0 for j in range(3)}))
        streams['command'].append(dict(base,yaw=0,**{f'position[{j}]':v for j,v in enumerate([0,0,-.8])}))
    for key,rows in streams.items():
        name='fmu__in__trajectory_setpoint' if key=='command' else 'fmu__out__'+key
        write(folder/'topics'/(name+'.csv.gz'),rows)
    output=tmp_path/'prepared';report=prepare(source,output)
    assert report['accepted_samples']>0 and not report['training_ready']
    arrays=np.load(next(output.glob('*.npz')))
    assert arrays['features'].shape[1]==17 and arrays['targets'].shape[1]==4
    assert np.isfinite(arrays['targets']).all()
    assert report['weights_changed'] is False
