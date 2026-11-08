"""Summarize a recorded sensor-aligned window flight using independent Gazebo positions.

SIMULATOR ANALYSIS ONLY. Usage: python plot_window_flight.py RUN_FOLDER OUTPUT_FOLDER.
Exports the plan, referee result, height table, alignment observations, and a route plot.
It does not feed truth back into flight control.
"""
import csv
import json
import shutil
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from compare_astar_runs import draw_map
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'drone_code/team_nav'))
from team_nav.course_map import load_course_map
from team_nav.practice_flights.window_route import window_geometry


def main():
    run, output = map(Path,sys.argv[1:3]);output.mkdir(parents=True,exist_ok=True)
    root = Path(__file__).resolve().parents[2]
    course = json.loads((root/'drone_code/team_nav/course_maps/real_course_2026-11-08.json').read_text())
    flight = next(run.glob('course_run_records/*practice_practice_corrected_course'))
    plan = json.loads((flight/'astar_plan.json').read_text())
    record = next(run.glob('referee/flight_*.csv'))
    summary = json.loads(record.with_name(record.stem+'_summary.json').read_text())
    with record.open() as source:
        rows = [row for row in csv.DictReader(source) if row['armed']=='True']
    actual = np.array([[float(row['course_x_metres']),float(row['course_y_metres']),float(row['height_metres'])] for row in rows])
    points = [plan['start_metres']]
    for stage in plan['stages']:
        points.extend(stage['points'])
    points = np.array(points)
    events = [json.loads(line) for line in (flight/'events.jsonl').read_text().splitlines()]
    alignment = [event for event in events if event['kind'].startswith('window_')]
    (output/'alignment_events.json').write_text(json.dumps(alignment,indent=2)+'\n')
    (output/'referee_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    for name in ['astar_plan.json','waypoints_with_heights.csv']:
        shutil.copyfile(flight/name,output/name)
    figure,(axis,height_axis) = plt.subplots(1,2,figsize=(13,6),dpi=160)
    draw_map(axis,course)
    axis.plot(points[:,0],points[:,1],'--o',ms=3,color='#147f9b',label='Planned route')
    axis.plot(actual[:,0],actual[:,1],color='#263b4b',lw=1,label='Recorded flight')
    for number,stage in enumerate(plan['stages'],1):
        point = stage['points'][-1]
        axis.annotate(str(number),point[:2],xytext=(5,5),textcoords='offset points',weight='bold')
    wall = next(wall for wall in course['walls'] if wall['openings']);opening=wall['openings'][0]
    start,end = np.array(wall['start_metres']),np.array(wall['end_metres'])
    along = (end-start)/np.linalg.norm(end-start);normal=np.array([-along[1],along[0]])
    centre = start+along*(opening['start_along_wall_metres']+opening['end_along_wall_metres'])/2
    axis.scatter(*centre,marker='s',s=50,color='#cf7334',label='Window at 1.363 m')
    axis.set(xlim=(-3.2,3.2),ylim=(-3.2,3.2),xlabel='East (m)',ylabel='North (m)',
             title=f"{len(summary['checkpoint_seconds'])}/4 checkpoints; contacts: {summary['touches']}; pad: {summary['landed_on_pad']}")
    axis.set_aspect('equal');axis.legend(fontsize=8,loc='lower left')
    offset = actual[:,:2]-centre
    near = (abs(offset@normal)<.65)&(abs(offset@along)<.40)&(actual[:,2]>.9)
    near_frame = (abs(offset@normal)<.30)&(abs(offset@along)<.30)&(actual[:,2]>1.)
    clearance = []
    map_object = load_course_map(root/'drone_code/team_nav/course_maps/real_course_2026-11-08.json')
    headings = np.radians([float(row['heading_degrees']) for row in rows])
    for height in np.unique(np.round(actual[near_frame,2],2)):
        selected = near_frame&(np.round(actual[:,2],2)==height)
        geometry = window_geometry(map_object,height)
        clearance.extend(geometry.footprint_clearance(actual[selected,:2],headings[selected]).tolist())
    through = (offset@normal)[near_frame]
    crossed = len(through)>1 and through[0]<-.25 and through[-1]>.25
    metrics = {'window_alignment_confirmed':any(event['kind']=='window_aligned' for event in alignment),
               'window_plane_crossed':bool(crossed),
               'minimum_window_airframe_clearance_metres':min(clearance) if clearance else None,
               'recorded_window_height_range_metres': [float(actual[near_frame,2].min()),float(actual[near_frame,2].max())] if near_frame.any() else None}
    with (output/'recorded_path.csv').open('w',newline='') as destination:
        writer = csv.DictWriter(destination,fieldnames=['seconds','course_x_metres','course_y_metres','height_metres','heading_degrees'])
        writer.writeheader()
        writer.writerows({key:row[key] for key in writer.fieldnames} for row in rows)
    (output/'window_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    height_axis.plot((offset@normal)[near],actual[near,2],color='#263b4b',label='Actual height near window')
    for height,label in [(opening['bottom_height_metres'],'Sill'),(opening['top_height_metres'],'Header')]:
        height_axis.axhline(height,color='#657584',ls=':',label=f'{label}: {height:.3f} m')
    height_axis.axhline(1.3626,color='#147f9b',ls='--',label='Target: 1.363 m')
    height_axis.axvspan(-wall['thickness_metres']/2,wall['thickness_metres']/2,color='#cf7334',alpha=.15,label='Wall thickness')
    height_axis.set(xlim=(-.65,.65),ylim=(.9,1.8),xlabel='Distance through window plane (m)',ylabel='Height above floor (m)',title='Independent height trace at the window')
    height_axis.legend(fontsize=8);height_axis.grid(alpha=.15)
    figure.tight_layout();figure.savefig(output/'flight_and_window_height.png')


if __name__=='__main__':
    main()
