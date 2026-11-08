"""Draw two recorded A* Gazebo trials and export their waypoint-height tables.

SIMULATOR ANALYSIS ONLY. Reads saved referee/camera-run files; never controls a drone.
Usage: python compare_astar_runs.py BASELINE_RUN MOVED_RUN OUTPUT_FOLDER
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
from matplotlib.patches import Circle, Polygon, Rectangle

ROOT = Path(__file__).resolve().parents[2]


def draw_map(axis, course):
    axis.add_patch(Rectangle((-3.048,-3.048),6.096,6.096,fill=False,color='#465665'))
    for wall in course['walls']:
        start,end = np.array(wall['start_metres']),np.array(wall['end_metres'])
        along = end-start
        normal = np.array([-along[1],along[0]])/np.linalg.norm(along)*wall['thickness_metres']/2
        axis.add_patch(Polygon([start+normal,end+normal,end-normal,start-normal],color='#657584'))
    for obstacle in course['low_obstacles']:
        color = '#d56d41' if obstacle['name']=='Sketch boulder north-west' else '#c4ab82'
        axis.add_patch(Circle(obstacle['centre_metres'],obstacle['radius_metres'],color=color,alpha=.8))
    for tube in course['tubes']:
        start,end = np.array(tube['entrance_centre_metres']),np.array(tube['exit_centre_metres'])
        along = end-start
        normal = np.array([-along[1],along[0]])/np.linalg.norm(along)*tube['outside_width_metres']/2
        axis.add_patch(Polygon([start+normal,end+normal,end-normal,start-normal],color='#92b5ca',alpha=.25,hatch='///'))
    pad = course['landing_pad']
    axis.add_patch(Circle(pad['centre_metres'],pad['outer_radius_metres'],fc='#d4e9d7',ec='#4d8f5a'))
    for name in ['end of first corridor','Gate A','tunnel exit','Gate B']:
        line = next(line for line in course['checkpoint_lines'] if line['name']==name)
        points = np.array([line['start_metres'],line['end_metres']])
        axis.plot(points[:,0],points[:,1],':',color='#8b61a8',lw=1.5)


def draw_run(axis, folder, output, label, color):
    record = next(folder.glob('referee/flight_*.csv'))
    summary = json.loads(record.with_name(record.stem+'_summary.json').read_text())
    course = json.loads((ROOT/summary['world_map_file'].removeprefix('/repository/')).read_text())
    flight = next(folder.glob('course_run_records/*practice_practice_astar_five_waypoints'))
    plan = json.loads((flight/'astar_plan.json').read_text())
    draw_map(axis,course)
    points = [plan['start_metres']]
    for number,stage in enumerate(plan['stages'],1):
        points.extend(stage['points'])
        end = stage['points'][-1]
        axis.scatter(end[0],end[1],s=210,c=color,edgecolors='white',zorder=6)
        axis.text(end[0],end[1],str(number),ha='center',va='center',color='white',weight='bold',zorder=7)
    points = np.array(points)
    axis.plot(points[:,0],points[:,1],'--o',color=color,ms=3,lw=1.5,label='A* planned waypoints')
    rows = [row for row in csv.DictReader(record.open()) if row['armed']=='True']
    actual = np.array([[float(row['course_x_metres']),float(row['course_y_metres'])] for row in rows])
    if len(actual):
        axis.plot(actual[:,0],actual[:,1],color='#263b4b',alpha=.75,lw=1,label='Recorded flight')
    axis.set(xlim=(-3.2,3.2),ylim=(-3.2,3.2),xlabel='East (m)',ylabel='North (m)')
    axis.set_aspect('equal');axis.grid(alpha=.15)
    axis.set_title(f"{label}\n{len(summary['checkpoint_seconds'])}/4 checkpoints; pad: {summary['landed_on_pad']}; contacts: {summary['touches']}")
    axis.legend(loc='lower left',fontsize=8)
    stem = 'baseline' if label.startswith('Baseline') else 'moved'
    shutil.copyfile(flight/'waypoints_with_heights.csv',output/f'{stem}_waypoints_with_heights.csv')
    shutil.copyfile(flight/'astar_plan.json',output/f'{stem}_astar_plan.json')
    (output/f'{stem}_referee.json').write_text(json.dumps(summary,indent=2)+'\n')


def main():
    baseline,moved,output = map(Path,sys.argv[1:4])
    output.mkdir(parents=True,exist_ok=True)
    figure,axes = plt.subplots(1,2,figsize=(13,7),dpi=160)
    draw_run(axes[0],baseline,output,'Baseline: original boulder','#147f9b')
    draw_run(axes[1],moved,output,'Moved: boulder 0.45 m south','#cf7334')
    figure.suptitle('Same five goals; A* changes the intermediate route',fontsize=17)
    figure.text(.5,.025,'All flight waypoints: 0.75 m. Touchdown: 0 m. Both cases use the supplied map; this is not an unknown-obstacle replanning test.',ha='center',fontsize=9)
    figure.tight_layout(rect=(0,.10,1,.94))
    figure.savefig(output/'route_comparison.png')
    figure.savefig(output/'route_comparison.svg')
    # Matplotlib puts trailing spaces in SVG paths; keep generated Git diffs clean.
    svg = output/'route_comparison.svg'
    svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')


if __name__=='__main__':
    main()
