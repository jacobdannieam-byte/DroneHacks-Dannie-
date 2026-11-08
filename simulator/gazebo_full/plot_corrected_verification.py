"""Plot a bounded Gazebo verification and two explicitly unflown window proposals.

Simulator analysis only. Usage: python plot_corrected_verification.py RUN_FOLDER OUTPUT_FOLDER.
Reads saved records; does not control flight or change the world.
"""
import csv
import json
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from compare_astar_runs import draw_map


def main():
    run, output = map(Path, sys.argv[1:3])
    root = Path(__file__).resolve().parents[2]
    course = json.loads((root/'drone_code/team_nav/course_maps/real_course_2026-11-08.json').read_text())
    flight = next(run.glob('course_run_records/*practice_practice_corrected_course'))
    plan = json.loads((flight/'astar_plan.json').read_text())
    points = [plan['start_metres']]
    for stage in plan['stages']:
        points.extend(stage['points'])
    points = np.array(points)
    record = next(run.glob('referee/flight_*.csv'))
    with record.open() as source:
        rows = [row for row in csv.DictReader(source) if row['armed'] == 'True']
    actual = np.array([[float(row['course_x_metres']),float(row['course_y_metres'])] for row in rows])
    figure, (map_axis, height_axis) = plt.subplots(1,2,figsize=(13,6),dpi=160)
    draw_map(map_axis, course)
    map_axis.plot(points[:,0],points[:,1],'o--',color='#147f9b',label='Planned first two goals')
    map_axis.plot(actual[:,0],actual[:,1],color='#263b4b',lw=1,label='Recorded Gazebo flight')
    for index, point in enumerate(points[1:],1):
        map_axis.annotate(str(index),point[:2],xytext=(0,10),textcoords='offset points',weight='bold')
    map_axis.set(xlim=(-3.2,3.2),ylim=(-3.2,3.2),xlabel='East (m)',ylabel='North (m)',
                 title='Flown: direct goal 1 → 2 at 0.75 m')
    map_axis.set_aspect('equal');map_axis.legend(fontsize=8,loc='lower left')
    wall = next(wall for wall in course['walls'] if wall['openings'])
    opening = wall['openings'][0]
    thickness = wall['thickness_metres']
    for bottom, top in [(wall['bottom_height_metres'],opening['bottom_height_metres']),
                        (opening['top_height_metres'],wall['top_height_metres'])]:
        height_axis.add_patch(Rectangle((-thickness/2,bottom),thickness,top-bottom,color='#657584'))
    for mode, color in [('through','#147f9b'),('over','#cf7334')]:
        proposal = json.loads((flight/f'{mode}_window_proposal.json').read_text())
        heights = np.array(proposal['points_metres'])[:,2]
        height_axis.plot([-.6,-.6,.6,.6],heights,'o--',color=color,
                         label=f"{mode.title()} proposal: {proposal['crossing_height_metres']:.3f} m (unflown)")
    height_axis.axhline(course['course_edges']['wall_top_height_metres']-.1,color='#bd3443',ls=':',label='Existing target-height limit: 1.627 m')
    height_axis.set(xlim=(-.85,.85),ylim=(0,3),xlabel='Distance through wall (m)',ylabel='Height above floor (m)',
                    title='Unflown proposals: both fail preflight checks')
    height_axis.legend(fontsize=8,loc='lower left');height_axis.grid(alpha=.15)
    figure.tight_layout()
    output.mkdir(parents=True,exist_ok=True)
    figure.savefig(output/'verification_and_proposals.png')


if __name__ == '__main__':
    main()
