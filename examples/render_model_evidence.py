"""Render actual timestamped Gazebo camera frames with separately labelled telemetry."""
import argparse
import csv
import json
from bisect import bisect_right
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def render(root):
    frames=[json.loads(x) for x in (root/'video_frames/frames.jsonl').read_text().splitlines()]
    telemetry=list(csv.DictReader(next((root/'flight_records').glob('*.csv')).open()))
    times=[float(x['seconds']) for x in telemetry]
    decision_file=root/'model_owner.decisions.jsonl'
    rows=[json.loads(x) for x in decision_file.read_text().splitlines()] if decision_file.exists() else []
    decisions={r['sequence']:r for r in rows if r['kind']=='decision'}
    publications=[r for r in rows if r['kind']=='published']
    stamps=[r['stamp_ns']/1e9 for r in publications]
    owner=json.loads((root/'model_owner.json').read_text())
    result=owner.get('result','unknown')
    fontpath=Path('C:/Windows/Fonts/consola.ttf')
    font=ImageFont.truetype(str(fontpath),19) if fontpath.exists() else ImageFont.load_default()
    path=root/'actual-gazebo-flight.mp4'
    with imageio.get_writer(str(path),fps=1,codec='libx264',quality=8,macro_block_size=16,
                           ffmpeg_params=['-movflags','+faststart']) as writer:
        for i,frame in enumerate(frames):
            t=frame['source_ros_s'];canvas=Image.new('RGB',(1280,640),'#111923')
            camera=Image.open(root/'video_frames'/frame['file']).convert('RGB').resize((768,576))
            canvas.paste(camera,(0,48));draw=ImageDraw.Draw(canvas)
            draw.text((15,13),'ACTUAL GAZEBO CAMERA | 1 fps | simulation time',font=font,fill='white')
            index=bisect_right(times,t)-1; count=bisect_right(stamps,t)
            lines=['RECORDED TELEMETRY / DECISIONS',f'ROS time: {t:.1f} s','Diagnostic: bounds nonblocking',
                   'Registration: OFF',f'Published PINN commands: {count}']
            if index>=0:
                obs=telemetry[index]
                lines += [f"Height: {float(obs['height_metres']):.3f} m",
                          f"Course X: {float(obs['course_x_metres']):.3f} m",
                          f"Course Y: {float(obs['course_y_metres']):.3f} m",
                          f"Armed: {obs['armed']} | landed: {obs['landed']}"]
            if count:
                receipt=publications[count-1];decision=decisions[receipt['sequence']]
                lines += [f"Last command waypoint: {decision['waypoint']+1}/22",
                          f"PINN choice: {decision['selected']}",f"D0 comparison: {decision['d0_selected']}",
                          'Command NED metres:',str([round(x,3) for x in receipt['position_ned_m']])]
            else:lines += ['No PINN command published yet','Native startup / climb / recovery']
            lines += ['', 'FINAL OUTCOME (whole run):',result,'Not a safety qualification']
            for line,y in zip(lines,range(24,620,29)):
                draw.text((785,y),line,font=font,fill='#d9e5f2')
            writer.append_data(np.asarray(canvas))
            if i==min(35,len(frames)-1):canvas.save(root/'video-poster.png')
    reader=imageio.get_reader(str(path));decoded=sum(1 for _ in reader);reader.close()
    if decoded!=len(frames):raise ValueError('Video frame count mismatch')
    metadata=dict(video=str(path.resolve()),frames=decoded,fps=1,
        first_ros_s=frames[0]['source_ros_s'],last_ros_s=frames[-1]['source_ros_s'],
        source='Unmodified Gazebo camera content enlarged, with separate recorded-data panel',result=result)
    (root/'video-manifest.json').write_text(json.dumps(metadata,indent=2))
    print(json.dumps(metadata))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('evidence',type=Path)
    render(parser.parse_args().evidence)
