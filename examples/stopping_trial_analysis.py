"""Offline analysis of independent stopping telemetry; never commands a vehicle.

Times are monotonic seconds in one declared clock. Position is metres in one
fixed frame; velocity is metres/second in that frame. The collector must retain
the original recording and command timestamps. Synthetic tests are not evidence.
"""
import argparse
import json
import math
from pathlib import Path


def analyze(document):
    if document.get('recording_faults') or document.get('recording_complete') is not True:
        raise ValueError('recording incomplete or faulted')
    rows = document['samples']
    brake = document['brake_command_s']
    land = document['land_command_s']
    disarm = document['disarm_s']
    for value in (brake, land, disarm):
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError('invalid event time')
    events = document.get('events', [])
    for name, event_time in (('brake_command', brake), ('land_command', land), ('disarm', disarm)):
        matches = [e for e in events if e.get('event') == name]
        if len(matches) != 1:
            raise ValueError('missing or duplicate causal event')
        event = matches[0]
        stamp, receipt = event.get('stamp_ns'), event.get('receipt_ns')
        source = 'native_vehicle_status' if name == 'disarm' else 'owner_event_topic'
        if (type(stamp) is not int or type(receipt) is not int or
                not 0 <= receipt-stamp <= 100_000_000 or
                stamp/1e9 != event_time or event.get('source') != source):
            raise ValueError('event provenance or clock mismatch')
    if not rows or not brake < land < disarm or disarm-land > 30:
        raise ValueError('missing samples or invalid landing timing')
    if not document.get('clock_id') or not document.get('frame_id'):
        raise ValueError('clock and frame provenance required')
    previous = None
    for row in rows:
        if len(row['position_m']) != 3 or len(row['velocity_mps']) != 3:
            raise ValueError('three dimensional telemetry required')
        values = [row['time_s'], *row['position_m'], *row['velocity_mps']]
        if any(isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) for v in values):
            raise ValueError('nonfinite telemetry')
        if previous is not None and not 0 < row['time_s']-previous <= .1 + 1e-9:
            raise ValueError('nonmonotonic or missing telemetry')
        previous = row['time_s']
    before = [r for r in rows if r['time_s'] <= brake]
    after = [r for r in rows if r['time_s'] >= brake]
    if not before or not after or brake-before[-1]['time_s'] > .1 or after[-1]['time_s'] < disarm:
        raise ValueError('incomplete event coverage')
    speeds = [math.hypot(*r['velocity_mps']) for r in rows]
    if max(speeds) > .50:
        raise ValueError('speed limit exceeded')
    initial = math.hypot(*before[-1]['velocity_mps'])
    if initial < .10:
        raise ValueError('trial did not demonstrate moving stop')
    # Require sustained settling before landing, not a transient zero sample.
    stop = None
    for index, row in enumerate(after):
        end = row['time_s'] + .5
        window = [r for r in after[index:] if r['time_s'] <= end + 1e-9]
        if end >= land or not window or window[-1]['time_s'] < end - .01:
            continue
        if all(math.hypot(*r['velocity_mps']) <= .05 for r in window):
            stop = row
            break
    if stop is None:
        raise ValueError('no sustained stop before landing')
    settled_until_land = [r for r in after if stop['time_s'] <= r['time_s'] <= land]
    if any(math.hypot(*r['velocity_mps']) > .05 for r in settled_until_land):
        raise ValueError('motion resumed before landing')
    interval = [r for r in after if r['time_s'] <= land]
    excursion = max(math.dist(before[-1]['position_m'], r['position_m']) for r in interval)
    # Use the entire command-to-settle interval as a conservative delay bound;
    # do not infer actuator onset from noisy acceleration.
    return dict(result='PASS', initial_speed_mps=initial, peak_speed_mps=max(speeds),
                reaction_s=stop['time_s']-brake, braking_distance_m=excursion,
                landing_duration_s=disarm-land,
                interpretation='conservative command-to-settle time and maximum excursion; review margins separately')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path); parser.add_argument('output', type=Path)
    args=parser.parse_args()
    result=analyze(json.loads(args.recording.read_text()))
    with args.output.open('x') as stream: json.dump(result,stream,indent=2,allow_nan=False)

if __name__ == '__main__': main()
