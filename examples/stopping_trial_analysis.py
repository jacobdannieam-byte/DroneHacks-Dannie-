"""Offline analysis of independent stopping telemetry; never commands a vehicle.

Times are monotonic seconds in one declared clock. Position is metres in one
fixed frame; velocity is metres/second in that frame. The collector must retain
the original recording and command timestamps. Synthetic tests are not evidence.
"""
import argparse
import json
import math
import re
from pathlib import Path


def reviewed_provenance(document):
    """Check retained native clocks and attribution; not authenticate raw data."""
    from examples.h1_observation import ClockMapping, RESET_FIELDS
    if document.get('provenance_scope') != 'reviewed_calibration':
        raise ValueError('reviewed calibration provenance required')
    if document.get('trial_kind', 'stopping') != 'stopping':
        raise ValueError('hover-only trial cannot measure a moving stop')
    mapping = ClockMapping(**document['clock_mapping'])
    identity = {key: document.get(key) for key in ('trial_id', 'owner_id', 'profile_sha256')}
    if (document.get('clock_id') != mapping.ros_clock_id or
            any(not isinstance(v, str) or not v.strip() for v in identity.values()) or
            not re.fullmatch('[0-9a-f]{64}', identity['profile_sha256']) or
            document.get('pending_clock_samples') != 0):
        raise ValueError('invalid calibration identity or pending samples')

    def native(row):
        fields = ('source_stamp_us', 'source_publish_us', 'mapped_stamp_ns', 'receipt_ns',
                  'ingestion_ns', 'receipt_wall_ns', 'ingestion_wall_ns')
        if any(type(row.get(key)) is not int for key in fields):
            raise ValueError('native timestamp provenance required')
        stamp, receipt, ingestion = row['mapped_stamp_ns'], row['receipt_ns'], row['ingestion_ns']
        if (mapping.capture(row['source_stamp_us'], ingestion) != stamp or
                row['source_publish_us'] < row['source_stamp_us'] or
                row['source_publish_us'] * 1000 + mapping.offset_ns + mapping.max_error_ns > ingestion or
                not mapping.valid_from_ns <= receipt <= mapping.valid_until_ns or
                not 0 <= ingestion - receipt <= 50_000_000 or
                not 0 <= row['ingestion_wall_ns'] - row['receipt_wall_ns'] <= 50_000_000 or
                not 0 <= ingestion - stamp + mapping.max_error_ns <= 100_000_000):
            raise ValueError('native clock/ingestion mismatch')
        return stamp

    counters = None
    for row in document['samples']:
        if native(row) / 1e9 != row['time_s']:
            raise ValueError('sample time differs from native stamp')
        current = row.get('reset_counters')
        if (not isinstance(current, dict) or set(current) != set(RESET_FIELDS) or
                any(type(v) is not int or not 0 <= v <= 255 for v in current.values()) or
                (counters is not None and counters != current)):
            raise ValueError('missing or changed estimator reset counters')
        counters = current
    statuses = document.get('status_reports', [])
    previous = None
    armed_seen = False
    first_disarm = None
    for row in statuses:
        stamp = native(row)
        if ((previous is not None and stamp <= previous) or type(row.get('arming_state')) is not int
                or row.get('arming_state') not in (1, 2)):
            raise ValueError('invalid native status history')
        previous = stamp
        if row['arming_state'] == 2:
            armed_seen = True
        elif armed_seen and first_disarm is None:
            first_disarm = row
    for event in document['events']:
        if any(event.get(k) != v for k, v in identity.items()):
            raise ValueError('owner event identity mismatch')
        if event['event'] == 'disarm':
            if (first_disarm is None or native(event) != first_disarm['mapped_stamp_ns'] or
                    any(event.get(k) != first_disarm[k] for k in
                        ('source_stamp_us', 'source_publish_us', 'receipt_ns', 'ingestion_ns'))):
                raise ValueError('disarm missing native transition evidence')
        elif type(event.get('px4_issue_us')) is not int or event.get('px4_issue_us') != mapping.source_stamp_us(event['stamp_ns']):
            raise ValueError('owner command clock mismatch')
        elif (event.get('publisher_binding') != 'singleton_graph_only' or
              event.get('publisher_node') != identity['owner_id'] or
              not isinstance(event.get('publisher_gid'), str) or
              not re.fullmatch('[0-9a-f]{48}', event['publisher_gid']) or
              int(event['publisher_gid'], 16) == 0):
            raise ValueError('owner event lacks native publisher binding')
    return mapping.max_error_ns / 1e9


def analyze(document, *, require_reviewed=False):
    if document.get('recording_faults') or document.get('recording_complete') is not True:
        raise ValueError('recording incomplete or faulted')
    strict = require_reviewed or document.get('provenance_scope') == 'reviewed_calibration'
    clock_error_s = reviewed_provenance(document) if strict else 0.
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
        stamp = event.get('stamp_ns')
        receipt = event.get('ingestion_ns') if strict and name == 'disarm' else event.get('receipt_ns')
        source = 'native_vehicle_status' if name == 'disarm' else 'owner_event_topic'
        if (type(stamp) is not int or type(receipt) is not int or
                not 0 <= receipt-stamp <= 100_000_000 or
                stamp/1e9 != event_time or event.get('source') != source):
            raise ValueError('event provenance or clock mismatch')
    if not rows or not brake < land < disarm or disarm-land + clock_error_s > 30:
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
        end = row['time_s'] + .5 + 2 * clock_error_s
        last = next((j for j in range(index, len(after)) if after[j]['time_s'] >= end - 1e-9), None)
        if last is None or after[last]['time_s'] + clock_error_s >= land:
            continue
        window = after[index:last + 1]
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
                reaction_s=stop['time_s']-brake + clock_error_s, braking_distance_m=excursion,
                landing_duration_s=disarm-land + clock_error_s,
                provenance_scope='reviewed_calibration' if strict else 'synthetic_or_legacy',
                review_status='unreviewed',
                interpretation='conservative command-to-settle time and maximum excursion; review margins separately')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path); parser.add_argument('output', type=Path)
    args=parser.parse_args()
    result=analyze(json.loads(args.recording.read_text()), require_reviewed=True)
    with args.output.open('x') as stream: json.dump(result,stream,indent=2,allow_nan=False)

if __name__ == '__main__': main()
