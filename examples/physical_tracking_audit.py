"""Summarize sampled tracking evidence without running flight code.

Arrival-age samples cannot establish capture age, callback gaps or reset history.
Repeated sampled poses are reported as evidence, not automatically called faults.
"""
import argparse
import csv
import gzip
import json
from pathlib import Path
import numpy as np


def summarize(folder):
    with gzip.open(folder / 'drone_condition.csv.gz', 'rt') as stream:
        rows = list(csv.DictReader(stream))
    ages = [float(row['camera_tracking_age_seconds']) for row in rows if row['camera_tracking_age_seconds']]
    times = np.array([float(row['seconds_since_start']) for row in rows])
    poses = [(row['tracked_forward_metres'], row['tracked_left_metres'], row['tracked_up_metres']) for row in rows]
    repeated = sum(a == b for a, b in zip(poses, poses[1:]))
    with gzip.open(folder / 'events.jsonl.gz', 'rt') as stream:
        events = [json.loads(line) for line in stream]
    return dict(recording=folder.name, samples=len(rows),
        arrival_age_percentiles_seconds=dict(zip(['p50','p95','p99','max'], np.percentile(ages,[50,95,99,100]).tolist())),
        samples_over_02=sum(age > .2 for age in ages), samples_over_05=sum(age > .5 for age in ages),
        nonincreasing_sample_times=int(np.sum(np.diff(times) <= 0)),
        largest_sample_gap_seconds=float(np.max(np.diff(times))),
        consecutive_identical_sampled_poses=repeated,
        stops=[event for event in events if event['kind']=='stop'],
        unverified=['capture age', 'callback arrival gaps', 'estimator resets', 'deployment identity',
                    'frozen pose versus repeated sampling of latest report'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders', nargs='+', type=Path)
    args = parser.parse_args()
    print(json.dumps([summarize(folder) for folder in args.folders], indent=2))
