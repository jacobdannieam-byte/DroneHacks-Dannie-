"""Pinned Gazebo/PX4 shared-clock proof, only for the isolated simulator launcher.

GZBridge copies Gazebo sim time into the PX4 lockstep scheduler. hrt reads that
scheduler; the ROS bridge forwards the same sec/nsec pair. With integral-us
Gazebo ticks, unit conversion has zero error. DDS latency is NOT clock-offset
uncertainty: ahead-of-node-clock delivery still uses the original 50 ms fence.
"""
import hashlib
import json
import os
from pathlib import Path

PATH = Path(__file__).resolve().parents[1]/'scenarios/model_flight/shared_clock_contract.json'
EVIDENCE_ID = 'pinned-gazebo-px4-shared-clock-v1'


def verify(expected_sha256, image):
    raw=PATH.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=expected_sha256:
        raise ValueError('clock source contract digest mismatch')
    contract=json.loads(raw)
    if image!=contract['image_sha256'] or os.environ.get('EXACT_SITL_CLOCK_CONTRACT_SHA256')!=expected_sha256:
        raise ValueError('exact clock requires pinned isolated launch contract')
    for name,digest in contract['source_and_binary_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest:
            raise ValueError('clock implementation differs from reviewed source: '+name)
    build=Path('/opt/PX4-Autopilot/build/px4_sitl_default/build.ninja').read_text()
    if '-DENABLE_LOCKSTEP_SCHEDULER' not in build:
        raise ValueError('PX4 is not built with lockstep scheduler')
    return contract


def require_integral_microsecond_clock(sec,nsec):
    if type(sec) is not int or type(nsec) is not int or sec<0 or not 0<=nsec<10**9 or nsec%1000:
        raise ValueError('shared clock lost integral-microsecond identity')
