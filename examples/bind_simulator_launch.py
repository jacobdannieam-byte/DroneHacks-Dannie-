"""Bind launch-only clearance to reviewed generated simulator geometry.

Only the validated empty calibration enclosure and a resolved flat collision
model are supported. Course maps and unresolved model includes are refused.
"""
from hashlib import sha256
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from onboard.adapters.geometry import R_ENU_NED, rot_z
from refnav.launch_clearance import SimulatorLaunchReference, BoundLaunchGuard


def bind(runtime, settings, root):
    from simulator.gazebo_trial.calibration_lane import validate_world, numbers, pose
    if not runtime.adapter.simulated_estimator or settings.get('scope')!='isolated_calibration_launch':
        raise ValueError('launch reference is restricted to isolated calibration simulator')
    paths=[Path(root)/settings[key] for key in ('world_file','resolved_vehicle_file')]
    expected=[settings[key] for key in ('world_sha256','resolved_vehicle_sha256')]
    raw=[p.read_bytes() for p in paths]
    if [sha256(b).hexdigest() for b in raw]!=expected:
        raise ValueError('launch geometry source identity mismatch')
    authorization=settings.get('deployment_authorization', {})
    if (authorization.get('scope')!='participant_approved_isolated_launch_only'
            or authorization.get('world_sha256')!=expected[0]
            or authorization.get('resolved_vehicle_sha256')!=expected[1]
            or not isinstance(authorization.get('reviewer'),str) or not authorization['reviewer'].strip()
            or authorization.get('navigation_authority') is not False):
        raise ValueError('explicit participant deployment launch-only authorization required')
    world=validate_world(raw[0].decode())
    model=ET.fromstring(raw[1]).find('model')
    if model is None or model.findall('.//include') or model.findall('model') or any(pose(model)):
        raise ValueError('resolved collision model required, including camera and protrusions')
    low=[];high=[]
    for link in model.findall('link'):
        lp=pose(link)
        if any(lp[3:]): raise ValueError('rotated collision link requires explicit geometry support')
        for collision in link.findall('collision'):
            cp=pose(collision)
            if any(cp[3:]) or collision.find('geometry/box/size') is None:
                raise ValueError('unreviewed collision geometry')
            size=np.asarray(numbers(collision.findtext('geometry/box/size'),3))
            if np.any(size<=0): raise ValueError('invalid collision size')
            center=np.asarray(lp[:3])+cp[:3]
            low.append(center-size/2);high.append(center+size/2)
    if not low: raise ValueError('missing collision geometry')
    lower=np.min(low,axis=0);upper=np.max(high,axis=0)
    # No measured tilt bound exists: enclose every attitude, including camera.
    radius=float(np.linalg.norm(np.maximum(abs(lower),abs(upper))))
    half=(radius,radius,radius)
    profile=runtime.loop.replay.hybrid
    alignment=profile.alignment
    transform=rot_z(alignment.map_from_enu_yaw_rad)@R_ENU_NED
    # This initial adapter requires exact authored world/map axis alignment.
    if (settings.get('world_frame_id')!=alignment.map.frame_id
            or alignment.map_from_enu_yaw_rad != 0.
            or tuple(alignment.ned_origin_in_map_m)!=(0.,0.,0.)
            or authorization.get('frame_id')!=alignment.map.frame_id
            or authorization.get('epoch')!=alignment.map.epoch
            or authorization.get('world_from_map') != 'identity_enu'):
        raise ValueError('explicit matching world/map frame required')
    ref=SimulatorLaunchReference(sha256(b''.join(raw)).hexdigest(),alignment.map.frame_id,
        alignment.map.epoch,world['interior_low_enu_m'][2],world['interior_high_enu_m'][2],
        tuple(world['interior_low_enu_m'][:2]),tuple(world['interior_high_enu_m'][:2]),
        half,float(-lower[2]),0.)
    error=tuple(settings['reviewed_error_ned_m'])
    if not settings.get('uncertainty_review') or len(error)!=3 or any(v<0 for v in error):
        raise ValueError('reviewed directional uncertainty required')
    def valid():
        return [sha256(p.read_bytes()).hexdigest() for p in paths]==expected
    return BoundLaunchGuard(ref,valid,error,transform,alignment.ned_origin_in_map_m,alignment=alignment)
