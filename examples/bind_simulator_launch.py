"""Bind launch-only clearance to reviewed generated simulator geometry.

Only the validated empty calibration enclosure and a resolved flat collision
model are supported. Course maps and unresolved model includes are refused.
"""
from hashlib import sha256
from itertools import product
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from onboard.adapters.geometry import R_ENU_NED, rot_z
from refnav.launch_clearance import BoundLaunchGuard, ContactTransition, SimulatorLaunchReference


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
    transition, support_names = None, set()
    if 'contact_transition' in settings:
        pinned = settings['contact_transition']
        if not isinstance(pinned, dict) or set(pinned) != {'file', 'sha256'}:
            raise ValueError('pin the reviewed contact-transition file and SHA256')
        transition_path = Path(root) / pinned['file']
        transition_raw = transition_path.read_bytes()
        transition_sha = sha256(transition_raw).hexdigest()
        if (len(transition_raw) > 16_384 or pinned['sha256'] != transition_sha
                or authorization.get('contact_transition_sha256') != transition_sha):
            raise ValueError('explicit reviewed contact-transition identity required')
        document = json.loads(transition_raw)
        if (not isinstance(document, dict)
                or set(document) != {'scope', 'world_sha256', 'resolved_vehicle_sha256', 'support_collisions', 'bounds'}
                or document.get('scope') != 'reviewed_model_conditioned_isolated_sitl_contact'
                or [document.get(key) for key in ('world_sha256', 'resolved_vehicle_sha256')] != expected):
            raise ValueError('contact-transition assumptions must match the exact isolated sources')
        names = document['support_collisions']
        if (not isinstance(names, list) or not names or len(names) > 8
                or any(not isinstance(name, str) or not name for name in names) or len(set(names)) != len(names)):
            raise ValueError('name each reviewed support collision explicitly')
        support_names = set(names)
        transition = ContactTransition(**document['bounds'])
        if transition.pad_center_xy_m != (0., 0.):
            raise ValueError('contact pad must be centered on the reviewed vehicle spawn')
        paths.append(transition_path)
        expected.append(transition_sha)
    world=validate_world(raw[0].decode())
    model=ET.fromstring(raw[1]).find('model')
    if model is None or model.findall('.//include') or model.findall('model') or any(pose(model)):
        raise ValueError('resolved collision model required, including camera and protrusions')
    low=[];high=[]
    vertices, support, collision_names = [], [], set()
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
            if transition is not None:
                name = f'{link.get("name")}/{collision.get("name")}'
                if name in collision_names:
                    raise ValueError('duplicate collision identity')
                collision_names.add(name)
                for corner in product((-1., 1.), repeat=3):
                    vertices.append(center + size / 2 * corner)
                    support.append(name in support_names and corner[2] == -1.)
    if not low: raise ValueError('missing collision geometry')
    lower=np.min(low,axis=0);upper=np.max(high,axis=0)
    if transition is not None and (not support_names <= collision_names or float(lower[2]) != world['interior_low_enu_m'][2]):
        raise ValueError('reviewed support faces must match the modeled floor at the spawn pose')
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
    return BoundLaunchGuard(ref,valid,error,transform,alignment.ned_origin_in_map_m,alignment=alignment,
                            contact_transition=transition, collision_vertices=vertices, support_vertices=support)
