"""Resolve collision-only launch geometry; output is not a flyable model."""
import argparse
import json
from hashlib import sha256
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def resolve(base_path, wrapper_path, camera_path):
    paths=list(map(Path,(base_path,wrapper_path,camera_path)))
    base,wrapper,camera=[ET.fromstring(p.read_bytes()).find('model') for p in paths]
    def pose(node):
        entry=node.find('pose')
        if entry is None: return np.zeros(6)
        if entry.get('relative_to'): raise ValueError('unresolved relative frame')
        value=np.array(list(map(float,entry.text.split())))
        if value.shape!=(6,) or not np.isfinite(value).all(): raise ValueError('invalid pose')
        return value
    if wrapper is None or any(pose(wrapper)) or wrapper.findall('link') or wrapper.findall('model'):
        raise ValueError('unsupported wrapper structure')
    includes=wrapper.findall('include')
    if len(includes)!=2 or {i.findtext('uri') for i in includes}!={'model://drone_16_like','model://oak_d_s2_camera'}:
        raise ValueError('unexpected model includes')
    root=ET.Element('sdf',version='1.9'); output=ET.SubElement(root,'model',name='resolved_launch_collisions')
    report=[]
    for inc in includes:
        mount=pose(inc)
        if any(mount[3:]): raise ValueError('unsupported mount rotation')
        model=base if inc.findtext('uri')=='model://drone_16_like' else camera
        if model is None or model.findall('include') or model.findall('model') or any(pose(model)):
            raise ValueError('unresolved nested model')
        links={link.get('name') for link in model.findall('link')}
        joints=model.findall('joint')
        children=[j.findtext('child') for j in joints]
        if len(set(children))!=len(children):
            raise ValueError('multiple joints for one child are unsupported')
        for joint in joints:
            # Supported geometry is a single flat star of fixed/Z-revolute
            # joints. Never silently ignore a displaced pivot or another
            # articulation upstream of a collision link.
            axis=joint.find('axis/xyz')
            if (joint.get('type') not in ('fixed','revolute')
                    or joint.findtext('child') not in links
                    or joint.findtext('parent') not in links
                    or joint.findtext('parent') in children
                    or any(pose(joint))
                    or (axis is not None and axis.attrib)
                    or joint.find('axis/use_parent_model_frame') is not None
                    or joint.find('axis2') is not None):
                raise ValueError('unsupported joint frame or parent articulation')
        for link in model.findall('link'):
            lp=pose(link)
            if any(lp[3:]): raise ValueError('unsupported link rotation')
            joints=[j for j in model.findall('joint') if j.findtext('child')==link.get('name')]
            rotating=any(j.get('type')!='fixed' for j in joints)
            if rotating and (len(joints)!=1 or joints[0].get('type')!='revolute'
                    or joints[0].findtext('axis/xyz','').strip()!='0 0 1'):
                raise ValueError('unsupported articulated collision geometry')
            for collision in link.findall('collision'):
                cp=pose(collision)
                dims=np.array(list(map(float,collision.findtext('geometry/box/size','').split())))
                if dims.shape!=(3,) or not np.isfinite(dims).all() or np.any(dims<=0):
                    raise ValueError('unsupported collision shape')
                # Preserve rotations conservatively with the box circumsphere.
                size=np.full(3,np.linalg.norm(dims)) if any(cp[3:]) else dims
                if rotating:
                    if any(cp[:3]) or any(cp[3:]): raise ValueError('offset rotor collision needs swept-joint support')
                    size=np.array([np.linalg.norm(dims[:2]),np.linalg.norm(dims[:2]),dims[2]])
                center=mount[:3]+lp[:3]+cp[:3]
                name=f'{model.get("name")}__{link.get("name")}__{collision.get("name")}'
                target=ET.SubElement(output,'link',name=name)
                ET.SubElement(target,'pose').text=' '.join(map(str,center))+' 0 0 0'
                c=ET.SubElement(target,'collision',name='collision')
                ET.SubElement(ET.SubElement(ET.SubElement(c,'geometry'),'box'),'size').text=' '.join(map(str,size))
                report.append(dict(name=name,center_m=center.tolist(),enclosing_size_m=size.tolist()))
    if not report: raise ValueError('no collisions')
    return ET.tostring(root),dict(scope='collision-only enclosing model, not flight model',
        sources={str(p):sha256(p.read_bytes()).hexdigest() for p in paths},collisions=report)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('base','wrapper','camera','output'): parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args(); model,report=resolve(args.base,args.wrapper,args.camera)
    args.output.mkdir(exist_ok=False)
    (args.output/'resolved_vehicle.sdf').write_bytes(model)
    (args.output/'collision-provenance.json').write_text(json.dumps(report,indent=2))
