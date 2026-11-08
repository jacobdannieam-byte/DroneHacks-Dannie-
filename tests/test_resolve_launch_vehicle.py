import xml.etree.ElementTree as ET
import numpy as np
import pytest
from examples.resolve_launch_vehicle import resolve

def files(tmp_path):
    base=tmp_path/'base.sdf';wrapper=tmp_path/'wrapper.sdf';camera=tmp_path/'camera.sdf'
    shape='<collision name="c"><geometry><box><size>.2 .02 .01</size></box></geometry></collision>'
    base.write_text('<sdf><model name="base"><link name="base_link"/><link name="rotor">'+shape+'</link><joint type="revolute"><parent>base_link</parent><child>rotor</child><axis><xyz>0 0 1</xyz></axis></joint></model></sdf>')
    camera.write_text('<sdf><model name="camera"><link name="camera">'+shape+'</link></model></sdf>')
    wrapper.write_text('<sdf><model name="wrapper"><include><uri>model://drone_16_like</uri></include><include><uri>model://oak_d_s2_camera</uri><pose>.12 0 0 0 0 0</pose></include></model></sdf>')
    return base,wrapper,camera

def test_rotor_sweep_and_camera_mount_retained(tmp_path):
    xml,report=resolve(*files(tmp_path))
    assert len(ET.fromstring(xml).findall('.//collision'))==2
    rotor,camera=report['collisions']
    assert rotor['enclosing_size_m']==pytest.approx([np.hypot(.2,.02)]*2+[.01])
    assert camera['center_m']==[.12,0.,0.]

def test_unknown_include_refused(tmp_path):
    base,wrapper,camera=files(tmp_path)
    wrapper.write_text(wrapper.read_text().replace('oak_d_s2_camera','unreviewed_camera'))
    with pytest.raises(ValueError): resolve(base,wrapper,camera)


@pytest.mark.parametrize('change', ['pivot','pivot_frame','axis_frame','parent_axis','articulated_parent'])
def test_unsupported_joint_frames_and_articulation_refused(tmp_path,change):
    base,wrapper,camera=files(tmp_path)
    doc=ET.fromstring(base.read_bytes()); model=doc.find('model'); joint=model.find('joint')
    if change=='pivot': ET.SubElement(joint,'pose').text='.2 0 0 0 0 0'
    elif change=='pivot_frame': ET.SubElement(joint,'pose',relative_to='base_link').text='0 0 0 0 0 0'
    elif change=='axis_frame': joint.find('axis/xyz').set('expressed_in','base_link')
    elif change=='parent_axis': ET.SubElement(joint.find('axis'),'use_parent_model_frame').text='true'
    else:
        ET.SubElement(model,'link',name='mount')
        upstream=ET.SubElement(model,'joint',type='fixed')
        ET.SubElement(upstream,'parent').text='mount'
        ET.SubElement(upstream,'child').text='base_link'
    base.write_bytes(ET.tostring(doc))
    with pytest.raises(ValueError,match='joint frame|articulation|relative frame'):
        resolve(base,wrapper,camera)
