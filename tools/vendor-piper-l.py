"""Rebuild PIPER-L + conservative stock-gripper model from pinned upstream.

Finger collision boxes bound all mesh vertices across the complete prismatic
stroke. This deliberately overestimates occupied space when opening is unknown.
Only rigid assembly and directly connected arm pairs are excluded, not the
generic Piper ACM. Source mesh and kinematics remain available for auditing.
"""
from pathlib import Path
import copy,hashlib,json,shutil,struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'ros2_ws/src/agx_arm_urdf/piper_l'
DEST=ROOT/'ros2_ws/src/robot_console_servo/model/piper_l'

def main():
    DEST.mkdir(parents=True,exist_ok=True)
    (DEST/'meshes').mkdir(exist_ok=True)
    if SOURCE.exists():
        for p in (SOURCE/'meshes').glob('*.stl'):shutil.copy2(p,DEST/'meshes'/p.name)
        shutil.copytree(SOURCE/'urdf',DEST/'upstream',dirs_exist_ok=True)
    root=ET.parse(DEST/'upstream/piper_l_description.urdf').getroot()
    gripper=ET.parse(DEST/'upstream/piper_l_with_gripper_description.xacro').getroot()
    for name in ('flange_link','gripper_base'):
        root.append(copy.deepcopy(gripper.find(f"link[@name='{name}']")))
    for name in ('flange_joint','gripper_base_joint'):
        root.append(copy.deepcopy(gripper.find(f"joint[@name='{name}']")))
    envelopes={}
    for i in (1,2):
        j=gripper.find(f"joint[@name='gripper_joint{i}']")
        raw=(DEST/f'meshes/gripper_link{i}.stl').read_bytes()
        count=struct.unpack_from('<I',raw,80)[0]
        if len(raw)!=84+50*count:raise ValueError('Expected binary STL')
        vertices=np.array([struct.unpack_from('<9f',raw,84+k*50+12) for k in range(count)]).reshape(-1,3)
        origin=j.find('origin');r=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy'),sep=' ')).as_matrix()
        xyz=np.fromstring(origin.get('xyz'),sep=' ');axis=np.fromstring(j.find('axis').get('xyz'),sep=' ')
        points=np.concatenate([((vertices+float(j.find('limit').get(s))*axis)@r.T)+xyz for s in ('lower','upper')])
        low=points.min(axis=0)-.002;high=points.max(axis=0)+.002
        link=ET.SubElement(root,'link',name=f'finger_envelope{i}')
        for kind in ('visual','collision'):
            item=ET.SubElement(link,kind);ET.SubElement(item,'origin',xyz=' '.join(map(str,(low+high)/2)),rpy='0 0 0')
            ET.SubElement(ET.SubElement(item,'geometry'),'box',size=' '.join(map(str,high-low)))
        fixed=ET.SubElement(root,'joint',name=f'finger_envelope_joint{i}',type='fixed')
        ET.SubElement(fixed,'parent',link='gripper_base');ET.SubElement(fixed,'child',link=f'finger_envelope{i}')
        envelopes[str(i)]={'min_m':low.tolist(),'max_m':high.tolist(),'padding_m':.002}
    for mesh in root.iter('mesh'):
        name=Path(mesh.get('filename')).name.replace('.dae','.stl')
        mesh.set('filename',f'package://robot_console_servo/model/piper_l/meshes/{name}')
    ET.indent(root);ET.ElementTree(root).write(DEST/'piper_l_stock_gripper.urdf',encoding='unicode')
    srdf=ET.Element('robot',name='piper_l');group=ET.SubElement(srdf,'group',name='arm')
    ET.SubElement(group,'chain',base_link='base_link',tip_link='link6')
    excluded=set()
    for joint in root.findall('joint'):
        a=joint.find('parent').get('link');b=joint.find('child').get('link')
        excluded.add(tuple(sorted((a,b))))
    rigid=['link6','flange_link','gripper_base','finger_envelope1','finger_envelope2']
    for i,a in enumerate(rigid):
        for b in rigid[i+1:]:excluded.add(tuple(sorted((a,b))))
    # Flange and gripper housing are rigid extensions of link6, directly mounted
    # to the link5/link6 wrist bearing. FCL measures 1.0/5.5 mm designed clearance.
    # Treat these mounting neighbors as adjacent; retain finger-to-arm checks.
    excluded.update({tuple(sorted(('link5','flange_link'))),tuple(sorted(('link5','gripper_base')))})
    for a,b in sorted(excluded):ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='Adjacent' if not (a in rigid and b in rigid) else 'Rigid gripper assembly / swept envelope overlap')
    ET.indent(srdf);ET.ElementTree(srdf).write(DEST/'piper_l.srdf',encoding='unicode')
    manifest={'upstream':'https://github.com/agilexrobotics/agx_arm_urdf','commit':'f6642ce0d7872c686f29c99e9e10cd23d1d49313','control_point':'base_link -> link6 (SDK flange)','finger_envelopes':envelopes,'collision_exclusions':'direct neighbors and rigid flange/gripper assembly only; not generic Piper ACM','hardware_verified':False}
    manifest['sha256']={p.relative_to(DEST).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in DEST.rglob('*') if p.is_file() and p.name!='manifest.json'}
    (DEST/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
if __name__=='__main__':main()
