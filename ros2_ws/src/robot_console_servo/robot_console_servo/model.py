"""URDF chain FK, including fixed transforms and joint-origin zero offsets.
The control point is SDK flange link6, not the attached gripper tip.
"""
import hashlib
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

class Model:
    def __init__(self, xml, base='base_link', tip='link6'):
        root=ET.fromstring(xml)
        self.id=root.get('name')+':'+hashlib.sha256(xml.encode()).hexdigest()[:16]+':'+base+':'+tip
        by_child={j.find('child').get('link'):j for j in root.findall('joint')}
        chain=[];seen=set();cursor=tip
        while cursor!=base:
            if cursor in seen or cursor not in by_child:raise ValueError('broken_URDF_chain')
            seen.add(cursor);j=by_child[cursor];chain.append(j);cursor=j.find('parent').get('link')
        self.joints=[];self.names=[];self.lower=[];self.upper=[]
        for j in reversed(chain):
            kind=j.get('type');origin=j.find('origin');axis=j.find('axis')
            xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
            rpy=np.fromstring(origin.get('rpy','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
            a=np.fromstring(axis.get('xyz','1 0 0'),sep=' ') if axis is not None else np.array([1.,0,0])
            if kind not in ('fixed','revolute') or len(xyz)!=3 or len(rpy)!=3 or len(a)!=3 or np.linalg.norm(a)==0:raise ValueError('unsupported_URDF_joint')
            self.joints.append((xyz,Rotation.from_euler('xyz',rpy).as_matrix(),a/np.linalg.norm(a),kind))
            if kind=='revolute':
                limit=j.find('limit');self.names.append(j.get('name'))
                self.lower.append(float(limit.get('lower')));self.upper.append(float(limit.get('upper')))
        if len(self.names)!=6:raise ValueError('Expected six-axis flange chain')
    def pose(self,q):
        if len(q)!=6 or not np.isfinite(q).all():raise ValueError('invalid_joint_feedback')
        r=np.eye(3);p=np.zeros(3);index=0
        for xyz,origin,axis,kind in self.joints:
            p+=r@xyz;r=r@origin
            if kind=='revolute':r=r@Rotation.from_rotvec(axis*q[index]).as_matrix();index+=1
        return np.r_[p,Rotation.from_matrix(r).as_euler('xyz')]
