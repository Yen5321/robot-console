import math
import yaml
import numpy as np

RANGES={
 'cycle_hz':(50,100),'input_timeout_s':(.05,.25),'linear_speed_m_s':(.002,.05),
 'angular_speed_rad_s':(math.radians(1),math.radians(10)+1e-10),
 'linear_acceleration_m_s2':(.01,.1),'angular_acceleration_rad_s2':(.01745,.3491),
 'joint_velocity_rad_s':(.01,.3),'joint_acceleration_rad_s2':(.01,.6)}

def load(path):
    # Reuse duplicate-key rejection from a small independent loader, not hardware modules.
    class Unique(yaml.SafeLoader):pass
    def mapping(loader,node,deep=False):
        d={}
        for k,v in node.value:
            key=loader.construct_object(k,deep=deep)
            if key in d:raise ValueError('Duplicate motion key: '+str(key))
            d[key]=loader.construct_object(v,deep=deep)
        return d
    Unique.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,mapping)
    with open(path,encoding='utf-8') as f:d=yaml.load(f,Loader=Unique)
    if not isinstance(d,dict) or set(d)!=set(RANGES)|{'workspace_m'}:raise ValueError('Unknown/missing motion settings')
    for key,(lo,hi) in RANGES.items():
        v=d[key]
        if type(v) not in (int,float) or not math.isfinite(v) or not lo<=v<=hi:raise ValueError('Invalid motion setting: '+key)
    w=np.asarray(d['workspace_m'],dtype=float)
    if w.shape!=(3,2) or not np.isfinite(w).all() or not np.all(w[:,0]<w[:,1]) or np.max(abs(w))>2:raise ValueError('Invalid workspace_m')
    return d
