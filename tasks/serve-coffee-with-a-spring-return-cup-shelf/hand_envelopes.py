"""Conservative per-convex-piece capsules in each actual URDF link frame."""
import xml.etree.ElementTree as ET
import numpy as np
from yam_arena import YAM_URDF
_CACHE=None

def envelopes():
    global _CACHE
    if _CACHE is not None:return _CACHE
    result={};tree=ET.parse(YAM_URDF).getroot()
    for name in ('gripper','tip_left','tip_right'):
        parts=[]
        for collision in tree.find("link[@name='%s']"%name).findall('collision'):
            mesh=collision.find('geometry/mesh')
            if mesh is None:raise RuntimeError('Unsupported hand collision primitive')
            raw=(YAM_URDF.parent/mesh.get('filename')).read_bytes();count=int.from_bytes(raw[80:84],'little')
            dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
            v=np.frombuffer(raw,dtype=dtype,count=count,offset=84)['vertices'].reshape(-1,3).astype(float)
            v*=np.array([float(x) for x in mesh.get('scale','1 1 1').split()])
            origin=collision.find('origin');p=np.array([float(x) for x in origin.get('xyz','0 0 0').split()]);rx,ry,rz=[float(x) for x in origin.get('rpy','0 0 0').split()]
            cx,sx=np.cos(rx),np.sin(rx);cy,sy=np.cos(ry),np.sin(ry);cz,sz=np.cos(rz),np.sin(rz)
            R=np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])@np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])@np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
            v=v@R.T+p;low,high=v.min(0),v.max(0);center=(low+high)/2;half=(high-low)/2;axis=int(half.argmax())
            a=center.copy();b=center.copy();a[axis]-=half[axis];b[axis]+=half[axis];radius=max(.0001,float(np.linalg.norm(np.delete(half,axis))))
            parts.append((a,b,radius))
        if not parts:raise RuntimeError('Missing hand collision geometry')
        result[name]=parts
    _CACHE=result;return result
