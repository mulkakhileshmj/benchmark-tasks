"""Shared native geometry and visible instruments for the YAM tasks."""
import math
import numpy as np
from review_runtime import Compound, Articulated
from yam_arena.constants import PAD_TIP_PAST_TCP as D, FINGER_OPEN as OPEN, FINGER_CLOSED as CLOSED
from yam_arena.demo import Waypoint
from yam_arena.urdf import quat_xyzw_to_matrix

TOP=.7475
GRAY=(.55,.57,.59)
GLYPHS={'A':('010','101','111','101','101'),'B':('110','101','110','101','110'),
        'C':('111','100','100','100','111'),'P':('110','101','110','100','100'),
        'Q':('010','101','101','111','011'),'R':('110','101','110','101','101')}

def labelled(asset,char):
    from dataclasses import replace
    z=asset.size[2]/2+.0004
    parts=list(asset.parts)+[box((0,0,z),(.012,.016,.0002),(1,1,1),visual_only=True)]
    for row,pattern in enumerate(GLYPHS[char]):
        for col,on in enumerate(pattern):
            if on=='1':parts.append(box(((col-1)*.0025,(2-row)*.0025,z+.0002),(.002,.002,.0002),(0,0,0),visual_only=True))
    return replace(asset,parts=tuple(parts))

def box(pos,size,color=GRAY,friction=.5,**kw):
    return dict(pos=tuple(pos),size=tuple(size),color=color,friction=friction,contact_offset=.00005,**kw)

def cylinder(pos,radius,height,axis='Z',color=GRAY,friction=.5):
    size=[2*radius]*3;size['XYZ'.index(axis)]=height
    return box(pos,size,color,friction,shape='cylinder',radius=radius,height=height,axis=axis)

def solid(name,pos,size,mass=.15,color=GRAY,friction=.5,free=True):
    return Compound(name,pos=tuple(pos),size=tuple(size),mass=mass if free else None,
                    parts=(box((0,0,0),size,color,friction),))

def quat_x(angle):return (math.sin(angle/2),0,0,math.cos(angle/2))

def joint(name,axis,body0,body1,anchor,lower,upper,kind='prismatic',**kw):
    return dict(name=name,type=kind,axis=axis,body0=body0,body1=body1,
                anchor0=anchor,anchor1=(0,0,0),lower=lower,upper=upper,
                stiffness=kw.pop('stiffness',0.),damping=kw.pop('damping',.02),
                max_force=kw.pop('max_force',20.),**kw)

def coordinate(base,obj,name):
    art=base.scene[obj]
    return float(art.data.joint_pos.torch[0,art.joint_names.index(name)])

def motor(base,obj,name,target):
    """A commanded actuator target, never a write of object pose or velocity."""
    art=base.scene[obj]
    values=art.data.joint_pos.torch.clone()
    values[:,art.joint_names.index(name)]=target
    art.set_joint_position_target(values)

def hands_clear(base,point,margin=.065):
    from yam_arena.runtime import pad_tips_world
    origin=base.scene.env_origins[0].detach().cpu().numpy()
    return all(np.linalg.norm(pad_tips_world(base.scene[a])-origin-point)>margin for a in ('left_robot','right_robot'))

def in_region(state,center,half):
    return bool(np.all(np.abs(state.pos[:2]-np.array(center[:2]))<=half)
                and state.low[2]>=TOP-.003 and state.high[2]<TOP+.12
                and np.linalg.norm(state.lin_vel)<.01)

def move(path,seconds,point,finger=OPEN,yaw=0.):
    path.append(Waypoint(float(seconds),tuple(point),finger,yaw=yaw))

def carry(path,source,target,half,finger=CLOSED):
    a=np.array(source)+[0,0,D+.004];b=np.array(target)+[0,0,D+.008]
    safe=max(float(a[2]),float(b[2]))+.10
    move(path,1.3,[a[0],a[1],safe]);move(path,1.2,a)
    move(path,.8,a,finger);move(path,1.,[a[0],a[1],safe],finger)
    move(path,4.,[b[0],b[1],safe],finger);move(path,3.,b,finger)
    move(path,.8,b);move(path,1.,[b[0],b[1],safe])

def push(path,point):
    p=np.array(point)+[0,0,D]
    move(path,1.2,p+[0,0,.06]);move(path,.8,p+[0,0,.015],CLOSED)
    move(path,.5,p-[0,0,.008],CLOSED);move(path,.8,p+[0,0,.06])

SEGMENTS=((0,.009,.010,.002),(.006,.0045,.002,.009),(.006,-.0045,.002,.009),
          (0,-.009,.010,.002),(-.006,-.0045,.002,.009),(-.006,.0045,.002,.009),(0,0,.010,.002))
DIGITS={'0':'abcdef','1':'bc','2':'abdeg','3':'abcdg','4':'bcfg','5':'acdfg','6':'acdefg','7':'abc','8':'abcdefg','9':'abcdfg','-':'g',' ':''}

class Display:
    """Physical camera-visible seven-segment instrument; writes only visual colors."""
    def __init__(self,name,pos):
        self.name=name;self.text='    ';self._prims=None
        parts=[box((0,0,0),(.092,.034,.008),(.02,.02,.02))]
        for digit in range(4):
            for x,y,w,h in SEGMENTS:
                parts.append(box((-.032+digit*.021+x,y,.0045),(w,h,.0005),(.08,0,0),visual_only=True))
        for x,y,color in ((-.044,.014,(0,1,1)),(.044,.014,(0,1,0)),
                          (.044,-.014,(0,0,1)),(-.044,-.014,(1,0,1))):
            parts.append(box((x,y,.0045),(.003,.003,.0005),color,visual_only=True))
        self.asset=Compound(name,pos=tuple(pos),size=(.092,.034,.009),parts=tuple(parts))

    def show(self,value):
        text=f'{int(round(value*10)):4d}'
        if len(text)>4:raise ValueError('Instrument range exceeded')
        from pxr import UsdGeom,Gf
        from isaaclab.sim.utils.stage import get_current_stage
        if self._prims is None:
            matches=[p for p in get_current_stage().Traverse()
                     if '/env_0/' in p.GetPath().pathString and self.name in p.GetPath().pathString.split('/')
                     and p.GetName().startswith('p') and p.GetName()[1:].isdigit()]
            self._prims={int(p.GetName()[1:]):p for p in matches}
            if len(self._prims)!=33:raise RuntimeError('Public display geometry unavailable')
        for digit,char in enumerate(text):
            for i,letter in enumerate('abcdefg'):
                on=letter in DIGITS[char]
                UsdGeom.Gprim(self._prims[1+digit*7+i]).GetDisplayColorAttr().Set([Gf.Vec3f(1.,.04,.02) if on else Gf.Vec3f(.08,0,0)])
        self.text=text

    def transcribed(self):
        return float(self.text)/10.

def button(name,pos):
    return Articulated(name,pos=tuple(pos),size=(.035,.035,.03),mass=.1,fixed_base=True,
        parts=(box((0,0,.004),(.035,.035,.008),(.12,.12,.12)),),
        links=(dict(name='cap',pos=(0,0,.022),mass=.015,parts=(box((0,0,0),(.028,.028,.012),(.9,.4,.05)),)),),
        joints=(joint('press','Z','root','cap',(0,0,.022),-.008,0.,stiffness=350.,damping=3.,max_force=10.),))
