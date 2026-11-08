"""Find bright transmission then extinction using the blue analyzer. Hold the source button while testing. Show a stable bright reading followed by a stable dark reading. Release the source button without changing the dark setting. Release both hands. Do not use a powered off source as proof of extinction."""
import math,os
from collections import deque
import numpy as np
import release_contact
from yam_articulated import Articulated
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=1.;SCORE_OVER='end'
def box(pos,size,color):return dict(pos=pos,size=size,color=color)
def cyl(pos,r,h,color):return dict(shape='cylinder',pos=pos,size=(2*r,2*r,h),radius=r,height=h,color=color)
BLUE=(.12,.35,.85);GREEN=(.1,.8,.2);GREY=(.55,.55,.55)
ROOT=(box((0,0,.004),(.25,.28,.008),GREY),cyl((0,0,.025),.015,.042,GREY),box((.065,.09,.035),(.060,.030,.025),(.02,.02,.02)),box((-.03,.20,.013),(.040,.040,.010),GREY))
LINKS=(dict(name='analyzer',mass=.080,pos=(0,0,.050),parts=(cyl((0,0,0),.045,.008,BLUE),box((0,.027,.006),(.003,.080,.003),(1.,1.,1.)),cyl((0,.060,.030),.004,.060,BLUE))),dict(name='knob',mass=.025,pos=(0,.060,.110),parts=(cyl((0,0,0),.011,.020,BLUE),)),dict(name='button',mass=.020,pos=(-.03,.20,.030),parts=(box((0,0,0),(.026,.026,.020),GREEN),)))
JOINTS=(dict(name='analyzer_angle',body0='root',body1='analyzer',type='revolute',axis='Z',anchor0=(0,0,.050),lower=0.,upper=180.,stiffness=0.,damping=.010,max_force=.006),dict(name='knob_swivel',body0='analyzer',body1='knob',type='revolute',axis='Z',anchor0=(0,.060,.060),continuous=True,stiffness=0.,damping=.001,max_force=0.),dict(name='source_press',body0='root',body1='button',type='prismatic',axis='Z',anchor0=(-.03,.20,.030),lower=-.012,upper=0.,stiffness=200.,damping=2.,max_force=4.))
OBJECTS=[Articulated('polarizer_fixture',pos=(.19,0,TOP),size=(.25,.28,.10),fixed_base=True,self_collisions=False,color=GREY,parts=ROOT,links=LINKS,joints=JOINTS)]
_angles=deque();_stable=False;_released_stable=False
_q={};_v={};_intensity=0.;_powered=False;_bright=False;_dark=False;_bright_time=0.;_dark_time=0.;_dark_angle=None;_last_t=0.;_open=False;_panel=None
def reset_history():
 global _angles,_stable,_released_stable
 global _q,_v,_intensity,_powered,_bright,_dark,_bright_time,_dark_time,_dark_angle,_last_t,_open,_panel
 _angles.clear();_stable=False;_released_stable=False
 _q={};_v={};_intensity=0.;_powered=False;_bright=False;_dark=False;_bright_time=0.;_dark_time=0.;_dark_angle=None;_last_t=0.;_open=False;_panel=None;release_contact.reset()
def observe(states,t,base):
 global _angles,_stable,_released_stable
 global _q,_v,_intensity,_powered,_bright,_dark,_bright_time,_dark_time,_dark_angle,_last_t,_open,_panel
 release_contact.bind(base)
 global _expert_arm_q
 _expert_arm_q={arm:base.scene[arm+'_robot'].data.joint_pos.torch[0,:6].detach().cpu().numpy().copy() for arm in ('left','right')}
 art=base.scene['polarizer_fixture'];_q=dict(zip(art.joint_names,map(float,art.data.joint_pos.torch[0].detach().cpu().numpy())));_v=dict(zip(art.joint_names,map(float,art.data.joint_vel.torch[0].detach().cpu().numpy())))
 angle=_q['analyzer_angle'];_angles.append((float(t),angle))
 while len(_angles)>1 and _angles[1][0]<=float(t)-.5:_angles.popleft()
 span=max(a for _,a in _angles)-min(a for _,a in _angles)
 complete=_angles[-1][0]-_angles[0][0]>=.499
 _stable=complete and span<math.radians(.5)
 _released_stable=complete and span<math.radians(.25)
 _powered=_q['source_press']<-.008
 # Ideal linear polarizers. The signal is normalized after the fixed first filter.
 _intensity=math.cos(angle-math.radians(35))**2 if _powered else 0.
 dt=max(0.,min(.1,float(t)-_last_t));_last_t=float(t)
 _bright_time=_bright_time+dt if _powered and _intensity>.90 and _stable else 0.
 if _bright_time>=.5:_bright=True
 _dark_time=_dark_time+dt if _bright and _powered and _intensity<.01 and _stable else 0.
 if _dark_time>=.5:_dark=True;_dark_angle=angle
 if _dark_angle is not None and abs(angle-_dark_angle)>math.radians(2):_dark=False;_dark_angle=None
 _open=all(float(np.max(np.abs(base.scene[name].data.joint_pos.torch[0,base.scene[name].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for name in ['left_robot','right_robot'])
 if _panel is None:
  from isaaclab.sim.utils.stage import get_current_stage
  from pxr import UsdGeom
  for prim in get_current_stage().Traverse():
   path=prim.GetPath().pathString
   if '/env_0/' in path and '/polarizer_fixture/' in path and path.endswith('/root/p2'):_panel=UsdGeom.Gprim(prim).GetDisplayColorAttr();break
  if _panel is None:raise RuntimeError('Detector display not found')
 from pxr import Gf
 # Visual readout only. This never changes a rigid pose or a joint target.
 _panel.Set([Gf.Vec3f(.02,.02+.95*_intensity,.02)])
def score(states):
 if not _q:return 0.
 if _bright and _dark:
  if not _powered and _q['source_press']>-.003 and _released_stable and _open and release_contact.all_clear():return 1.
  return .75
 if _bright:return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return float(not _bright)+float(not _dark)
def handle(angle):
 a=math.radians(angle);return (.19-.060*math.sin(a),.060*math.cos(a),TOP+.110+D)
def demo():
 press=(.16,.20,TOP+.040+D);down=(press[0],press[1],press[2]-.016)
 left=[Waypoint(1.5,(press[0],press[1],TOP+.14),C),Waypoint(1.5,press,C),Waypoint(1.,down,C),Waypoint(22.,down,C),Waypoint(1.,press,C),Waypoint(1.,(press[0],press[1],TOP+.14),O)]
 start=handle(0)
 right=[Waypoint(1.5,(start[0],start[1],TOP+.14),O),Waypoint(1.5,start,O),Waypoint(.8,start,C),Waypoint(.5,start,C)]
 right += [Waypoint(.35,handle(a),C) for a in range(5,36,5)]
 right += [Waypoint(3.,handle(35),C)]
 dark=float(os.environ.get('OPTICAL_DEMO_DARK_DEGREES','107'))
 right += [Waypoint(.35,handle(a),C) for a in range(40,int(dark)+1,5)]
 last=handle(dark);a=math.radians(dark);back=(last[0]+.090*math.cos(a),last[1]+.090*math.sin(a),last[2])
 right += [Waypoint(.35,last,C),Waypoint(3.,last,C),Waypoint(1.5,back,C),Waypoint(1.,back,O),Waypoint(1.5,(back[0],back[1],TOP+.16),O)]
 return dict(left=left,right=right)
def diagnostics():return dict(joints=_q,intensity=_intensity,powered=_powered,bright_proof=_bright,dark_proof=_dark,bright_hold=_bright_time,dark_hold=_dark_time,fingers_open=_open,angle_window_stable=_stable,released_window_stable=_released_stable)
