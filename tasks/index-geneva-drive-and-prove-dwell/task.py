"""Hold the grey trainer still. Turn the blue crank through two complete forward turns. The orange slotted wheel must advance two quarter turns and stay still during each dwell. Finish with the blue handle at its starting mark and the orange pointer opposite its starting mark. Release both hands. Move only the blue handle and the grey support tab. The trainer is unpowered."""
import math
import numpy as np
import geometry,output_contact
assert geometry.OUTER==.060,'This task revision requires a 60 mm Geneva wheel radius'
from yam_articulated import Articulated
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=1.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size)
FIXTURE=Articulated('geneva_fixture',pos=(.23,0,TOP),size=(.29,.30,.095),mass=3.2,fixed_base=False,self_collisions=True,
 color=(.65,.65,.65),parts=(p((0,0,.004),(.29,.18,.008)),p((0,.130,.012),(.025,.16,.016)),p((0,.185,.026),(.030,.028,.028))),
 links=(dict(name='driver',mass=.080,pos=(-.04,0,.040),parts=geometry.driver_parts()),dict(name='star',mass=.090,pos=(.04,0,.040),parts=geometry.star_parts()),dict(name='handle',mass=.025,pos=(-.04,geometry.R,.110),parts=(dict(shape='cylinder',pos=(0,0,0),size=(.022,.022,.020),radius=.011,height=.020,axis='Z',color=(.12,.35,.85)),))),
 joints=(dict(name='driver_angle',body0='root',body1='driver',type='revolute',axis='Z',anchor0=(-.04,0,.040),continuous=True,stiffness=0.,damping=.003,max_force=0.),
         dict(name='output_angle',body0='root',body1='star',type='revolute',axis='Z',anchor0=(.04,0,.040),continuous=True,stiffness=0.,damping=.004,max_force=0.),dict(name='handle_swivel',body0='driver',body1='handle',type='revolute',axis='Z',anchor0=(0,geometry.R,.070),continuous=True,stiffness=0.,damping=.001,max_force=0.)))
OBJECTS=[FIXTURE]
_bad=False;_previous=None;_total=np.zeros(2);_reference=None;_indices=0;_dwell=None;_velocity=np.zeros(2);_open=False;_clear=False
_root_actual_pose=None;_bad_causes=[];_root_shift=0.
def reset_history():
 global _bad,_previous,_total,_reference,_indices,_dwell,_velocity,_open,_clear
 global _root_actual_pose,_bad_causes,_root_shift
 _bad=False;_previous=None;_total=np.zeros(2);_reference=None;_indices=0;_dwell=None;_velocity=np.zeros(2);_open=False;_clear=False
 output_contact.reset()
 _root_actual_pose=None;_bad_causes=[];_root_shift=0.
def observe(states,t,base):
 global _bad,_previous,_total,_reference,_indices,_dwell,_velocity,_open,_clear
 global _root_actual_pose,_root_shift
 output_contact.bind(base)
 if not output_contact.output_clear():_bad=True;_bad_causes.append('robot output contact')
 art=base.scene['geneva_fixture'];ids=art.find_joints(['driver_angle','output_angle'],preserve_order=True)[0]
 values=art.data.joint_pos.torch[0,ids].detach().cpu().numpy();_velocity=art.data.joint_vel.torch[0,ids].detach().cpu().numpy()
 if _previous is not None:_total+=np.arctan2(np.sin(values-_previous),np.cos(values-_previous))
 _previous=values.copy()
 pose=art.data.body_link_pose_w.torch[0,art.body_names.index('root')].detach().cpu().numpy()
 _root_actual_pose=pose.copy();_root_actual_pose[:3]-=base.scene.env_origins[0].detach().cpu().numpy()
 if _reference is None:_reference=pose.copy()
 angle=2*math.acos(min(1.,abs(float(np.dot(pose[3:],_reference[3:])))))
 _root_shift=float(np.linalg.norm(pose[:3]-_reference[:3]))
 if _root_shift>.005 or angle>math.radians(5) or not np.isfinite(values).all() or _total[0]<math.radians(-5):_bad=True;_bad_causes.append('fixture or input limit')
 phase=(_total[0]+math.pi/2)%(2*math.pi)
 if math.radians(75)<phase<math.radians(285):
  if _dwell is None:_dwell=float(_total[1])
  elif abs(_total[1]-_dwell)>math.radians(5):_bad=True;_bad_causes.append('dwell movement')
 else:_dwell=None
 if _total[0]>math.radians(300) and abs(_total[1]+math.pi/2)<math.radians(5):_indices=max(_indices,1)
 if _indices>=1 and _total[0]>math.radians(650) and abs(_total[1]+math.pi)<math.radians(5):_indices=2
 _open=all(float(np.max(np.abs(base.scene[name].data.joint_pos.torch[0,base.scene[name].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for name in ['left_robot','right_robot'])
 _clear=output_contact.all_clear()
def score(states):
 if _bad:return 0.
 if _indices>=2 and abs(_total[0]-4*math.pi)<math.radians(8) and abs(_total[1]+math.pi)<math.radians(5) and np.max(np.abs(_velocity))<.05:return 1. if _open and _clear else .75
 if _indices>=2:return .5
 if _indices>=1:return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return abs(_total[0]-4*math.pi)+abs(_total[1]+math.pi)
def demo():
 grip=(.23,.185,TOP+.026+D)
 left=[Waypoint(1.5,(grip[0],grip[1],TOP+.14),O),Waypoint(1.5,grip,O),Waypoint(.8,grip,C),Waypoint(32.,grip,C),Waypoint(.8,grip,O),Waypoint(1.5,(grip[0],grip[1],TOP+.14),O)]
 start=(.19,geometry.R,TOP+.110+D)
 right=[Waypoint(1.5,(start[0],start[1],TOP+.14),O),Waypoint(2.3,(start[0],start[1],TOP+.14),O),Waypoint(1.5,start,O),Waypoint(.8,start,C)]
 for angle in np.linspace(math.pi/2,math.pi/2+4*math.pi,49)[1:]:right.append(Waypoint(.50,(.19+geometry.R*math.cos(angle),geometry.R*math.sin(angle),start[2]),C))
 finish=(.19+geometry.R*math.cos(math.pi/2+math.radians(13)),geometry.R*math.sin(math.pi/2+math.radians(13)),start[2])
 right += [Waypoint(2.,finish,C),Waypoint(.8,finish,O),Waypoint(1.5,(finish[0],finish[1],TOP+.14),O)]
 return dict(left=left,right=right)

def diagnostics():return dict(driver_deg=math.degrees(_total[0]),output_deg=math.degrees(_total[1]),indices=_indices,bad=_bad,bad_causes=sorted(set(_bad_causes)),root_shift_m=_root_shift,robot_clear=_clear,fingers_open=_open)
