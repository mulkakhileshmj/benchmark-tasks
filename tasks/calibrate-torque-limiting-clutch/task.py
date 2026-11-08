"""Adjust the grey preload control so the clutch carries the light proof load but slips against the heavier proof load. Use the blue input handle to test each load. Pull the green selector fully toward the output lever for the heavy test. Return the input to its start before selecting heavy. Keep one unchanged preload setting for both tests. Do not touch the orange output lever or the two spring load blocks. Release both hands. The trainer is unpowered."""
import math
from collections import deque
import numpy as np
import output_contact
from yam_articulated import Articulated
from yam_arena.task_file import Box,TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=1.;SCORE_OVER='end'
def p(pos,size,color=None):
 item=dict(pos=pos,size=size)
 if color is not None:item['color']=color
 return item
def cylinder(pos,radius,height,color):return dict(shape='cylinder',pos=pos,size=(radius*2,radius*2,height),radius=radius,height=height,axis='Z',color=color)
ROOT=(p((0,.02,.004),(.30,.30,.008)),p((.065,-.012,.045),(.070,.012,.014)))
LINKS=(
 dict(name='input',mass=.070,pos=(0,0,.025),parts=(cylinder((0,0,0),.020,.010,(.12,.35,.85)),p((0,-.0375,.007),(.014,.075,.010),(.12,.35,.85)),cylinder((0,-.075,.032),.004,.056,(.65,.65,.65)),cylinder((0,-.075,.060),.011,.020,(.12,.35,.85)))),
 dict(name='output',mass=.060,pos=(0,0,.045),parts=(cylinder((0,0,0),.016,.010,(.95,.50,.05)),p((.045,0,0),(.090,.010,.010),(.95,.50,.05)))),
 dict(name='light_load',mass=.020,pos=(.080,.024,.045),parts=(p((0,0,0),(.014,.008,.016),(.9,.8,.1)),)),
 dict(name='selector',mass=.050,pos=(.030,.024,.045),parts=(p((0,.058,-.025),(.022,.140,.012),(.1,.8,.2)),p((0,.116,0),(.014,.014,.040),(.1,.8,.2)),p((0,.116,.020),(.026,.026,.020),(.1,.8,.2)))),
 dict(name='heavy_load',mass=.020,pos=(.030,.024,.045),parts=(p((0,0,0),(.014,.008,.016),(.9,.25,.1)),)),
 dict(name='preload',mass=.030,pos=(-.100,.120,.055),parts=(p((0,0,0),(.026,.020,.020),(.6,.6,.6)),)))
JOINTS=(
 dict(name='input_angle',body0='root',body1='input',type='revolute',axis='Z',anchor0=(0,0,.025),lower=0.,upper=60.,stiffness=0.,damping=.003,max_force=.002),
 dict(name='clutch_slip',body0='input',body1='output',type='revolute',axis='Z',anchor0=(0,0,.020),lower=-90.,upper=10.,stiffness=0.,damping=25.,max_force=.010),
 dict(name='light_compression',body0='root',body1='light_load',type='prismatic',axis='Y',anchor0=(.080,.024,.045),lower=0.,upper=.040,stiffness=30.,damping=1.2,max_force=3.),
 dict(name='heavy_selector',body0='root',body1='selector',type='prismatic',axis='X',anchor0=(.030,.024,.045),lower=0.,upper=.030,stiffness=0.,damping=2.,max_force=.3),
 dict(name='heavy_compression',body0='selector',body1='heavy_load',type='prismatic',axis='Y',lower=0.,upper=.040,stiffness=70.,damping=1.2,max_force=5.),
 dict(name='preload_setting',body0='root',body1='preload',type='prismatic',axis='Y',anchor0=(-.100,.120,.055),lower=0.,upper=.020,stiffness=0.,damping=2.,max_force=.3))
FIXTURE=Articulated('clutch_fixture',pos=(.25,0,TOP),size=(.30,.30,.10),fixed_base=True,self_collisions=True,color=(.65,.65,.65),parts=ROOT,links=LINKS,joints=JOINTS)
REFERENCE=Box('transmission_mark',pos=(.25+.06*math.cos(math.radians(20)),.06*math.sin(math.radians(20)),TOP+.009),size=(.10,.003,.001),yaw=math.radians(20),color=(.12,.8,.25))
OBJECTS=[FIXTURE,REFERENCE]
_bad=False;_light=False;_heavy=False;_reset=False;_setting=None;_history=deque(maxlen=20);_q={};_vel={};_capacity=.01;_clear=False;_open=False
def reset_history():
 global _bad,_light,_heavy,_reset,_setting,_history,_q,_vel,_capacity,_clear,_open
 _bad=False;_light=False;_heavy=False;_reset=False;_setting=None;_history=deque(maxlen=20);_q={};_vel={};_capacity=.01;_clear=False;_open=False
 output_contact.reset()
def observe(states,t,base):
 global _bad,_light,_heavy,_reset,_setting,_q,_vel,_capacity,_clear,_open
 output_contact.bind(base)
 if not output_contact.output_clear():_bad=True
 art=base.scene['clutch_fixture'];positions=art.data.joint_pos.torch[0].detach().cpu().numpy();velocities=art.data.joint_vel.torch[0].detach().cpu().numpy()
 _q=dict(zip(art.joint_names,map(float,positions)));_vel=dict(zip(art.joint_names,map(float,velocities)))
 if not np.isfinite(positions).all():_bad=True
 setting=_q['preload_setting'];_capacity=.010+4.*float(np.clip(setting,0.,.020))
 ids=art.find_joints('clutch_slip')[0]
 art.write_joint_effort_limit_to_sim_index(limits=_capacity,joint_ids=ids)
 if _setting is not None and abs(setting-_setting)>.0005:_light=False;_heavy=False;_reset=False;_setting=None;_history.clear()
 angle=_q['input_angle'];output=angle+_q['clutch_slip']
 safe=not _bad
 light=safe and _q['heavy_selector']<.005 and angle>math.radians(20) and output>math.radians(18) and abs(_q['clutch_slip'])<math.radians(3) and _q['light_compression']>.010
 _history.append(dict(t=float(t),input=angle,output=output,light=light,heavy=_q['heavy_selector']>.028,heavy_compression=_q['heavy_compression']))
 if len(_history)>=10 and all(row['light'] for row in list(_history)[-10:]):_light=True;_setting=setting if _setting is None else _setting
 if _light and abs(angle)<math.radians(3) and abs(output)<math.radians(3):_reset=True
 if _light and _reset and len(_history)==20:
  window=list(_history)
  steady=max(row['output'] for row in window)-min(row['output'] for row in window)<math.radians(2)
  slip=window[-1]['input']-window[0]['input']>math.radians(7)
  loaded=all(row['heavy'] and row['heavy_compression']>.002 for row in window)
  if safe and loaded and steady and slip and angle>math.radians(40) and math.radians(8)<output<math.radians(18.5) and -_q['clutch_slip']>math.radians(20):_heavy=True
 _open=all(float(np.max(np.abs(base.scene[name].data.joint_pos.torch[0,base.scene[name].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for name in ['left_robot','right_robot'])
 _clear=output_contact.all_clear()
def score(states):
 if _bad:return 0.
 if _light and _heavy:
  still=all(abs(v)<(.05 if 'angle' in name or name=='clutch_slip' else .004) for name,v in _vel.items())
  return 1. if _open and _clear and still else .75
 if _light:return .5
 if _q and _q['light_compression']>.005 and _q['input_angle']>math.radians(10):return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return float(not _light)+float(not _heavy)
def input_handle(degrees):
 a=math.radians(degrees)
 return (.25+.075*math.sin(a),-.075*math.cos(a),TOP+.085+D)
def demo():
 preload=(.15,.12,TOP+.055+D);setpoint=(.15,.1305,preload[2]);select=(.28,.14,TOP+.065+D);heavy=(.31,.14,select[2])
 left=[Waypoint(1.5,(preload[0],preload[1],TOP+.14),O),Waypoint(1.2,preload,O),Waypoint(.6,preload,C),Waypoint(1.5,setpoint,C),Waypoint(.6,setpoint,O),Waypoint(1.,(setpoint[0],setpoint[1],TOP+.14),O),Waypoint(1.5,(select[0],select[1],TOP+.14),O),Waypoint(1.2,select,O),Waypoint(.6,select,C),Waypoint(5.,select,C),Waypoint(2.,heavy,C),Waypoint(7.,heavy,C),Waypoint(.6,heavy,O),Waypoint(1.,(heavy[0],heavy[1],TOP+.14),O)]
 start=input_handle(0)
 right=[Waypoint(1.5,(start[0],start[1],TOP+.14),O),Waypoint(4.9,(start[0],start[1],TOP+.14),O),Waypoint(1.2,start,O),Waypoint(.6,start,C)]
 right += [Waypoint(.5,input_handle(angle),C,yaw=math.radians(angle)) for angle in (5,10,15,22)]
 right += [Waypoint(1.,input_handle(22),C,yaw=math.radians(22))]
 right += [Waypoint(.4,input_handle(angle),C,yaw=math.radians(angle)) for angle in (15,7,0)]
 right += [Waypoint(4.5,start,C)]
 right += [Waypoint(.5,input_handle(angle),C,yaw=math.radians(angle)) for angle in (5,10,15,20,25,30,35,40,45,50)]
 right += [Waypoint(1.,input_handle(50),C,yaw=math.radians(50)),Waypoint(.6,input_handle(50),O,yaw=math.radians(50)),Waypoint(1.5,(input_handle(50)[0],input_handle(50)[1],TOP+.14),O,yaw=math.radians(50))]
 return dict(left=left,right=right)
def diagnostics():return dict(bad=_bad,light_proof=_light,heavy_proof=_heavy,reset_between=_reset,capacity_nm=_capacity,setting=_setting,joints=_q,robot_clear=_clear,fingers_open=_open)
