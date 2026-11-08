"""Level the passive instrument platform by moving its two ballast sliders. Set the ballast with both grey brake handles in their starting positions. Pull both brake handles fully forward. Prove that the free platform returns to level after a roll tilt and after a pitch tilt. Do not move ballast or apply either brake during these proofs. Release both hands. The platform has no levelling motor."""
import math
from dataclasses import replace
import numpy as np
import release_contact
from yam_articulated import Articulated
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=1.;SCORE_OVER='end'
PIVOT=(.27,0,TOP+.070)
def p(pos,size,color):return dict(pos=pos,size=size,color=color)
BLUE=(.12,.35,.85);GREY=(.60,.60,.60)
ROOT=(p((0,0,-.066),(.30,.30,.008),GREY),p((-.115,0,-.035),(.015,.025,.070),GREY),p((.115,0,-.035),(.015,.025,.070),GREY))
FRAME=tuple(p((x,y,0),size,GREY) for x,y,size in [(-.100,0,(.010,.210,.010)),(.100,0,(.010,.210,.010)),(0,-.100,(.210,.010,.010)),(0,.100,(.210,.010,.010))])
PLATFORM=(p((0,0,-.020),(.160,.160,.008),BLUE),p((.05,.03,-.004),(.040,.040,.024),(.9,.15,.1)),p((0,.140,.080),(.026,.026,.024),BLUE),p((-.140,0,.080),(.026,.026,.024),BLUE),p((0,.140,.025),(.012,.012,.110),BLUE),p((-.140,0,.025),(.012,.012,.110),BLUE),p((0,.105,-.020),(.012,.082,.012),BLUE),p((-.105,0,-.020),(.082,.012,.012),BLUE))
LINKS=(dict(name='roll_frame',mass=.080,pos=(0,0,0),center_of_mass=(0,0,0),parts=FRAME),dict(name='platform',mass=.200,pos=(0,0,0),center_of_mass=(.025,.015,-.012),parts=PLATFORM),dict(name='x_weight',mass=.100,pos=(.040,-.040,-.008),center_of_mass=(0,0,0),parts=(p((0,0,0),(.030,.030,.016),(.95,.75,.1)),p((0,0,.030),(.012,.012,.060),(.95,.75,.1)),p((0,0,.060),(.026,.026,.022),(.95,.75,.1)))),dict(name='y_weight',mass=.100,pos=(-.040,.040,-.008),center_of_mass=(0,0,0),parts=(p((0,0,0),(.030,.030,.016),(.1,.75,.95)),p((0,0,.030),(.012,.012,.060),(.1,.75,.95)),p((0,0,.060),(.026,.026,.022),(.1,.75,.95)))),dict(name='roll_brake',mass=.030,pos=(-.110,.200,.010),parts=(p((0,0,0),(.024,.024,.024),GREY),)),dict(name='pitch_brake',mass=.030,pos=(-.110,-.200,.010),parts=(p((0,0,0),(.024,.024,.024),GREY),)))
JOINTS=(dict(name='roll',body0='root',body1='roll_frame',type='revolute',axis='X',lower=-20.,upper=20.,stiffness=0.,damping=1000.,max_force=2.),dict(name='pitch',body0='roll_frame',body1='platform',type='revolute',axis='Y',lower=-20.,upper=20.,stiffness=0.,damping=1000.,max_force=2.),dict(name='x_ballast',body0='platform',body1='x_weight',type='prismatic',axis='X',anchor0=(.040,-.040,-.008),lower=-.065,upper=0.,stiffness=0.,damping=10000.,max_force=.5,friction=.5),dict(name='y_ballast',body0='platform',body1='y_weight',type='prismatic',axis='Y',anchor0=(-.040,.040,-.008),lower=-.050,upper=0.,stiffness=0.,damping=10000.,max_force=.5,friction=.5),dict(name='roll_release',body0='root',body1='roll_brake',type='prismatic',axis='X',anchor0=(-.110,.200,.010),lower=0.,upper=.020,stiffness=0.,damping=2.,max_force=.2),dict(name='pitch_release',body0='root',body1='pitch_brake',type='prismatic',axis='X',anchor0=(-.110,-.200,.010),lower=0.,upper=.020,stiffness=0.,damping=2.,max_force=.2))
OBJECTS=[Articulated('gimbal_fixture',pos=PIVOT,size=(.30,.30,.16),fixed_base=True,self_collisions=False,color=GREY,parts=ROOT,links=LINKS,joints=JOINTS)]
_q={};_v={};_setting=None;_proof=[False]*3;_excited=[False]*2;_hold=0.;_last_t=0.;_clear=False;_open=False;_free=False
def reset_history():
 global _q,_v,_setting,_proof,_excited,_hold,_last_t,_clear,_open,_free
 _q={};_v={};_setting=None;_proof=[False]*3;_excited=[False]*2;_hold=0.;_last_t=0.;_clear=False;_open=False;_free=False;release_contact.reset()
def observe(states,t,base):
 global _q,_v,_setting,_proof,_excited,_hold,_last_t,_clear,_open,_free
 global _expert_arm_q
 _expert_arm_q={arm:base.scene[arm+'_robot'].data.joint_pos.torch[0,:6].detach().cpu().numpy().copy() for arm in ('left','right')}
 release_contact.bind(base);art=base.scene['gimbal_fixture']
 _q=dict(zip(art.joint_names,map(float,art.data.joint_pos.torch[0].detach().cpu().numpy())));_v=dict(zip(art.joint_names,map(float,art.data.joint_vel.torch[0].detach().cpu().numpy())))
 _free=all(_q[name]>.015 for name in ('roll_release','pitch_release'))
 for axis,brake in [('roll','roll_release'),('pitch','pitch_release')]:
  ids=art.find_joints(axis)[0];opened=_q[brake]>.015
  # Passive friction brakes. Target velocity is always zero. No angle servo exists.
  art.write_joint_damping_to_sim_index(damping=.012 if opened else 1000.,joint_ids=ids)
  art.write_joint_effort_limit_to_sim_index(limits=.030 if opened else 2.,joint_ids=ids)
 setting=np.array([_q['x_ballast'],_q['y_ballast']])
 if not _free or (_setting is not None and np.max(np.abs(setting-_setting))>.0005):_setting=None;_proof=[False]*3;_excited=[False]*2;_hold=0.
 if _free and _setting is None:_setting=setting.copy()
 _clear=release_contact.output_clear()
 _open=all(float(np.max(np.abs(base.scene[name].data.joint_pos.torch[0,base.scene[name].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for name in ['left_robot','right_robot'])
 if _free and _proof[0]:
  if abs(_q['roll'])>math.radians(8):_excited[0]=True
  if _proof[1] and abs(_q['pitch'])>math.radians(8):_excited[1]=True
 dt=max(0.,min(.1,float(t)-_last_t));_last_t=float(t)
 level=max(abs(_q[a]) for a in ('roll','pitch'))<math.radians(2) and max(abs(_v[a]) for a in ('roll','pitch'))<.04
 _hold=_hold+dt if _free and _clear and level else 0.
 if _hold>=1.:
  if not _proof[0]:_proof[0]=True;_hold=0.
  elif _excited[0] and not _proof[1]:_proof[1]=True;_hold=0.
  elif _excited[1]:_proof[2]=True
def score(states):
 if not _q:return 0.
 if all(_proof) and _free and _clear and _open and _hold>=1. and release_contact.all_clear():return 1.
 return .25*sum(_proof)
def success(states):return score(states)==1.
def distance(states):return float(3-sum(_proof))
def point(local):return tuple(np.array(PIVOT)+np.array(local)+np.array([0,0,D]))
def tilt(axis,degrees):
 a=math.radians(degrees)
 if axis=='roll':return point((0,.140*math.cos(a)-.080*math.sin(a),.140*math.sin(a)+.080*math.cos(a)))
 return point((-.140*math.cos(a)+.080*math.sin(a),0,.140*math.sin(a)+.080*math.cos(a)))
def demo():
 paths={}
 for arm,start,end,brake,axis,angle in [('left',(-.04,.04,.052),(-.04,.010,.052),(-.11,.20,.010),'roll',-17),('right',(.04,-.04,.052),(-.010,-.04,.052),(-.11,-.20,.010),'pitch',-17)]:
  pick=point(start);place=point(end);b=point(brake);released=point((brake[0]+.020,brake[1],brake[2]));up=lambda pos:(pos[0],pos[1],TOP+.15)
  seq=[Waypoint(1.5,up(pick),O),Waypoint(1.5,pick,O),Waypoint(.8,pick,C),Waypoint(2.,place,C),Waypoint(1.5,place,C),Waypoint(.8,place,O),Waypoint(1.5,up(place),O),Waypoint(1.5,up(b),O),Waypoint(1.5,b,O),Waypoint(.8,b,C),Waypoint(1.5,released,C),Waypoint(.8,released,O),Waypoint(1.5,up(released),O)]
  if arm=='left':seq[:7]=[replace(w,yaw=-math.pi/2) for w in seq[:7]]
  if arm=='left':seq.insert(0,Waypoint(11.5,(.18,.24,TOP+.16),O))
  else:seq[7:7]=[Waypoint(1.5,(.18,-.24,TOP+.16),O),Waypoint(10.,(.18,-.24,TOP+.16),O)]
  # Both axes are released before either perturbation. The other hand waits clear.
  if arm=='right':seq.append(Waypoint(14.,up(released),O))
  else:seq.append(Waypoint(3.,up(released),O))
  neutral=tilt(axis,0);off=tilt(axis,angle);high=lambda pos:(pos[0],pos[1],TOP+.20)
  seq += [Waypoint(1.5,high(neutral),O),Waypoint(1.5,neutral,O),Waypoint(.8,neutral,C),Waypoint(1.5,off,C),Waypoint(.8,off,O),Waypoint(1.5,high(off),O),Waypoint(8.,high(off),O)]
  paths[arm]=seq
 return paths
def diagnostics():return dict(joints=_q,proofs=_proof,excited=_excited,free=_free,clear=_clear,hold=_hold,fingers_open=_open)
