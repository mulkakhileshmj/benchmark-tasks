"""Seat the blue platform on its matching cup and groove and flat datums. Leave all three ball feet seated and both hands clear. The fixture is unpowered."""
import math
import numpy as np
import seating_contact
from yam_articulated import Compound
from yam_arena.task_file import TABLE_TOP,Box
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_arena.urdf import quat_xyzw_to_matrix
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=1.;SCORE_OVER='end'
R=.010;CENTRE=.010+R*math.sqrt(2);SEAT_Z=TOP+CENTRE+.018
def p(pos,size):return dict(pos=pos,size=size)
def wedge(axis,sign,origin,length=.040):
 points=[]
 for v in (-length/2,length/2):
  for u,z in ((0,0),(.020,0),(.020,.020)):
   point=np.array([sign*u,v,z] if axis=='X' else [v,sign*u,z])+origin;points.append(point)
 points=np.array(points);lo=points.min(axis=0);hi=points.max(axis=0);centre=(lo+hi)/2
 return dict(shape='mesh',pos=centre.tolist(),size=(hi-lo).tolist(),vertices=(points-centre).tolist(),faces=[[2,1,0],[3,4,5],[0,1,4,3],[1,2,5,4],[2,0,3,5]])
CONEXY=(-.050,.050);GROOVEXY=(.050,.050);FLATXY=(0.,-.050)
cone=[p((-.050,.050,.009),(.040,.040,.002))]
for axis in ('X','Y'):
 for sign in (-1,1):cone.append(wedge(axis,sign,np.array([*CONEXY,.010])))
groove=[p((.050,.050,.009),(.050,.040,.002))]
for sign in (-1,1):groove.append(wedge('Y',sign,np.array([*GROOVEXY,.010]),.050))
flat_top=CENTRE-R
DATUMS=[Compound('mount_cone',pos=(.26,0,TOP),size=(.04,.04,.03),mass=.5,kinematic=True,color=(.65,.65,.65),parts=tuple(cone)),
 Compound('mount_groove',pos=(.26,0,TOP),size=(.05,.04,.03),mass=.5,kinematic=True,color=(.65,.65,.65),parts=tuple(groove)),
 Compound('mount_flat',pos=(.26,0,TOP),size=(.05,.05,flat_top-.008),mass=.5,kinematic=True,color=(.65,.65,.65),parts=(p((0,-.05,(.008+flat_top)/2),(.050,.050,flat_top-.008)),))]
parts=[p((0,0,0),(.160,.150,.012))]
for x,y in (CONEXY,GROOVEXY,FLATXY):parts.append(dict(shape='sphere',pos=(x,y,-.018),size=(.020,.020,.020),radius=R,color=(.75,.75,.75)))
for sign in (-1,1):
 parts.append(p((0,sign*.085,.032),(.014,.014,.064)))
 parts.append(p((0,sign*.085,.065),(.028,.028,.024)))
PLATE=Compound('mount_plate',pos=(.10,0,TOP+.028),size=(.160,.198,.055),mass=.200,color=(.12,.35,.85),parts=tuple(parts))
BASE=Box('mount_base',pos=(.26,0,TOP+.004),size=(.18,.18,.008),color=(.45,.45,.45))
OBJECTS=[BASE,*DATUMS,PLATE]
_bad=False;_pose=None;_open=False;_clear=False;_contacts=np.zeros(3,dtype=bool);_references=None
_qualified_since=None;_observation_time=None
def reset_history():
 global _bad,_pose,_open,_clear,_contacts,_references,_qualified_since,_observation_time
 _bad=False;_pose=None;_open=False;_clear=False;_contacts=np.zeros(3,dtype=bool);_references=None
 _qualified_since=None;_observation_time=None;seating_contact.reset()
def observe(states,t,base):
 global _bad,_pose,_open,_clear,_contacts,_references,_qualified_since,_observation_time
 seating_contact.bind(base)
 origin=base.scene.env_origins[0].detach().cpu().numpy()
 _pose=base.arena_world.get_pose_w('mount_plate')[0].detach().cpu().numpy().copy();_pose[:3]-=origin
 references=np.array([base.arena_world.get_pose_w(name)[0,:3].detach().cpu().numpy() for name in ('mount_cone','mount_groove','mount_flat')])
 if _references is None:_references=references.copy()
 finite=all(np.isfinite(np.asarray(getattr(states['mount_plate'],field))).all() for field in ('pos','quat_xyzw','lin_vel','ang_vel','low','high'))
 if not finite or not np.isfinite(_pose).all() or not np.isfinite(references).all() or np.max(np.linalg.norm(references-_references,axis=1))>.002 or states['mount_plate'].low[2]<TOP-.008:_bad=True
 _open=all(float(np.min(np.abs(base.scene[name].data.joint_pos.torch[0,base.scene[name].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for name in ['left_robot','right_robot'])
 _clear=seating_contact.clear();_contacts=seating_contact.contacts()
 now=float(t)
 if not math.isfinite(now):
  _bad=True;_qualified_since=None;_observation_time=None;return
 if _observation_time is not None and (now<_observation_time or now-_observation_time>.100001):_qualified_since=None
 _observation_time=now
 if instantaneous_score(states)==1.:
  if _qualified_since is None:_qualified_since=now
 else:_qualified_since=None
def instantaneous_score(states):
 if _bad or _pose is None:return 0.
 rotation=quat_xyzw_to_matrix(_pose[3:]);yaw=abs(math.atan2(rotation[1,0],rotation[0,0]))
 aligned=np.max(np.abs(_pose[:2]-np.array([.26,0.])))<.003 and abs(_pose[2]-SEAT_Z)<.003 and rotation[2,2]>math.cos(math.radians(3)) and yaw<math.radians(3)
 supported=_contacts
 if aligned and supported.all() and states['mount_plate'].resting(.008,.1):return 1. if _open and _clear else .75
 if supported.any() and rotation[2,2]>math.cos(math.radians(10)):return .5
 if states['mount_plate'].low[2]>TOP+.015:return .25
 return 0.

def score(states):
 value=instantaneous_score(states)
 if value!=1.:return value
 if _qualified_since is not None and _observation_time is not None and _observation_time-_qualified_since>=1.-1e-9:return 1.
 return .9
def success(states):return score(states)==1.
def distance(states):return 1. if _pose is None else float(np.linalg.norm(_pose[:3]-np.array([.26,0,SEAT_Z])))
def demo():
 paths={}
 for arm,sign in [('left',1),('right',-1)]:
  pick=(.10,sign*.085,TOP+.093+D);carry=(.26,sign*.085,TOP+.155+D);seat=(.26,sign*.085,SEAT_Z+.065+D-.0005)
  paths[arm]=[Waypoint(1.5,(pick[0],pick[1],TOP+.14),O),Waypoint(1.5,pick,O),Waypoint(.8,pick,C),Waypoint(1.5,(pick[0],pick[1],carry[2]),C),Waypoint(2.,carry,C),Waypoint(2.,seat,C),Waypoint(1.,seat,C),Waypoint(.8,seat,O),Waypoint(1.5,(seat[0],seat[1],TOP+.14),O)]
 return paths
def diagnostics():
 return dict(bad=_bad,root_pose=None if _pose is None else _pose.tolist(),datum_contacts=_contacts.tolist(),contact_gaps_m=[float(v) if math.isfinite(v) else None for v in seating_contact.gaps()],robot_clear=_clear,fingers_open=_open)

_demo=demo
def demo():
 paths=_demo()
 for arm in paths:
  seat=paths[arm][-3].pos
  paths[arm]=paths[arm][:-2]+[Waypoint(5.,seat,C)]
 return paths
