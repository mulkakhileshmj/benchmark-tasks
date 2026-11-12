"""Open the bound volume on its imaging cradle without exceeding its opening limit.

Move the inert volume from the narrow transport cradle to the wide imaging cradle.
Open it to the marked 110 degree spread. Support both textblocks on their cradle
surfaces and leave the binding channel clear. Never open beyond 115 degrees.
Release both hands and leave the volume still. Do not turn individual pages.
"""
import math
import numpy as np
from yam_arena.task_file import TABLE_TOP
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_arena.urdf import quat_xyzw_to_matrix
from yam_articulated import Articulated,Compound
import geometry as g
from scoring import SupportScore
import support_contacts
from oriented_waypoints import OrientedWaypoint as W

SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=1.;SCORE_OVER='end'

def part(pos,size,color):return dict(pos=pos,size=size,color=color)

links=[];joints=[]
for side,name in enumerate(('leaf_pos','leaf_neg')):
    angle=g.START_ANGLES[side]
    links.append(dict(name=name,mass=.275,pos=g.hinge(side).tolist(),initial_angle_deg=angle,
        parts=(part((0,g.LEAF_WIDTH/2,0),(g.LENGTH,g.LEAF_WIDTH,g.LEAF_THICKNESS),(.76,.70,.55)),)))
    joints.append(dict(name=name+'_hinge',body0='root',body1=name,type='revolute',axis='X',
        anchor0=g.hinge(side).tolist(),anchor1=(0,0,0),initial_angle_deg=angle,
        lower=-angle,upper=180.-angle,stiffness=0.,damping=.0003,max_force=.02))
BOOK=Articulated('book',pos=(g.SOURCE_X,0,TOP+g.SPINE_HEIGHT),size=(g.LENGTH,.21,.20),mass=.050,
    color=(.23,.22,.16),parts=(part((0,0,0),(g.LENGTH,.014,.032),(.23,.22,.16)),),
    links=tuple(links),joints=tuple(joints),self_collisions=True)
OBJECTS=[BOOK]
for side,angle in enumerate(g.START_ANGLES):
    OBJECTS.append(Compound(f'source_{side}',pos=(g.SOURCE_X,0,TOP),size=(g.LENGTH,.20,.20),
        mass=.5,kinematic=True,color=(.37,.43,.47),
        parts=tuple(g.support_piece(side,angle,x,.090) for x in (-.075,.075))))
for side,angle in enumerate(g.GOAL_ANGLES):
    for k,x in enumerate((-.075,.075)):
        OBJECTS.append(Compound(f'target_{side}_{k}',pos=(g.TARGET_X,0,TOP),size=(.09,.20,.15),
            mass=.5,kinematic=True,color=(.14,.35,.45),parts=(g.support_piece(side,angle,x,.090),)))

_scorer=SupportScore();_diag={}

def reset_history():
    global _scorer,_diag
    _scorer=SupportScore();_diag={};support_contacts.reset()

def observe(states,t,base):
    global _diag
    support_contacts.bind(base)
    contacts,gaps,clear,spine_clear=support_contacts.measured()
    art=base.scene['book'];origin=base.scene.env_origins[0].detach().cpu().numpy()
    poses=art.data.body_link_pose_w.torch[0].detach().cpu().numpy()
    velocities=art.data.body_link_vel_w.torch[0].detach().cpu().numpy()
    poses=poses.copy();poses[:,:3]-=origin
    root=poses[art.body_names.index('root')];rotation=quat_xyzw_to_matrix(root[3:])
    angles=[];leaf_rotations=[]
    for name in ('leaf_pos','leaf_neg'):
        r=quat_xyzw_to_matrix(poses[art.body_names.index(name),3:])
        relative=rotation.T@r
        angles.append(math.degrees(math.atan2(r[2,1],r[1,1])))
        leaf_rotations.append(r)
    # Native leaf-to-leaf rotation is independent of all common root rotations.
    opening=g.relative_opening(*leaf_rotations)
    error=float(np.linalg.norm(root[:3]-np.array([g.TARGET_X,0,TOP+g.SPINE_HEIGHT])))
    tilt=math.degrees(math.acos(np.clip(rotation[2,2],-1.,1.)))
    yaw=abs(math.degrees(math.atan2(rotation[1,0],rotation[0,0])))
    fingers=[]
    for name in ('left_robot','right_robot'):
        robot=base.scene[name];ids=robot.find_joints(FINGER_JOINTS,preserve_order=True)[0]
        fingers.append(float(np.min(np.abs(robot.data.joint_pos.torch[0,ids].detach().cpu().numpy())))>.035)
    finite=bool(np.isfinite(poses).all() and np.isfinite(velocities).all())
    at_rest=bool(np.max(np.linalg.norm(velocities[:,:3],axis=1))<=.008 and np.max(np.linalg.norm(velocities[:,3:],axis=1))<=.1)
    on_table=states['book'].low[2]>=TOP-.008
    signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    spine_corners=signs*np.array([g.LENGTH,.014,.032])/2
    spine_low=float((spine_corners@rotation.T+root[:3])[:,2].min())
    spine_clear=spine_clear and spine_low>TOP+.001
    _scorer.update(t=float(t),opening_deg=opening,angle_errors_deg=np.array(angles)-g.GOAL_ANGLES,
        root_error_m=error,root_tilt_deg=tilt,root_yaw_deg=yaw,contacts=contacts,
        released=clear and all(fingers),at_rest=at_rest,on_table=on_table,spine_clear=spine_clear,finite=finite)
    _diag=dict(root_pose=root.tolist(),angles_deg=angles,opening_deg=opening,
        root_error_m=error,contacts=contacts.tolist(),contact_gaps_m=[float(v) if math.isfinite(v) else None for v in gaps],
        robot_clear=clear,spine_clear=spine_clear,spine_low_m=spine_low-TOP,
        fingers_open=fingers,at_rest=at_rest,bad=_scorer.bad,reason=_scorer.reason)

def score(states):return _scorer.value
def success(states):return score(states)==1.
def diagnostics():return _diag.copy()
def distance(states):return 1.-score(states)

def _tcp(side,angle,x,spine_height,extra=0.):
    # Clamp the bare 14 mm textblock 10 mm in from its fore edge. No grip tabs.
    return tuple(np.array([x,0,TOP+spine_height])+g.leaf_point(side,angle,.170+D+extra))

def demo_plan(mode='correct'):
    plans={}
    for side,arm in enumerate(('left','right')):
        start=g.START_ANGLES[side];goal=g.GOAL_ANGLES[side]
        if mode=='overopen':goal=(0.,180.)[side]
        height=g.SPINE_HEIGHT+(.003 if mode=='hover' else 0.)
        roll=lambda angle:math.radians(angle-90.)
        pick=_tcp(side,start,g.SOURCE_X,g.SPINE_HEIGHT)
        away=_tcp(side,start,g.SOURCE_X,g.SPINE_HEIGHT,.045)
        lifted=_tcp(side,start,g.SOURCE_X,.065)
        transport=_tcp(side,start,g.TARGET_X,.065)
        opened=_tcp(side,goal,g.TARGET_X,.065)
        seated=_tcp(side,goal,g.TARGET_X,height)
        retreat=_tcp(side,goal,g.TARGET_X,height,.065)
        plans[arm]=[
            W(2.,away,O,'approach fore edge',roll=roll(start)),
            W(1.5,pick,O,'straddle textblock edge',roll=roll(start)),
            W(1.,pick,C,'support textblock',roll=roll(start)),
            W(2.,lifted,C,'lift together',roll=roll(start)),
            W(3.,transport,C,'carry above imaging cradle',roll=roll(start)),
            W(3.,opened,C,'open together',roll=roll(goal)),
            W(3.,seated,C,'lower onto both supports',roll=roll(goal)),
            W(1.5,seated,C,'settle',roll=roll(goal)),
            W(1.,seated,O,'release',roll=roll(goal)),
            W(2.,retreat,O,'clear',roll=roll(goal)),
            W(2.,retreat,O,'observe released stability',roll=roll(goal))]
        if mode=='hover':
            plans[arm]=plans[arm][:8]+[W(5.,seated,C,'keep held above supports',roll=roll(goal))]
    return plans

def demo():return demo_plan()


