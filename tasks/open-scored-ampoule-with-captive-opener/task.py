"""Score the inert ampoule neck using the abrasive collar, then open it. Keep its body upright.

Keep both grippers off the ampoule cap and neck. Leave the open body in its
holder and the cap retained inside the opener on its rack. Release both hands.
"""
from dataclasses import replace
import math
import numpy as np
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from oriented_waypoints import OrientedWaypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_arena.urdf import quat_xyzw_to_matrix
from yam_articulated import Articulated,Compound
from ampoule_native import BreakawayCap
from scoring import Scorer,Sample
import contact_monitor
import scoring_collar

SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=1.
AXY=np.array([.26,.020]);RXY=np.array([.28,-.14])
BODY_ROOT=np.array([*AXY,TOP+.024]);CAP_ROOT=np.array([*AXY,TOP+.044])
RACK_ROOT=np.array([*RXY,TOP+.084]);INSERT_ROOT=np.array([*AXY,TOP+.0815])

def box(pos,size,color=None):
    out=dict(pos=tuple(pos),size=tuple(size),contact_offset=.0001)
    if color is not None:out['color']=color
    return out

def radial_mesh(sections,n=16):
    vertices=[]
    for z,r in sections:
        vertices.extend([(r*math.cos(2*math.pi*k/n),r*math.sin(2*math.pi*k/n),z) for k in range(n)])
    faces=[list(reversed(range(n))),list(range((len(sections)-1)*n,len(sections)*n))]
    for s in range(len(sections)-1):
        for k in range(n):faces.append([s*n+k,s*n+(k+1)%n,(s+1)*n+(k+1)%n,(s+1)*n+k])
    lo=np.min(vertices,axis=0);hi=np.max(vertices,axis=0);centre=(lo+hi)/2
    return dict(shape='mesh',pos=centre.tolist(),size=(hi-lo).tolist(),
                vertices=(np.array(vertices)-centre).tolist(),faces=faces,contact_offset=.0001)

BODY=Compound('ampoule_body',pos=tuple(BODY_ROOT),size=(.012,.012,.040),mass=.008,
              color=(.72,.83,.89),parts=(radial_mesh([(-.020,.006),(.014,.006),(.0198,.003)]),))
witness=[]
for index in range(36):
    angle=2*math.pi*(index+.5)/36
    witness.append(dict(pos=(.00308*math.cos(angle),.00308*math.sin(angle),.0196),
        size=(.00015,.00048,.0003),rot_z=angle,color=(.72,.83,.89),visual_only=True))
BODY=replace(BODY,parts=BODY.parts+tuple(witness))
CAP=BreakawayCap('ampoule_cap',pos=tuple(CAP_ROOT),size=(.009,.009,.020),mass=.001,
              color=(.65,.78,.88),parts=(radial_mesh([(0,.003),(.005,.0045),(.014,.0045),(.020,.003)]),))

# The opener is full size. The four small spring jaws approximate a compliant
# retaining ring. They close by spring drive and yield only through contact.
walls=[box((sign*.009,0,.001),(.006,.024,.080)) for sign in (-1,1)]
walls +=[box((0,sign*.009,.001),(.012,.006,.080)) for sign in (-1,1)]
walls +=[box((0,0,.0415),(.024,.024,.003))]
links=[];joints=[]
for axis in ('X','Y'):
    for sign in (-1,1):
        name=axis.lower()+('p' if sign>0 else 'n')
        position=(sign*.0052,0,-.027) if axis=='X' else (0,sign*.0052,-.027)
        size=(.002,.004,.006) if axis=='X' else (.004,.002,.006)
        links.append(dict(name=name,mass=.001,pos=position,parts=(box((0,0,0),size),)))
        joints.append(dict(name=name+'_spring',body0='root',body1=name,type='prismatic',axis=axis,
            anchor0=position,anchor1=(0,0,0),lower=0. if sign>0 else -.003,
            upper=.003 if sign>0 else 0.,target=0.,stiffness=5000.,damping=5.,max_force=5.))
for axis in ('X','Y'):
    for sign in (-1,1):
        name='file_'+axis.lower()+('p' if sign>0 else 'n')
        position=(sign*.00345,0,-.0377) if axis=='X' else (0,sign*.00345,-.0377)
        size=(.001,.002,.001) if axis=='X' else (.002,.001,.001)
        vertices=[]
        for z,inner in ((-.0015,.00135),(.0003,-.0005)):
            for u,v in ((inner,-.001),(.00255,-.001),(.00255,.001),(inner,.001)):
                vertices.append((sign*u,v,z) if axis=='X' else (v,sign*u,z))
        vertices=np.asarray(vertices);centre=(vertices.min(0)+vertices.max(0))/2
        blade=dict(shape='mesh',pos=centre.tolist(),size=(vertices.max(0)-vertices.min(0)).tolist(),
            vertices=(vertices-centre).tolist(),faces=((0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)),
            color=(.7,.7,.7),contact_offset=.00005)
        links.append(dict(name=name,mass=.0005,pos=position,parts=(blade,)))
        joints.append(dict(name=name+'_spring',body0='root',body1=name,type='prismatic',axis=axis,
            anchor0=position,anchor1=(0,0,0),lower=0. if sign>0 else -.003,
            upper=.003 if sign>0 else 0.,target=0.,stiffness=500.,damping=.5,max_force=1.))
OPENER=Articulated('opener',pos=tuple(RACK_ROOT),size=(.024,.024,.083),mass=.016,
                   color=(.15,.35,.75),parts=tuple(walls),links=tuple(links),joints=tuple(joints))

def holder_parts(xy,height,inner,outer):
    thick=(outer-inner)/2
    p=[box((*xy,.002),(outer+.016,outer+.016,.004))]
    for sign in (-1,1):
        p.append(box((xy[0]+sign*(inner+thick)/2,xy[1],height/2), (thick,outer,height)))
        p.append(box((xy[0],xy[1]+sign*(inner+thick)/2,height/2),(inner,thick,height)))
    return p

# Holder only reaches 8 mm high so both arms still have to steady and operate
# the ampoule. The neck is not enclosed by the holder.
HOLDER=Compound('body_holder',pos=(0,0,TOP),size=(.036,.036,.008),
                color=(.45,.45,.45),parts=tuple(holder_parts(AXY,.008,.013,.020)))
# Rack ledges support the opener at z=40 mm with its mouth downward and clear.
rp=[box((*RXY,.003),(.070,.070,.006))]
for sign in (-1,1):
    rp.append(box((RXY[0]+sign*.018,RXY[1],.022),(.010,.050,.044)))
    rp.append(box((RXY[0]+sign*.010,RXY[1],.044),(.006,.050,.002)))
RACK=Compound('opener_rack',pos=(0,0,TOP),size=(.070,.070,.045),
              color=(.45,.60,.45),parts=tuple(rp))
# Body must spawn before cap so native neck can bind both rigid bodies.
OBJECTS=[HOLDER,RACK,BODY,OPENER,CAP]
_scorer=Scorer();_score=0.;_diag={}

def reset_history():
    global _scorer,_score,_diag
    _scorer=Scorer();_score=0.;_diag={};contact_monitor.reset();scoring_collar.reset()

def array(x):
    if hasattr(x,'torch'):x=x.torch
    if hasattr(x,'detach'):x=x.detach().cpu().numpy()
    return np.asarray(x,dtype=float)

def physical_vertices(parts):
    signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    return np.concatenate([np.asarray(p['vertices'])+p['pos'] if p.get('shape')=='mesh'
                           else signs*np.asarray(p['size'])/2+p['pos'] for p in parts if not p.get('visual_only')])

MOTION_GEOMETRY={'ampoule_body:':physical_vertices(BODY.parts),
                 'ampoule_cap:':physical_vertices(CAP.parts),'opener:root':physical_vertices(OPENER.parts)}
MOTION_GEOMETRY.update({'opener:'+link['name']:physical_vertices(link['parts']) for link in OPENER.links})

def observe(states,t,base):
    global _score,_diag
    origin=array(base.scene.env_origins[0])
    body=array(base.arena_world.get_pose_w('ampoule_body')[0]).copy();body[:3]-=origin
    cap=array(base.arena_world.get_pose_w('ampoule_cap')[0]).copy();cap[:3]-=origin
    art=base.scene['opener'];idx=art.body_names.index('root')
    tool=array(art.data.body_link_pose_w)[0,idx].copy();tool[:3]-=origin
    if not all(np.isfinite(p).all() and abs(np.linalg.norm(p[3:])-1.)<.01 for p in (body,cap,tool)):
        raise RuntimeError('Invalid ampoule transform')
    rb=quat_xyzw_to_matrix(body[3:]);rt=quat_xyzw_to_matrix(tool[3:]);rc=quat_xyzw_to_matrix(cap[3:])
    neck=body[:3]+rb@np.array([0,0,.020]);gap=float(np.linalg.norm(cap[:3]-neck))
    caplocal=rt.T@(cap[:3]-tool[:3])
    contained=bool(np.max(np.abs(caplocal[:2]))<=.0015 and -.041<=caplocal[2]<=-.033)
    # All cap corners must be inside the protected cavity below its closed top.
    part=CAP.parts[0]
    vertices=(np.array(part['vertices'])+np.array(part['pos']))@rc.T+cap[:3]
    inside=(vertices-tool[:3])@rt
    covered=bool(np.max(np.abs(inside[:,:2]))<=.006 and inside[:,2].min()>=-.039 and inside[:,2].max()<=.040)
    upright=math.degrees(math.acos(float(np.clip(rb[2,2],-1,1))))
    contact_monitor.bind(base,np.array(CAP.parts[0]['vertices'])+np.array(CAP.parts[0]['pos']),
                         MOTION_GEOMETRY,dict(body=BODY_ROOT,tool=RACK_ROOT))
    contact=contact_monitor.read()
    finger_open=[];clear=[]
    for name in ('left_robot','right_robot'):
        robot=base.scene[name];ids=robot.find_joints(FINGER_JOINTS,preserve_order=True)[0]
        finger_open.append(float(np.min(np.abs(array(robot.data.joint_pos)[0,ids])))>.035)
        # Native direct object contact is used as a release guard. Contacts
        # with the fixed rack are irrelevant to whether the object is released.
    body_in_holder=bool(np.max(np.abs(body[:2]-AXY))<.002 and abs(body[2]-BODY_ROOT[2])<.002 and upright<5)
    tool_in_rack=bool(np.max(np.abs(tool[:3]-RACK_ROOT))<.003 and rt[2,2]>math.cos(math.radians(5)))
    slow=contact['observed_rest']['settled']
    scoring_collar.bind(base)
    abrasion=scoring_collar.report()
    sample=Sample(float(t),gap,contained,covered,upright,body_in_holder,tool_in_rack,slow,
        all(finger_open) and contact['objects_clear'],rim_contact=contact['rim'],cap_contact=contact['cap'],
        dropped=min(states['ampoule_body'].low[2],states['ampoule_cap'].low[2])<TOP-.002,
        opener_outside_table=states['opener'].low[2]<TOP-.002,neck_broken=contact['neck_broken'],
        cap_escape=contact['cap_escape'],monitor_valid=contact['valid'],neck_scored=abrasion['scored'])
    _score=_scorer.update(sample)

    _diag=dict(score=_score,reason=_scorer.reason,body_pose=body.tolist(),cap_pose=cap.tolist(),
               tool_pose=tool.tolist(),neck_gap_m=gap,cap_local=caplocal.tolist(),contacts=contact,abrasion=abrasion)

def score(states):return _score
def success(states):return _score==1.
def diagnostics():return _diag

def demo():
    # Fixed time Cartesian commands. No task state is read by this controller.
    source=(*RXY,RACK_ROOT[2]+.012+D)
    # Seat the cap fully before levering. The lower command offsets the
    # repeatable loaded controller error observed in the native pilot.
    insert=(*AXY,INSERT_ROOT[2]+.012+D-.003)
    angle=math.radians(20.)
    neck=np.array([*AXY,TOP+.044])
    cleared=(insert[0],insert[1],insert[2]+.0015)
    lever=cleared[2]-neck[2]
    broken=tuple(neck+np.array([0.,-lever*math.sin(angle),lever*math.cos(angle)]))
    right=[Waypoint(1.5,(source[0],source[1],TOP+.155),O),Waypoint(1.5,source,O),
      Waypoint(.8,source,C),Waypoint(1.2,(source[0],source[1],TOP+.155),C),
      Waypoint(2.,(AXY[0],AXY[1],TOP+.155),C),
      Waypoint(2.,(AXY[0],AXY[1],TOP+.125),C),
      Waypoint(.8,(AXY[0],AXY[1],TOP+.125),C),Waypoint(2.5,insert,C,'seat cap inside opener'),
      Waypoint(1.,insert,C),
      OrientedWaypoint(1.,insert,C,yaw=math.radians(-12.)),
      OrientedWaypoint(3.,insert,C,yaw=math.radians(102.)),
      OrientedWaypoint(3.,insert,C,yaw=math.radians(-12.)),
      OrientedWaypoint(3.,insert,C,yaw=math.radians(102.)),
      OrientedWaypoint(3.,insert,C,yaw=0.),
      Waypoint(1.,cleared,C,'clear the scoring edge from the body shoulder'),
      OrientedWaypoint(1.5,broken,C,roll=angle),
      OrientedWaypoint(1.5,(broken[0],broken[1],broken[2]+.040),C,roll=angle),
      Waypoint(1.5,(broken[0],broken[1],TOP+.155),C),
      Waypoint(2.,(RXY[0],RXY[1],TOP+.155),C),Waypoint(1.5,source,C),
      Waypoint(.8,source,O),Waypoint(1.5,(source[0],source[1],TOP+.155),O)]
    # A 75 degree side approach keeps the neck accessible and remains inside
    # actual YAM joint limits. The pads close across x. Conservative URDF mesh
    # bounds put every gripper collider more than 7.2 mm from the neck centre
    # at the held pose. Native contact checks remain authoritative.
    hold=(AXY[0],AXY[1]+.006,TOP+.025)
    away=(hold[0],hold[1]+.045,hold[2]+.035)
    def side(seconds,pos,opening):
        return OrientedWaypoint(seconds,pos,opening,yaw=-math.pi/2,roll=math.radians(-75.))
    left=[side(1.5,away,O),side(1.5,hold,O),side(.8,hold,-.0045),
          side(30.5,hold,-.0045),side(.8,hold,O),side(1.5,away,O),side(3.,away,O)]
    return dict(left=left,right=right)
