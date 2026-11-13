"""Reconnect the basin stopper in its captive configuration. Open it with the external pivot lever, close it again, and leave both hands clear."""
import math
import numpy as np
import design as d
from drain_runtime import Articulated
from scoring import Score, Sample
from yam_arena.constants import FINGER_OPEN as O, FINGER_CLOSED as C, PAD_TIP_PAST_TCP as D
from yam_arena.demo import Waypoint

SCENE="station"
SCORE_OVER="end"
SUCCESS_HOLD_S=1.


def joint(name, type_, axis, body0, body1, anchor, lower, upper, **values):
    return dict(name=name,type=type_,axis=axis,body0=body0,body1=body1,
                anchor0=anchor,anchor1=(0,0,0),lower=lower,upper=upper,
                stiffness=values.pop('stiffness',0.),damping=values.pop('damping',0.),max_force=values.pop('max_force',1.),**values)


housing = d.ring(.0245,.013,.021,.003)
housing += [d.box((-.020,0,-.029),(.008,.030,.105)),
            d.box((.020,0,-.029),(.008,.030,.105)),
            d.box((0.,0.,-.0875),(.085,.095,.015))]
links=(
    dict(name="plug", pos=(0,0,0), mass=.039, parts=tuple(d.plug())),
    dict(name="pivot_ball", pos=d.BALL, mass=.008,
         parts=(dict(d.box((0,0,0),(.010,)*3),shape='sphere',radius=.005),)),
    dict(name="rod", pos=d.BALL, mass=.018, parts=tuple(d.rod())),
)
joints=(
    joint("lift","prismatic","Z","root","plug",(0,0,0),0.,.035,damping=.12),
    joint("insertion","prismatic","Y","root","pivot_ball",d.BALL,-d.PARK,0.,damping=.8),
    joint("tilt","revolute","X","pivot_ball","rod",(0,0,0),-3.,32.,
          stiffness=.5*math.pi/180,damping=.002*math.pi/180,max_force=.35),
)
WASTE=Articulated("waste",pos=d.CENTER,size=(.085,.145,.15),mass=1.,
                  fixed_base=True,self_collisions=True,parts=tuple(housing),links=links,joints=joints,
                  initial_joints=(("lift",0.),("insertion",-d.PARK),("tilt",0.)),
                  contact_pairs=(("plug","rod"),("plug","root")))
OBJECTS=[WASTE]
_score=Score()
_diag={}


def reset_history():
    global _score,_diag
    _score=Score()
    _diag={}


def observe(states,t,base):
    global _diag
    art=base.scene["waste"]
    q=art.data.joint_pos.torch[0].detach().cpu().numpy()
    qs=dict(zip(art.joint_names,q))
    from yam_arena.runtime import pad_tips_world
    # At maximum opening the midpoint lies 47 mm from either inner pad.
    origin=base.scene.env_origins[0].detach().cpu().numpy()
    fingers=np.asarray([pad_tips_world(base.scene[name])-origin
                        for name in ("left_robot","right_robot")])
    h=float(qs["lift"])
    plug=np.asarray(d.CENTER)+[0,0,.033+h]
    lever=d.point_on_rod(qs["tilt"],-d.ARM,qs["insertion"])
    plug_clear=np.linalg.norm(fingers-plug,axis=1).min()>.060
    clear=plug_clear and np.linalg.norm(fingers-lever,axis=1).min()>.060
    speed=abs(float(art.data.joint_vel.torch[0,art.joint_names.index("lift")]))
    s=Sample(float(t),float(qs["insertion"]),float(qs["tilt"]),h,bool(plug_clear),bool(clear),speed)
    _score.update(s)
    _diag=dict(s.__dict__,score=_score.value,opened=_score.opened,
               captured=d.gauge(s.angle,s.slide,s.height))


def diagnostics(): return _diag
def score(states): return _score.value
def success(states): return _score.value==1.
def distance(states): return 1.-_score.value


def demo():
    # The bearing guides the ball rod in the side port. Only gripper
    # commands move the rod. The plug rises by contact with its eye.
    def target(angle,slide=.0015):
        return d.point_on_rod(angle,-d.ARM,slide)+[0,0,D]
    start=target(0.,-d.PARK)
    end=target(0.)
    park=end+[.02,-.055,.085]
    sequence=[(1.7,start+[0,0,.065],O),(1.3,start,O),(.9,start,C),
              (1.8,end,C),(2.0,end,C)]
    for angle in (8,16,24,29):
        sequence.append((.55,target(math.radians(angle)),C))
    sequence.append((1.2,target(d.OPEN_ANGLE),C))
    for angle in (22,14,6,0):
        sequence.append((.55,target(math.radians(angle)),C))
    sequence.extend([(.8,end,C),(.6,end,O),(1.5,park,O),(2.2,park,O)])
    return dict(right=[Waypoint(dt,tuple(map(float,p)),f, yaw=math.pi/2) for dt,p,f in sequence])
