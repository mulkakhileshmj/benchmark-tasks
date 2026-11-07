"""Set the sine bar to 30 degrees using one of the three gauge blocks. The roller centres are 130 mm apart. Support the bar while inserting the block below its far roller. Use one block only. Leave the block on the reference plate and release both hands. The fixture is unpowered."""
import math
import numpy as np
import support_contact
from yam_arena.task_file import Box,TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_articulated import Articulated
from yam_arena.urdf import quat_xyzw_to_matrix
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=1.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size)
def roller(x):return dict(pos=(x,0,0),size=(.016,.032,.016),shape='cylinder',axis='Y',radius=.008,height=.032)
FIXTURE=Articulated('sine_fixture',pos=(.16,0,TOP),size=(.23,.14,.050),fixed_base=True,color=(.65,.65,.65),
 parts=(p((.08,0,.004),(.23,.14,.008)),),
 links=(dict(name='bar',mass=.120,pos=(0,0,.016),parts=(p((.065,0,.010),(.15,.028,.012)),roller(0),roller(.13),p((.13,.045,.020),(.018,.09,.012)),p((.13,.085,.028),(.025,.025,.026)))),),
 joints=(dict(name='sine_angle',body0='root',body1='bar',type='revolute',axis='Y',anchor0=(0,0,.016),lower=-75.,upper=0.,stiffness=0.,damping=.002,max_force=0.),))
BLOCKS=[Box(f'support_block_{height}',pos=(x,y,TOP+height/2000),size=(.040,.040,height/1000),mass=.080,color=color) for height,x,y,color in [(35,.14,-.16,(.9,.3,.1)),(50,.26,-.16,(.9,.8,.1)),(65,.20,-.16,(.1,.35,.85))]]
OBJECTS=[FIXTURE,*BLOCKS]
TARGET_X=.16+.13*math.cos(math.radians(30))
_angle=0.;_velocity=0.;_open=False;_clear=False;_supported=False;_bad=False
def reset_history():
 global _angle,_velocity,_open,_clear,_supported,_bad
 _angle=0.;_velocity=0.;_open=False;_clear=False;_supported=False;_bad=False
 support_contact.reset()
def observe(states,t,base):
 global _angle,_velocity,_open,_clear,_supported,_bad
 support_contact.bind(base)
 art=base.scene['sine_fixture'];_angle=-float(art.data.joint_pos.torch[0,0].item());_velocity=abs(float(art.data.joint_vel.torch[0,0].item()))
 _open=all(float(np.max(np.abs(base.scene[name].data.joint_pos.torch[0,base.scene[name].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for name in ['left_robot','right_robot'])
 _clear=support_contact.clear();_supported=support_contact.supported()
 if any(state.low[2]<TOP-.008 for name,state in states.items() if name.startswith('support_block')):_bad=True
def score(states):
 if _bad:return 0.
 block=states['support_block_65']
 q=np.asarray(block.quat_xyzw);x,y,z,w=q
 upright=1-2*(x*x+y*y)>math.cos(math.radians(4))
 roller_on_top=(np.array([TARGET_X,0,block.pos[2]])-block.pos)@quat_xyzw_to_matrix(q)
 placed=np.max(np.abs(roller_on_top[:2]))<.017 and abs(block.low[2]-TOP-.008)<.002 and upright
 correct=placed and abs(_angle-math.radians(30))<math.radians(1.5) and _velocity<.04 and block.resting(.008,.1)
 if correct and _supported:return 1. if _open and _clear else .75
 if placed:return .5
 if block.low[2]>TOP+.015:return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return abs(_angle-math.radians(30))
def handle(angle):
 a=math.radians(angle)
 return (.16+.13*math.cos(a)-.028*math.sin(a),.085,TOP+.016+.13*math.sin(a)+.028*math.cos(a)+D)
def demo():
 start=handle(0);raised=handle(70)
 left=[Waypoint(1.5,(start[0],start[1],TOP+.14),O),Waypoint(1.5,start,O),Waypoint(.8,start,C)]
 left += [Waypoint(.8,handle(angle),C) for angle in (15,30,45,60,70)]
 left += [Waypoint(9.,raised,C)]
 left += [Waypoint(.8,handle(angle),C) for angle in (60,45,32,30)]
 left += [Waypoint(.8,handle(30),O),Waypoint(1.5,(handle(30)[0],handle(30)[1],TOP+.16),O)]
 pick=(.20,-.16,TOP+.0325+D);stage=(TARGET_X,-.10,TOP+.0425+D);place=(TARGET_X,0,stage[2])
 right=[Waypoint(1.5,(pick[0],pick[1],TOP+.14),O),Waypoint(2.3,(pick[0],pick[1],TOP+.14),O),Waypoint(1.5,pick,O),Waypoint(.8,pick,C),Waypoint(1.2,(pick[0],pick[1],TOP+.14),C),Waypoint(1.8,(stage[0],stage[1],TOP+.14),C),Waypoint(1.5,stage,C),Waypoint(3.,place,C),Waypoint(.8,place,O),Waypoint(1.5,(place[0],place[1],TOP+.14),O)]
 return dict(left=left,right=right)

def diagnostics():return dict(angle_deg=math.degrees(_angle),supported=_supported,robot_clear=_clear,fingers_open=_open,bad=_bad,contact=support_contact._details)
