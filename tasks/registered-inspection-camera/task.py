import math
"""Set the differential bar to the green angle band while keeping its central carriage on the white centre band throughout the adjustment. Move only the two blue input handles. Do not touch the output bar or its central carriage. Release both handles when done. The trainer is unpowered."""
import math
import numpy as np
import output_contact
from yam_arena.task_file import TABLE_TOP,Box
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_articulated import Articulated
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=1.;SCORE_OVER='end'

def p(pos,size,color=None):
 value=dict(pos=pos,size=size)
 if color is not None:value['color']=color
 return value

OUTPUT=Articulated('floating_output',pos=(.24,0,TOP),size=(.16,.13,.055),fixed_base=True,color=(.7,.7,.7),
 parts=(p((0,0,.004),(.035,.13,.008)),),
 links=(dict(name='carriage',mass=.040,pos=(0,0,.035),parts=(p((0,0,0),(.025,.025,.020)),)),
        dict(name='bar',mass=.060,pos=(0,0,.045),parts=(p((0,0,0),(.16,.010,.010),(.95,.55,.05)),))),
 joints=(dict(name='output_shift',body0='root',body1='carriage',type='prismatic',axis='Y',anchor0=(0,0,.035),lower=-.05,upper=.05,stiffness=0.,damping=.01,max_force=0.),
         dict(name='bar_angle',body0='carriage',body1='bar',type='revolute',axis='Z',anchor0=(0,0,.010),lower=-35.,upper=35.,stiffness=0.,damping=.002,max_force=0.)))
INPUTS=[]
for name,x in [('input_left',.305),('input_right',.175)]:
 sign=1 if name=='input_left' else -1
 INPUTS.append(Articulated(name,pos=(x,0,TOP),size=(.044,.12,.085),fixed_base=True,color=(.12,.35,.85),
  parts=(p((0,0,.004),(.042,.12,.008),(.45,.45,.45)),),
  links=(dict(name='fork',mass=.055,pos=(0,0,.045),parts=(
   p((0,-.012,0),(.010,.006,.016)),p((0,.012,0),(.010,.006,.016)),
   p((0,0,-.012),(.044,.030,.008)),p((sign*.028,0,.030),(.024,.024,.020)),
   p((sign*.028,0,.012),(.010,.010,.024)),p((sign*.014,0,.012),(.040,.030,.008)))),),
  joints=(dict(name='input_travel',body0='root',body1='fork',type='prismatic',axis='Y',anchor0=(0,0,.045),lower=-.045,upper=.045,stiffness=0.,damping=.5,max_force=.2),)))
MARK=Box('centre_band',pos=(.24,0,TOP+.009),size=(.040,.008,.001),color=(1.,1.,1.))
# A fixed green bar beside the mechanism gives the target angle without a hidden label.
TARGET=Box('angle_band',pos=(.24,.105,TOP+.004),size=(.14,.008,.002),yaw=math.radians(25),color=(.12,.8,.25))
OBJECTS=[OUTPUT,*INPUTS,MARK,TARGET]
_bad=False;_centre=0.;_angle=0.;_inputs=np.zeros(2);_rest=False;_open=False;_clear=False

def reset_history():
 global _bad,_centre,_angle,_inputs,_rest,_open,_clear
 _bad=False;_centre=0.;_angle=0.;_inputs=np.zeros(2);_rest=False;_open=False;_clear=False
 output_contact.reset()

def observe(states,t,base):
 global _bad,_centre,_angle,_inputs,_rest,_open,_clear
 output_contact.bind(base)
 if not output_contact.output_clear():_bad=True
 art=base.scene['floating_output'];ids=art.find_joints(['output_shift','bar_angle'],preserve_order=True)[0]
 values=art.data.joint_pos.torch[0,ids].detach().cpu().numpy()
 speed=art.data.joint_vel.torch[0,ids].detach().cpu().numpy()
 _centre,_angle=map(float,values)
 _inputs=np.array([float(base.scene[name].data.joint_pos.torch[0,0].item()) for name in ['input_left','input_right']])
 _rest=abs(speed[0])<.004 and abs(speed[1])<.04 and all(abs(float(base.scene[name].data.joint_vel.torch[0,0].item()))<.004 for name in ['input_left','input_right'])
 if not np.isfinite(values).all() or not np.isfinite(_inputs).all() or abs(_centre)>.004:_bad=True
 _open=all(float(np.max(np.abs(base.scene[name].data.joint_pos.torch[0,base.scene[name].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for name in ['left_robot','right_robot'])
 _clear=output_contact.all_clear()

def score(states):
 if _bad:return 0.
 opposite=_inputs[0]>.010 and _inputs[1]<-.010
 if not opposite:return 0.
 if math.radians(21)<_angle<math.radians(29) and _rest:return 1. if _open and _clear else .75
 if _angle>math.radians(15):return .5
 return .25

def success(states):return score(states)==1.
def distance(states):return abs(_angle-math.radians(25))+abs(_centre)*10

def demo():
 paths={}
 for arm,x,y in [('left',.333,.032),('right',.147,-.032)]:
  start=(x,0,TOP+.075+D);end=(x,y,start[2])
  paths[arm]=[Waypoint(1.5,(x,0,TOP+.14),O),Waypoint(1.5,start,O),Waypoint(.8,start,C),Waypoint(5.,end,C),Waypoint(1.,end,C),Waypoint(.8,end,O),Waypoint(1.5,(x,y,TOP+.14),O)]
 return paths


# Camera alignment application. The camera is a rigid visual and collision proxy.
# Its optical axis is judged geometrically. No image formation is claimed.
from dataclasses import replace
_camera_mount = OBJECTS[0]
_camera_links = list(_camera_mount.links)
_camera_bar = dict(_camera_links[1])
_camera_bar['parts'] = tuple(_camera_bar['parts']) + (
    p((-.008,0,.026),(.046,.032,.032),(.12,.12,.12)),
    p((.021,0,.026),(.012,.022,.022),(.12,.35,.85)),
    p((.028,0,.026),(.002,.016,.016),(.05,.05,.05)))
_camera_links[1] = _camera_bar
OBJECTS[0] = replace(_camera_mount,links=tuple(_camera_links))
INSPECTION_TARGET = Box('inspection_target',pos=(.39,.070,TOP+.071),
    size=(.003,.024,.040),color=(.1,.85,.2))
OBJECTS.append(INSPECTION_TARGET)
_alignment_score = score
def score(states):
    value = _alignment_score(states)
    if value == 1.0:
        ray_y = _centre + (.39-.24)*math.tan(_angle)
        if abs(ray_y-.070) >= .012:
            return .75
    return value

# The numeric score itself includes the documented continuous success hold.
# Runner termination timing alone does not establish this scoring condition.
_instantaneous_score = score
_original_observe = observe
_original_reset_history = reset_history
_qualified_since = None
_observation_time = None
def reset_history():
    global _qualified_since, _observation_time
    _original_reset_history()
    _qualified_since = None
    _observation_time = None
def observe(states, t, base):
    global _qualified_since, _observation_time
    _original_observe(states, t, base)
    now = float(t)
    if not math.isfinite(now):
        _qualified_since = None
        _observation_time = None
        return
    if _observation_time is not None and (now < _observation_time or now - _observation_time > 0.100001):
        _qualified_since = None
    _observation_time = now
    if _instantaneous_score(states) == 1.0:
        if _qualified_since is None:
            _qualified_since = now
    else:
        _qualified_since = None
def score(states):
    value = _instantaneous_score(states)
    if value != 1.0:
        return value
    if _qualified_since is not None and _observation_time is not None and _observation_time - _qualified_since >= SUCCESS_HOLD_S - 1e-9:
        return 1.0
    return 0.9
