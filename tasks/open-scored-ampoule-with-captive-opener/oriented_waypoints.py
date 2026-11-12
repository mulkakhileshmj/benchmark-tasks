"""Fixed full-orientation waypoints. No state feedback is used by the demo."""
from dataclasses import dataclass
import numpy as np
from yam_arena import demo,ik
from yam_arena.urdf import axis_angle_matrix

@dataclass
class OrientedWaypoint(demo.Waypoint):
    roll:float=0.
    pitch:float=0.

def rotation(w):
    return (axis_angle_matrix(np.array([1.,0.,0.]),w.roll)
            @axis_angle_matrix(np.array([0.,1.,0.]),w.pitch)@demo.grip_rotation(w.yaw))

def _oriented_target(self,t):
    pos,rot,finger=self.start_pos,self.start_rot,self.finger_start
    for w in self.waypoints:
        goal=np.asarray(w.pos,dtype=float)
        goal_rot=rotation(w) if isinstance(w,OrientedWaypoint) else demo.grip_rotation(w.yaw)
        if t<w.duration:
            s=ik.smoothstep(t/w.duration)
            return pos+s*(goal-pos),ik.slerp_matrix(rot,goal_rot,s),finger+s*(w.finger-finger),w.label
        t-=w.duration;pos,rot,finger=goal,goal_rot,w.finger
    return pos,rot,finger,'done'

demo.ArmPlan.target=_oriented_target
