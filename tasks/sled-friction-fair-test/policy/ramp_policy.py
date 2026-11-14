"""Disclosed pose-feedback feasibility demo, using measured comparison outcomes.

This expert reads physical prop poses and crossing/stopping measurements. It
never reads the hidden inertia/friction assignment or edits prop state. Dream
receives the ordinary camera/joint observations.
"""
from dataclasses import replace
import numpy as np
from inspect_robots.types import Action,ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.demo import ArmPlan
from yam_arena.constants import FINGER_OPEN,READY
from review_native_common import carry,push,move

class RampPolicy(YamScriptedPolicy):
    def __init__(self,**kw):
        super().__init__(**kw);self.info=replace(self.info,name='measured_ramp_demo')

    def reset(self,scene):
        super().reset(scene);self._origin=0.;self._phase=0;self._done=False;self.selected=None;self.retries=0

    def _append(self,path,t):
        old=self._plans['left'];q=old.action7(self._end)[0]
        self._plans={'left':ArmPlan(path,self._bases['left'],q_start=q[:6],finger_start=q[6]*FINGER_OPEN)}
        self._origin=t;self._end=self._plans['left'].duration

    def act(self,observation):
        t=float(observation.state_time)+self._dt;local=t-self._origin
        module=self._task.module
        if local>self._end+.5 and not self._done:
            runs=getattr(module,'_runs',[])
            states=getattr(module,'_expert_states',{})
            kind='cross' if module.CONFIG==63 else 'stops'
            data=runs[-1][kind] if runs else {}
            if len(data)<2:
                if local>self._end+20:raise RuntimeError('Two physical comparison outcomes unavailable')
            else:
                choose=max if module.CONFIG==63 else min
                slow=choose(data,key=data.get)
                path=[];half=.025 if module.CONFIG==63 else .02
                if module.CONFIG==63 and self._phase==0:
                    third=next(n for n in module.NAMES if n not in data)
                    push(path,module.BUTTON+[0,0,.028])
                    for n,lane in zip((slow,third),module.LANES):
                        carry(path,states[n].pos,[module.START_X,lane,module.height(module.START_X)+half],half)
                    push(path,module.BUTTON+[0,0,.028]);move(path,6.,module.BUTTON+[0,0,.15])
                    self._phase=1
                else:
                    self.selected=slow
                    carry(path,states[slow].pos,module.BIN+[0,.025,half],half,finger=-.012)
                    move(path,2.,module.BIN+[0,0,.15]);self._done=True
                self._append(path,t);local=0.
        if self._done and local>self._end+2 and module.score({})<1 and self.retries<3:
            path=[]
            center=module._expert_states[self.selected].pos.copy()
            direction=module.BIN[:2]-center[:2]
            direction=direction/max(np.linalg.norm(direction),.001)
            behind=center.copy();behind[:2]-=.065*direction
            behind[2]=module.TOP+.15
            move(path,2.,behind,0.)
            behind[2]=module.TOP+.033
            move(path,2.,behind,0.)
            finish=module.BIN.copy();finish[:2]+=.012*direction
            finish[2]=module.TOP+.033
            move(path,4.,finish,0.)
            finish[2]=module.TOP+.18
            move(path,2.,finish,0.)
            move(path,2.,[.10,-.10,module.TOP+.22])

            self.retries+=1
            self._append(path,t)
            local=0.
        values=[self._plans[a].action7(local)[0] if a in self._plans else np.r_[READY,1.] for a in ('left','right')]
        if self._done and self.retries==0 and self.selected is not None and 3.3<local<13.0:
            from yam_arena import ik
            arm=self._plans['left']
            target,rotation,finger,label=arm.target(local)
            obj=module._expert_states[self.selected]
            desired=np.array(target)-np.array([0.,0.,.019])
            target=np.array(target)+np.clip(.6*(desired-obj.pos),-.025,.025)
            arm.q,_,_=ik.solve(target,ik.matrix_to_quat_xyzw(rotation),arm.q,base_pos=arm.base_pos,iters=75)
            values[0]=np.r_[arm.q,np.clip(finger/FINGER_OPEN,0,1)]
        meta={'request_stop':True} if self._done and local>self._end+3 else {}
        return ActionChunk(actions=[Action(data=pack(*values),meta=meta)],control_hz=1/self._dt)

def policy(**kw):return RampPolicy(**kw)
