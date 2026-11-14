"""Instrument demo using RGB digit recognition and commanded robot paths only."""
from dataclasses import replace
import math
import numpy as np
import cv2
from inspect_robots.types import Action,ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.demo import ArmPlan
from yam_arena.constants import FINGER_OPEN
from review_native_common import SEGMENTS,DIGITS,carry,move,push

def read_display(image):
    rgb=np.asarray(image)
    if rgb.ndim!=3 or rgb.shape[2]<3:return None
    r,g,b=[rgb[:,:,i].astype(float)/255 for i in range(3)]
    masks=((r<.8*g)&(r<.8*b)&(g>.12)&(b>.12), (r<.45*g)&(g>.18)&(b<.45*g),
           (r<.45*b)&(g<.45*b)&(b>.18),(r>.12)&(g<.8*r)&(g<.8*b)&(b>.12))
    points=[]
    for mask in masks:
        count,labels,stats,centers=cv2.connectedComponentsWithStats(mask.astype('uint8'))
        choices=[i for i in range(1,count) if 2<=stats[i,cv2.CC_STAT_AREA]<=1500]
        if not choices:return None
        choices=[max(choices,key=lambda i:stats[i,cv2.CC_STAT_AREA])]
        points.append(centers[choices[0]])
    points=np.array(points,dtype='float32')
    if abs(cv2.contourArea(points))<80:return None
    target=np.array([[0,0],[880,0],[880,280],[0,280]],dtype='float32')
    transform=cv2.getPerspectiveTransform(points,target)
    warped=cv2.warpPerspective(rgb[:,:,:3],transform,(881,281))
    colors=warped.astype(float)
    red=(colors[:,:,0]>.60*255)&(colors[:,:,0]>2*colors[:,:,1])&(colors[:,:,0]>2*colors[:,:,2])
    output=''
    reverse={frozenset(v):k for k,v in DIGITS.items()}
    for digit in range(4):
        active=set()
        for i,(x,y,w,h) in enumerate(SEGMENTS):
            cx=int((-.032+digit*.021+x+.044)*10000)
            cy=int((.014-y)*10000)
            dx=max(3,int(w*10000*.3));dy=max(3,int(h*10000*.3))
            patch=red[max(0,cy-dy):cy+dy+1,max(0,cx-dx):cx+dx+1]
            if patch.size and patch.mean()>.45:active.add('abcdefg'[i])
        if frozenset(active) not in reverse:return None
        output+=reverse[frozenset(active)]
    try:return float(output)/10.
    except ValueError:return None

def visible_number(observation):
    numbers=[v for image in observation.images.values() if (v:=read_display(image)) is not None]
    return numbers[0] if numbers and max(numbers)-min(numbers)<.11 else None

class VisiblePolicy(YamScriptedPolicy):
    def __init__(self,**kw):
        super().__init__(**kw);self.info=replace(self.info,name='visible_instrument')

    def reset(self,scene):
        super().reset(scene)
        self._origin=0.;self._phase=0;self._read=[];self._reference=None
        self._samples={};self._complete=False;self._adjustments=0

    def _append(self,path,t):
        # Continue from commanded robot joints, with no hidden object offset.
        old=self._plans['left'];q=old.action7(self._end)[0]
        self._plans={'left':ArmPlan(path,self._bases['left'],q_start=q[:6],finger_start=q[6]*FINGER_OPEN)}
        self._origin=t;self._end=self._plans['left'].duration

    def act(self,observation):
        t=float(observation.state_time)+self._dt;local=t-self._origin
        module=self._task.module
        value=visible_number(observation)
        if value is not None and local>=self._end-1.5:self._read.append(value)
        if local>self._end+.5 and not self._complete:
            if len(self._read)<5:
                if local>self._end+8:raise RuntimeError('Cannot read visible instrument; no hidden-value fallback')
            else:
                measured=float(np.mean(self._read[-20:]));self._read=[]
                path=[]
                if hasattr(module,'ZERO_ERROR'):
                    if self._phase==0:
                        self._reference=measured
                        carry(path,module.REF+[0,0,.05],module.PLATFORM+[0,0,.031],.025)
                        move(path,2.,module.PLATFORM+[0,0,.16]);self._phase=1
                    else:
                        error=measured-self._reference
                        if abs(error)<=.125:self._complete=True
                        elif self._adjustments>=12:raise RuntimeError('Visible adjustment exceeded press budget')
                        else:
                            push(path,(module.UP if error>0 else module.DOWN)+[0,0,.028])
                            move(path,2.,module.PLATFORM+[0,0,.17]);self._adjustments+=1
                else:
                    name=module.NAMES[self._phase]
                    self._samples[name]=measured
                    carry(path,module.PAD+[0,0,.023],module.INITIAL[name],module.HALF)
                    self._phase+=1
                    if self._phase<len(module.NAMES):
                        name=module.NAMES[self._phase]
                        carry(path,module.INITIAL[name],module.PAD+[0,0,.023],module.HALF)
                        move(path,35.,module.PAD+[0,0,.15])
                    else:
                        winner=max(self._samples,key=self._samples.get)
                        carry(path,module.INITIAL[winner],module.MAT+[0,0,.02],module.HALF)
                        move(path,2.,module.MAT+[0,0,.17]);self._complete=True
                if path:self._append(path,t);local=0.
        from yam_arena.constants import READY
        values=[self._plans[a].action7(local)[0] if a in self._plans else np.r_[READY,1.] for a in ('left','right')]
        meta={'request_stop':True} if self._complete and local>self._end+3 else {}
        meta['visible_display']=value
        return ActionChunk(actions=[Action(data=pack(*values),meta=meta)],control_hz=1/self._dt)

def policy(**kw):return VisiblePolicy(**kw)
