'The digital angle gauge may not read true; its reading appears on the red display, in degrees. The grey granite block marked REF is exactly level. Make the platform truly level, to within 0.15 degrees, using the UP and DOWN buttons of its levelling foot.'
"""Level the platform using its buttons and a gauge whose zero may be biased."""
import math
import numpy as np
from review_native_common import *
from review_native_common import Display

SCENE='station';SCORE_OVER='end';SUCCESS_HOLD_S=1.
PLATFORM=np.array([.35,-.01,TOP+.065])
REF=np.array([.19,.18,TOP+.025])
GAUGE=np.array([.19,.08,TOP+.025])
UP=np.array([.18,-.10,TOP]);DOWN=np.array([.18,-.18,TOP])
INITIAL_DEG=1.5;ZERO_ERROR=.8;STEP=.25;TOL=.15
OBJECTS=[solid('reference',REF,(.09,.08,.05),free=False),
         solid('gauge',GAUGE,(.05,.05,.05),mass=.16,color=(.9,.7,.05)),
         button('up',UP),button('down',DOWN)]
OBJECTS.append(Articulated('platform',pos=tuple(PLATFORM),size=(.20,.14,.025),fixed_base=True,
    parts=(box((0,0,-.015),(.20,.14,.015),(.25,.25,.25)),),
    links=(dict(name='surface',pos=(0,0,0),quat_xyzw=(0,math.sin(math.radians(INITIAL_DEG)/2),0,math.cos(math.radians(INITIAL_DEG)/2)),
                mass=.5,parts=(box((0,0,0),(.20,.14,.012)),)),),
    initial_joints=(('tilt',math.radians(INITIAL_DEG)),),
    joints=(joint('tilt','Y','root','surface',(0,0,0),-4.,4.,kind='revolute',
                  stiffness=4.,damping=.4,max_force=4.,target=INITIAL_DEG),)))
_display=Display('angle_display',(.12,.0,TOP+.012));OBJECTS.append(_display.asset)
_presses=[];_down={'up':False,'down':False};_target=INITIAL_DEG
_readings=[];_hold=None;_score=0.;_diag={};_stable=None;_next=0.

def reset_history():
    global _presses,_down,_target,_readings,_hold,_score,_diag,_stable,_next
    _presses=[];_down={'up':False,'down':False};_target=INITIAL_DEG
    _readings=[];_hold=None;_score=0.;_diag={};_stable=None;_next=0.

def observe(states,t,base):
    global _target,_stable,_next,_hold,_score,_diag
    for name,direction in [('up',-1),('down',1)]:
        pressed=coordinate(base,name,'press')<-.004
        if pressed and not _down[name]:
            _presses.append(direction)
            _target=float(np.clip(_target+direction*STEP,-4.,4.))
            motor(base,'platform','tilt',math.radians(_target))
        _down[name]=pressed
    gauge=states['gauge'];rot=quat_xyzw_to_matrix(gauge.quat_xyzw)
    observed_pitch=math.degrees(math.atan2(-rot[2,0],math.hypot(rot[0,0],rot[1,0])))
    _display.show(observed_pitch+ZERO_ERROR)
    over_ref=np.linalg.norm(gauge.pos[:2]-REF[:2])<.012 and abs(gauge.low[2]-(TOP+.05))<.003
    over_platform=np.linalg.norm(gauge.pos[:2]-PLATFORM[:2])<.045 and abs(gauge.low[2]-(TOP+.071))<.009
    where='reference' if over_ref else 'platform' if over_platform else None
    stable=where is not None and np.linalg.norm(gauge.lin_vel)<.004 and np.linalg.norm(gauge.ang_vel)<.04 and hands_clear(base,gauge.pos)
    if stable:
        if _stable is None or _stable[0]!=where:_stable=(where,float(t))
        if t-_stable[1]>=1. and t>=_next:
            _readings.append(dict(t=float(t),where=where,value=_display.transcribed(),presses=len(_presses)))
            _next=float(t)+.5
    else:_stable=None
    calibrated=any(r['where']=='reference' for r in _readings)
    measured=any(r['where']=='platform' and r['presses']==len(_presses) for r in _readings)
    true_tilt=math.degrees(coordinate(base,'platform','tilt'))
    base_score=.75 if calibrated and measured else .5 if measured else .25 if calibrated else 0.
    clear=all(hands_clear(base,p) for p in (PLATFORM,UP+[0,0,.02],DOWN+[0,0,.02]))
    if calibrated and measured and abs(true_tilt)<=TOL and clear:
        if _hold is None:_hold=float(t)
        if t-_hold>=1.:base_score=1.
    else:_hold=None
    wasted=len(_presses)-abs(sum(_presses));extra=max(0,len(_presses)-12)
    _score=base_score*max(0.,1.-.1*wasted-.1*extra)
    _diag=dict(score=_score,display=_display.text,readings=_readings,presses=len(_presses),wasted_presses=wasted,calibrated=calibrated)

def score(states):return _score
def success(states):return _score==1.
def distance(states):return 1.-_score
def diagnostics():return _diag

def demo():
    """Fixed-layout calibration prefix. The measured adjustment requires a visible-display policy."""
    p=[]
    carry(p,GAUGE,REF+[0,0,.05],.025)
    move(p,2.,REF+[0,0,.16])
    return {'left':p}
