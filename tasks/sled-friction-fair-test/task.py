'Use the two-lane track to determine which sled base has higher friction. Put that sled alone on the red mat. The red button toggles the shared start gate.'
"""Native gravity comparison with geometry selected by CONFIG."""
import math
import numpy as np
from review_native_common import *
from review_native_common import Display

SCENE='station';SCORE_OVER='end';SUCCESS_HOLD_S=1.
CONFIG = 74
THETA=math.radians(15 if CONFIG==63 else 30)
X0=.17;X1=.35;CENTER_Y=.11
START_X=.195
LANES=(.165,.055)
NAMES=('P','Q','R') if CONFIG==63 else ('light','heavy')
INITIAL={n:np.array([.10,.14-i*.09,TOP+.025]) for i,n in enumerate(NAMES)}
BIN=np.array([.23,-.01,TOP+.002])
BUTTON=np.array([.14,-.12,TOP])
VALUE={'P':.92,'Q':.70,'R':.50} if CONFIG==63 else {'light':.30,'heavy':.40}
TRUTH=max(VALUE,key=VALUE.get)

def height(x):return TOP+.008+max(0.,X1-x)*math.tan(THETA)

def ramp():
    vertices=[(x,y,z) for x in (X0,X1) for y in (-.11,.11)
              for z in (height(x)-TOP-.01,height(x)-TOP)]
    faces=((0,4,6,2),(1,3,7,5),(0,1,5,4),(2,6,7,3),(0,2,3,1),(4,5,7,6))
    points=[(x-.26,y,z) for x,y,z in vertices]
    return dict(pos=(0,0,0),size=(X1-X0,.22,height(X0)-TOP),shape='mesh',vertices=points,faces=faces,color=(.6,.48,.3),friction=.15)

OBJECTS=[Compound('ramp',pos=(.26,CENTER_Y,TOP),size=(.18,.22,.15),parts=(ramp(),)),
         solid('runout',(.44,CENTER_Y,TOP+.004),(.18,.23,.008),free=False,friction=.15),
         solid('end_stop',(.53,CENTER_Y,TOP+.045),(.012,.23,.09),free=False,color=(.12,.12,.12),friction=1.2),
         solid('red_mat',BIN,(.11,.10,.004),free=False,color=(.8,.05,.03)),button('gate_button',BUTTON)]
for name in NAMES:
    if CONFIG==63:
        mass=.15;r=.025;length=.06;k=VALUE[name]
        axial=k*mass*r*r;transverse=mass*(6*k*r*r+length*length)/12
        OBJECTS.append(labelled(Compound(name,pos=tuple(INITIAL[name]),size=(.05,.06,.05),mass=mass,
            inertia=(transverse,axial,transverse),parts=(cylinder((0,0,0),r,length,'Y',friction=1.2),)),name))
    else:
        OBJECTS.append(solid(name,INITIAL[name],(.05,.05,.04),mass=.15 if name=='light' else .25,friction=VALUE[name]))
gate_height=height(START_X)+.027
OBJECTS.append(Articulated('gate',pos=(START_X+.032,CENTER_Y,gate_height),size=(.018,.23,.13),
    fixed_base=True,parts=(box((0,0,-.15),(.008,.23,.01)),),
    links=(dict(name='bar',pos=(0,0,0),mass=.08,parts=(box((0,0,0),(.008,.22,.060),(.9,.35,.03)),)),),
    joints=(joint('lift','Y','root','bar',(0,0,0),0.,.25,stiffness=2000,damping=20,max_force=200),)))
_runs=[];_active=None;_pressed=False;_open=False;_score=0.;_diag={};_bad=False

def reset_history():
    global _runs,_active,_pressed,_open,_score,_diag,_bad
    _runs=[];_active=None;_pressed=False;_open=False;_score=0.;_diag={};_bad=False

def observe(states,t,base):
    global _pressed,_open,_active,_score,_diag,_bad
    if not all(np.isfinite(s.pos).all() and np.isfinite(s.lin_vel).all() for s in states.values()):
        raise RuntimeError('Invalid native body state')
    down=coordinate(base,'gate_button','press')<-.004
    if down and not _pressed:
        _open=not _open
        motor(base,'gate','lift',.24 if _open else 0.)
        if _open:
            starters=[n for n in NAMES if abs(states[n].pos[0]-START_X)<.025
                      and min(abs(states[n].pos[1]-y) for y in LANES)<.015
                      and np.linalg.norm(states[n].lin_vel)<.04
                      and hands_clear(base,states[n].pos)]
            _active=dict(start=float(t),starters=starters,cross={},stops={})
            _runs.append(_active)
    _pressed=down
    if _active:
        for n in _active['starters']:
            s=states[n]
            if s.low[2]<TOP-.01:_bad=True
            measured=_active['cross'] if CONFIG==63 else _active['stops']
            if n in measured:
                continue
            if not hands_clear(base,s.pos,.045):
                _active.setdefault('invalid',[]).append(n)
            if CONFIG==74 and s.high[0]>=.524:
                _active.setdefault('invalid',[]).append(n)
            if s.pos[0]>=X1+.06:_active['cross'].setdefault(n,float(t)-_active['start'])
            if t-_active['start']>.8 and s.pos[0]>X1 and np.linalg.norm(s.lin_vel)<.006:
                _active['stops'].setdefault(n,float(s.pos[0]-START_X))
    slower=set();valid_measurements=0
    for run in _runs:
        data=run['cross'] if CONFIG==63 else run['stops']
        data={n:v for n,v in data.items() if n not in run.get('invalid',[]) and states[n].pos[0]<.62}
        valid_measurements+=len(data)
        for a in data:
            for b in data:
                if a!=b and ((data[a]>data[b]+.02) if CONFIG==63 else (data[a]<data[b]-.03)):
                    slower.add((a,b))
    # A later direct comparison can establish the remaining relation by a chain.
    changed=True
    while changed:
        old=len(slower)
        slower|={(a,c) for a,b in tuple(slower) for x,c in tuple(slower) if b==x and a!=c}
        changed=len(slower)>old
    proven=all((TRUTH,n) in slower for n in NAMES if n!=TRUTH)
    binned=[n for n in NAMES if in_region(states[n],BIN,.04) and hands_clear(base,states[n].pos)]
    partial=(.75 if CONFIG==63 else .5) if proven else (.5 if CONFIG==63 else .25) if valid_measurements else 0.
    _score=0. if _bad else 1. if proven and binned==[TRUTH] else partial
    _diag=dict(score=_score,comparisons=sorted(slower),runs=_runs,binned=binned,failure=_bad)

def score(states):return _score
def success(states):return _score==1.
def distance(states):return 1.-_score
def diagnostics():return _diag

def demo():
    """A fixed-layout action-only comparison sequence; final selection must use observed race outcome."""
    path=[]
    half=.025 if CONFIG==63 else .02
    for name,lane in zip(NAMES,LANES):
        carry(path,INITIAL[name],[START_X,lane-.020,height(START_X)+half],half)
    push(path,BUTTON+[0,0,.028])
    move(path,2.,[.10,-.10,TOP+.22]);move(path,4.,[.10,-.10,TOP+.22])
    # No hidden type is used to pick a winner. This measurement-only reference
    # intentionally earns partial credit until a vision selection policy is tested.
    return {'left':path}
