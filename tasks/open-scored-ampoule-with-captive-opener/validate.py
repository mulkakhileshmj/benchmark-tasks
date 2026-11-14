"""CPU checks of score semantics and authored geometry and actual YAM IK.

This program is not a physics trial. A passing report never replaces native
contact and fracture calibration and full demo and model evidence.
"""
from dataclasses import replace,asdict
from pathlib import Path
import ast
import hashlib
import json
import sys
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'runtime'))
from scoring import Scorer,Sample

def sample(t=0.,**kw):
    s=Sample(t,0.,False,False,0.,True,True,True,True,neck_scored=True)
    return replace(s,**kw)

def check_scoring():
    names=[]
    def check(name,condition):
        assert condition,name
        names.append(name)
    m=Scorer();check('unscored fracture fails',m.update(sample(neck_broken=True,neck_scored=False))==0 and m.failed)
    m=Scorer();check('stationary unscored coverage only partial',m.update(sample(cap_in_opener=True,cap_covered=True,neck_scored=False))==.15)
    m=Scorer();check('untouched start zero',m.update(sample())==0)
    check('covering without break only partial',m.update(sample(.05,cap_in_opener=True,cap_covered=True))==.25)
    check('unsafe open fails',Scorer().update(sample(neck_gap_m=.02))==0)
    for flag in ('cap_contact','rim_contact','cap_escape','dropped','opener_outside_table'):
        m=Scorer();check(flag+' fails',m.update(sample(**{flag:True}))==0 and m.failed)
        check(flag+' remains failed',m.update(sample(.05,neck_gap_m=.02,cap_in_opener=True,cap_covered=True))==0)
    m=Scorer();check('tilted body fails',m.update(sample(body_upright_deg=16))==0 and m.failed)
    m=Scorer();check('NaN fails',m.update(sample(neck_gap_m=float('nan')))==0 and m.failed)
    m=Scorer();check('missing contact monitor fails',m.update(sample(monitor_valid=False))==0 and m.failed)
    m=Scorer();check('early escaped fracture before 12 mm fails',m.update(sample(neck_gap_m=.0006))==0 and m.failed)
    m=Scorer();check('native break without visible gap requires capture',m.update(sample(neck_broken=True))==0 and m.failed)
    check('separation without recorded native break is not success',Scorer().update(sample(neck_gap_m=.02,cap_in_opener=True,cap_covered=True))==.25)
    m=Scorer();s=sample(neck_gap_m=.02,cap_in_opener=True,cap_covered=True,neck_broken=True)
    check('initial release needs dwell',m.update(s)==.9)
    for k in range(1,20):check('hold '+str(k),m.update(replace(s,time=k*.05))==.9)
    check('full after one second',m.update(replace(s,time=1.))==1)
    m=Scorer();m.update(s)
    m.update(replace(s,time=.05,hands_open_clear=False))
    check('hold resets when held',m.update(replace(s,time=.1))==.9)
    m=Scorer();m.update(s)
    check('escaped cap fails permanently',m.update(replace(s,time=.05,cap_in_opener=False))==0 and m.failed)
    for name in ('body_in_holder','opener_in_rack'):
        check(name+' required',Scorer().update(replace(s,**{name:False}))==.5)
    check('slow required',Scorer().update(replace(s,bodies_slow=False))==.75)
    check('release required',Scorer().update(replace(s,hands_open_clear=False))==.75)
    return names

def check_geometry_and_ik():
    from yam_arena.task_file import load
    from yam_arena.demo import ArmPlan
    from yam_arena import station
    t=load(ROOT/'task.py');m=t.module
    assert np.allclose(m.BODY_ROOT+[0,0,.020],m.CAP_ROOT)
    assert m.BODY.parts[0]['size'][2]<=.0401
    # No initial ampoule solid intersects the holder floor or holder walls.
    assert abs(m.BODY_ROOT[2]-.020-m.TOP-.004)<1e-9
    for item in m.OBJECTS:
        item._check()
        for p in item.parts:
            if not p.get('visual_only'):assert p['contact_offset']<=.0001
    assert m.OBJECTS.index(m.BODY)<m.OBJECTS.index(m.CAP)
    assert m.CAP.parts[0]['size'][0] <= .00901
    assert m.OPENER.parts[0]['size'][2] <= .08001
    # Blade-like tiny scales are not enlarged for robot convenience.
    assert abs(sum(l['mass'] for l in m.OPENER.links)+m.OPENER.mass-.022)<1e-9
    from ampoule_native import BREAK_TORQUE_NM,BREAK_FORCE_N
    assert BREAK_TORQUE_NM==.20 and BREAK_FORCE_N==20.
    # The controller function must not read task state or write simulator state.
    node=next(x for x in ast.parse((ROOT/'task.py').read_text()).body if isinstance(x,ast.FunctionDef) and x.name=='demo')
    forbidden={'_expert_states','_expert_joints','set_pose','write_root_pose_to_sim','write_root_state_to_sim'}
    assert not any(isinstance(x,ast.Name) and x.id in forbidden for x in ast.walk(node))
    measured={}
    for arm,points in t.demo().items():
        plan=ArmPlan(points,station.BASES[arm])
        for time in np.arange(.05,plan.duration+.1,.05):plan.action7(float(time))
        measured[arm]=dict(duration_s=plan.duration,worst_ik_mm=plan.worst_ik_mm,worst_ik_deg=plan.worst_ik_deg)
        assert plan.duration+3<120
        assert plan.worst_ik_mm<2 and plan.worst_ik_deg<1,measured
    inventory=[]
    for item in m.OBJECTS:
        entry=asdict(item)
        entry['free']=item.free
        inventory.append(entry)
    (ROOT/'scene_inventory.json').write_text(json.dumps(dict(table_top=m.TOP,objects=inventory),indent=2)+'\n')
    return measured

if __name__=='__main__':
    tests=check_scoring()
    ik=check_geometry_and_ik()
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(ROOT.rglob('*.py')) if '__pycache__' not in p.parts}
    report=dict(status='CPU checks passed only',score_checks=tests,ik=ik,source_files=hashes,
        native_physics_validated=False,physical_break_calibration_validated=False,
        demo_score=None,model_score=None,media=None)
    (ROOT/'cpu_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(score_checks=len(tests),ik=ik,native_physics_validated=False),indent=2))
