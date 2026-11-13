"""Independent rigid contact experiment and official YAM trajectory check."""
import json
import math
import os
from pathlib import Path
import sys
from xml.etree import ElementTree as X

import mujoco as mj
import numpy as np

import design as d
from scoring import Score, Sample


def vector(values):
    return " ".join(str(float(v)) for v in values)


def shapes(body, parts, mass=None):
    volumes = [math.pi*p['radius']**2*p['height'] if 'radius' in p
               else math.prod(p['size']) for p in parts]
    for p, volume in zip(parts, volumes):
        geom = X.SubElement(body, 'geom', pos=vector(p['pos']))
        if mass:
            geom.set('mass', str(mass*volume/sum(volumes)))
        if p.get('shape') == 'cylinder':
            geom.set('type', 'cylinder')
            geom.set('size', vector([p['radius'],p['height']/2]))
            if p['axis']=='Y':
                geom.set('euler', vector([math.pi/2,0,0]))
        else:
            geom.set('type', 'box')
            geom.set('size',vector(np.array(p['size'])/2))
            geom.set('euler', vector([0,0,p.get('rot_z',0)]))


def mechanism():
    xml=X.fromstring('''<mujoco><compiler angle="radian"/><option timestep=".0005"/>
        <default><geom friction=".4 .002 .0001" solref=".002 1"
        solimp=".99 .999 .0001" margin=".00002"/></default><worldbody/></mujoco>''')
    world=xml.find('worldbody')
    shapes(world,d.ring(.0245,.013,.021,.003))
    plug=X.SubElement(world,'body',name='stopper')
    X.SubElement(plug,'joint',name='lift',type='slide',axis='0 0 1',range='0 .035',damping='.12')
    shapes(plug,d.plug(),.039)
    ball=X.SubElement(world,'body',name='ball',pos=vector(d.BALL))
    X.SubElement(ball,'joint',name='insertion',type='slide',axis='0 1 0',range='-.016 0',damping='.8')
    X.SubElement(ball,'geom',type='sphere',size='.005',mass='.008',contype='0',conaffinity='0')
    rod=X.SubElement(ball,'body',name='rod')
    X.SubElement(rod,'joint',name='tilt',axis='1 0 0',range=vector(np.radians([-3,32])),
                 stiffness='.5',damping='.002')
    shapes(rod,d.rod(),.018)
    return mj.MjModel.from_xml_string(X.tostring(xml,encoding='unicode'))


def trial(mode):
    m=mechanism()
    s=mj.MjData(m)
    s.qpos[1]=-d.PARK
    if mode=='underneath':
        s.qpos[0]=.011
    body=m.body('stopper').id
    rod=m.body('rod').id
    peak=proof=0.
    contacts=0
    seated=None
    captured_at_seat=False
    stroke_score=Score()
    for i in range(14600):
        t=i*m.opt.timestep
        insertion=-d.PARK*(1-np.clip((t-.2)/1.,0,1)) if mode!='withdrawn' else -d.PARK
        angle=float(np.interp(t,[0,1.5,2.7,3.1,4.3,7.3],[0,0,d.OPEN_ANGLE,d.OPEN_ANGLE,0,0]))
        h,slide,tilt=s.qpos
        vh,vs,vt=s.qvel
        s.qfrc_applied[1]=np.clip(800*(insertion-slide)-3*vs,-4,4)
        s.qfrc_applied[2]=np.clip(10*(angle-tilt)-.04*vt,-.5,.5)
        s.xfrc_applied[body,2]=1. if 5.6<=t<6.2 else 0.
        if mode=='underneath' and t<1.3:
            # An installation error performed on the very same rig: hold the
            # plug high while feeding the pivot, then let the eye rest on it.
            s.xfrc_applied[body,2]=np.clip(.039*9.81+300*(.011-h)-2*vh,-2,2)
        mj.mj_step(m,s)
        if 1.5<=t<3.2:
            peak=max(peak,h)
            for contact in s.contact:
                if set(m.geom_bodyid[[contact.geom1,contact.geom2]])=={body,rod}:
                    contacts+=1
        if 5.5<=t<5.6:
            seated=h
            captured_at_seat=d.gauge(tilt,slide,h)
        if 5.6<=t<6.2:
            proof=max(proof,h)
        if i%100==0 and t<5.6:
            stroke_score.update(Sample(t,slide,tilt,h,mode!='underneath' or t>=1.3,t>4.4,abs(vh)))
    return dict(peak_stroke_mm=1000*peak,closed_gap_mm=1000*seated,
                proof_lift_mm=1000*proof,rod_stopper_contacts=contacts,
                captured_at_seat=bool(captured_at_seat),stroke_score=stroke_score.value)


def main():
    root=Path(__file__).resolve().parent
    assisted=Score()
    for i,(slide,angle,h,clear) in enumerate([
        (0,0,0,True),(0,.3,.007,False),(0,.4,.010,True),(0,0,0,True)
    ]+[(0,0,0,True)]*22):
        assisted.update(Sample(i*.05,slide,angle,h,clear,clear,0))
    assert assisted.value==.25
    report={name:trial(name) for name in ('captured','underneath','withdrawn')}
    print(json.dumps(report,indent=2),flush=True)
    assert report['captured']['stroke_score']==1
    assert report['captured']['proof_lift_mm']<6
    for name in ('underneath','withdrawn'):
        assert report[name]['proof_lift_mm']>25 and report[name]['stroke_score']==0
    assert report['underneath']['peak_stroke_mm']>4
    assert report['withdrawn']['peak_stroke_mm']<.1
    assert report['captured']['closed_gap_mm']<.6
    assert report['underneath']['closed_gap_mm']>5
    yam=os.environ.get('YAM_SOURCE','/mnt/d/robocurve - evals/yam-isaac-lab-arena')
    sys.path[:0]=[str(root),str(root/'runtime'),yam]
    from yam_arena.task_file import load
    from yam_arena.demo import ArmPlan
    from yam_arena import station
    task=load(root/'task.py')
    task.module.WASTE._check()
    for arm,points in task.demo().items():
        plan=ArmPlan(points,station.BASES[arm])
        samples=np.arange(.05,plan.duration+.05,.05)
        for t in samples:
            plan.action7(float(t))
        report[arm+'_ik']=dict(worst_mm=plan.worst_ik_mm,worst_deg=plan.worst_ik_deg,
                               steps=len(samples),seconds=plan.duration)
        print(report[arm+'_ik'],flush=True)
        assert plan.worst_ik_mm<2 and plan.worst_ik_deg<1
    (root/'cpu_validation.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':
    main()
