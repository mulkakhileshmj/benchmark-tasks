"""CPU checks. IK uses the official local YAM source and no simulation."""
import argparse
import math
import json
from pathlib import Path
import sys
import numpy as np
import geometry as g
from scoring import SupportScore
from support_contacts import grouped

def scorer_tests():
    good=dict(t=0.,opening_deg=110.,angle_errors_deg=[0.,0.],root_error_m=0.,root_tilt_deg=0.,
        root_yaw_deg=0.,contacts=[True]*4,released=True,at_rest=True,on_table=True,spine_clear=True)
    def run(change=None,steps=23):
        scorer=SupportScore();values=[]
        for k in range(steps):
            st={**good,'t':k/20.}
            if change:st.update(change(k))
            values.append(scorer.update(**st))
        return scorer,values
    s,values=run();assert values[19]==.75 and values[20]==1. and not s.bad
    for i in range(4):
        contacts=[True]*4;contacts[i]=False
        s,values=run(lambda k:dict(contacts=contacts));assert max(values)<1.
    for contacts in ([False]*4,[True,True,False,False],[True,False,True,False]):
        s,values=run(lambda k:dict(contacts=contacts));assert max(values)<1.
    for kw in (dict(released=False),dict(at_rest=False),dict(spine_clear=False),dict(root_error_m=.006),
               dict(root_tilt_deg=4.),dict(root_yaw_deg=6.),dict(angle_errors_deg=[4.,0.]),
               dict(opening_deg=60.),dict(finite=False),dict(root_error_m=float('nan'))):
        s,values=run(lambda k,kw=kw:kw);assert max(values)<1.
    s,values=run(lambda k:dict(opening_deg=116.) if k==0 else {});assert s.bad and max(values)==0.
    s,values=run(lambda k:dict(on_table=False) if k==0 else {});assert s.bad and max(values)==0.
    s,values=run(lambda k:dict(released=False) if k==15 else {});assert values[-1]<1.
    s,values=run(lambda k:dict(t=(k/20.+1. if k>=15 else k/20.)));assert values[-1]<1.
    contacts,gaps=grouped([.0001,.0003,-.00001],[1,1,1,0],[0,1,2,3],.0002)
    assert contacts.tolist()==[True,False,True,False]
    try:grouped([float('nan')],[1],[0],.0002)
    except ValueError:pass
    else:raise AssertionError('Nonfinite contact buffers must error')
    try:grouped([0.],[2],[0],.0002)
    except ValueError:pass
    else:raise AssertionError('Invalid contact buffers must fail closed')
    return 24

def geometry_tests():
    assert g.opening(g.START_ANGLES)==60 and g.opening(g.GOAL_ANGLES)==110
    # Opening must not depend on the volume's common yaw or tilt.
    for root in (g.rx(45.),np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])@g.rx(33.)):
        assert math.isclose(g.relative_opening(root@g.rx(35.),root@g.rx(145.)),110.,abs_tol=1e-9)
        assert g.relative_opening(root@g.rx(10.),root@g.rx(140.))>115.
    for angles in (g.START_ANGLES,g.GOAL_ANGLES):
        for side,a in enumerate(angles):
            for s in (0.,.09,.18):
                p=g.underside(side,a,s)
                assert p[2]+g.SPINE_HEIGHT>0.
                assert abs(p[1])>.010
            mesh=g.support_piece(side,a,0.,.09)
            pts=np.asarray(mesh['vertices'])+mesh['pos']
            assert np.isfinite(pts).all() and np.min(pts[:,2])==0.
            assert math.isclose(np.max(pts[:,0])-np.min(pts[:,0]),.09)
    # Source and destination supports have no X overlap.
    assert g.TARGET_X-g.LENGTH/2>g.SOURCE_X+g.LENGTH/2
    return 15

def ik_tests():
    sys.path.insert(0,str(Path(__file__).parent/'runtime'))
    from yam_arena.task_file import load
    from yam_arena.demo import ArmPlan
    from yam_arena import station
    task=load(Path(__file__).parent/'task.py')
    results={}
    for arm,waypoints in task.demo().items():
        plan=ArmPlan(waypoints,station.BASES[arm])
        for t in np.arange(.05,plan.duration+.051,.05):plan.action7(float(t))
        results[arm]=dict(duration_s=plan.duration,worst_ik_mm=plan.worst_ik_mm,worst_ik_deg=plan.worst_ik_deg)
        if plan.worst_ik_mm>=2. or plan.worst_ik_deg>=1.:
            raise AssertionError(f'Unreachable draft trajectory: {arm}: {results[arm]}')
    return results

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--ik',action='store_true');args=parser.parse_args()
    result=dict(cpu_scoring_cases=scorer_tests(),geometry_cases=geometry_tests(),native_simulation='not run')
    if args.ik:result['ik']=ik_tests()
    print(json.dumps(result,indent=2))
