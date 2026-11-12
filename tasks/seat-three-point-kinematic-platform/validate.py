from pathlib import Path
from types import SimpleNamespace
import math,sys
import numpy as np
root=Path(__file__).resolve().parent
sys.path[:0]=[str(root/'runtime'),str(root)]
import task,seating_contact
from yam_arena.demo import ArmPlan
from yam_arena import station

for spec in task.OBJECTS:spec._check()
flags,gaps=seating_contact.grouped_contacts([0.,.0001,.0003],[1,1,1],[0,1,2],.0002)
assert flags.tolist()==[True,True,False]
flags,_=seating_contact.grouped_contacts([0.,0.],[1,1,0],[0,1,2],.0002)
assert flags.tolist()==[True,True,False]
flags,_=seating_contact.grouped_contacts([float('nan')],[1],[0],.0002)
assert not flags.any()
try:seating_contact.grouped_contacts([0.],[2],[0],.0002)
except ValueError:pass
else:raise AssertionError('Invalid native contact bounds must fail closed')

state=SimpleNamespace(resting=lambda *args:True,low=np.array([0,0,task.TOP+.014]))
states={'mount_plate':state}
task.reset_history()
task._pose=np.array([.26,0,task.SEAT_Z,0,0,0,1.])
task._contacts=np.ones(3,dtype=bool);task._open=True
assert task.score(states)==.75
task._clear=True
assert task.score(states)==.9
task._qualified_since=1.;task._observation_time=1.95
assert task.score(states)==.9
task._observation_time=2.
assert task.score(states)==1.
for index in range(3):
    task._contacts[:]=True;task._contacts[index]=False
    assert task.score(states)==.5
task._contacts[:]=True;task._bad=True
assert task.score(states)==0.

# Sphere to the authored 45 degree face and flat tangencies at the stated height.
assert abs((task.CENTRE-.010)/math.sqrt(2)-task.R)<1e-12
assert abs(task.CENTRE-task.flat_top-task.R)<1e-12
assert abs(task.SEAT_Z-task.TOP-.04214213562373095)<1e-12
task.reset_history()
for arm,points in task.demo().items():
    plan=ArmPlan(points,station.BASES[arm])
    for t in np.arange(.05,plan.duration+.1,.05):plan.action7(float(t))
    print(arm,plan.worst_ik_mm,plan.worst_ik_deg,plan.duration)
    assert plan.worst_ik_mm<2 and plan.worst_ik_deg<1
print('PASS independent datum contacts and full-score hold and release and geometry and reach')
