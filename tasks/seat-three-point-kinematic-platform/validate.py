from pathlib import Path
import sys,numpy as np
from types import SimpleNamespace
root=Path(__file__).resolve().parent
sys.path[:0]=[str(root/'runtime'),str(root)]
import task
from yam_arena.demo import ArmPlan
from yam_arena import station
for spec in task.OBJECTS:spec._check()
task.reset_history();task._pose=np.array([.26,0,task.SEAT_Z,0,0,0,1.]);task._forces=np.ones(3);task._open=True
state=SimpleNamespace(resting=lambda *args:True,low=np.array([0,0,task.TOP+.014]))
assert task.score({'mount_plate':state})==.75
task._clear=True;assert task.score({'mount_plate':state})==1.
task._forces[2]=0;assert task.score({'mount_plate':state})==.5
task._bad=True;assert task.score({'mount_plate':state})==0.
task.reset_history()
for arm,points in task.demo().items():
 plan=ArmPlan(points,station.BASES[arm])
 for time in np.arange(.05,plan.duration+.1,.05):plan.action7(float(time))
 print(arm,plan.worst_ik_mm,plan.worst_ik_deg,plan.duration)
 assert plan.worst_ik_mm<2 and plan.worst_ik_deg<1
print('PASS mount release and three support gates and reach')
