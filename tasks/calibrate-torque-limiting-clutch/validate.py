from pathlib import Path
import sys,numpy as np
root=Path(__file__).resolve().parent
sys.path[:0]=[str(root/'runtime'),str(root)]
import task
from yam_arena.demo import ArmPlan
from yam_arena import station
task.FIXTURE._check()
task.reset_history();assert task.score({})==0.
task._light=True;assert task.score({})==.5
task._heavy=True;assert task.score({})==.75
task._open=True;task._clear=True;assert task.score({})==1.
task._bad=True;assert task.score({})==0.
task.reset_history()
for arm,points in task.demo().items():
 plan=ArmPlan(points,station.BASES[arm])
 for time in np.arange(.05,plan.duration+.1,.05):plan.action7(float(time))
 print(arm,plan.worst_ik_mm,plan.worst_ik_deg,plan.duration)
 assert plan.worst_ik_mm<2 and plan.worst_ik_deg<1
 assert plan.duration+3<32.5
print('PASS clutch score gates and reach')
