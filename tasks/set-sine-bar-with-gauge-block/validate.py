from pathlib import Path
import sys,numpy as np,math
root=Path(__file__).resolve().parent
sys.path[:0]=[str(root/'runtime'),str(root)]
import task
from yam_arena.demo import ArmPlan
from yam_arena import station
from types import SimpleNamespace
block=SimpleNamespace(pos=np.array([task.TARGET_X,0,task.TOP+.0405]),low=np.array([task.TARGET_X-.02,-.02,task.TOP+.008]),quat_xyzw=np.array([0.,0.,0.,1.]),resting=lambda *args:True)
task.reset_history();task._angle=math.radians(30);task._open=True;task._clear=True
assert task.score({'support_block_65':block})==.5
task._supported=True;assert task.score({'support_block_65':block})==1.
task._clear=False;assert task.score({'support_block_65':block})==.75
task._clear=True;block.low[2]+=.035;assert task.score({'support_block_65':block})<1.,'Stacking blocks must not count'
task.reset_history()
for arm,points in task.demo().items():
 plan=ArmPlan(points,station.BASES[arm])
 for time in np.arange(.05,plan.duration+.1,.05):plan.action7(float(time))
 print(arm,plan.worst_ik_mm,plan.worst_ik_deg,plan.duration)
 assert plan.worst_ik_mm<2 and plan.worst_ik_deg<1
print('PASS sine geometry and support gates and reach')
