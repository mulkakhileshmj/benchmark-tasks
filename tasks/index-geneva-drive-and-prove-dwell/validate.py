from pathlib import Path
import sys,math,numpy as np
root=Path(__file__).resolve().parent
sys.path[:0]=[str(root/'runtime'),str(root)]
import task,geometry
from yam_arena.demo import ArmPlan
from yam_arena import station
task.FIXTURE._check()
assert len(geometry.star_parts())<=512
for part in geometry.star_parts()+geometry.driver_parts():
 assert np.isfinite(part['size']).all() and min(part['size'])>0
 if part.get('shape')=='mesh':
  assert np.isfinite(part['vertices']).all()
  assert min(index for face in part['faces'] for index in face)>=0
  assert max(index for face in part['faces'] for index in face)<len(part['vertices'])
task.reset_history();assert task.score({})==0.
task._indices=2;task._total=np.array([4*math.pi,-math.pi]);task._open=True;task._clear=True
assert task.score({})==1.
task._bad=True;assert task.score({})==0.
task.reset_history()
for arm,points in task.demo().items():
 plan=ArmPlan(points,station.BASES[arm])
 for time in np.arange(.05,plan.duration+.1,.05):plan.action7(float(time))
 print(arm,plan.worst_ik_mm,plan.worst_ik_deg,plan.duration)
 assert plan.worst_ik_mm<2 and plan.worst_ik_deg<1
print('PASS Geneva geometry and score gates and reach')
