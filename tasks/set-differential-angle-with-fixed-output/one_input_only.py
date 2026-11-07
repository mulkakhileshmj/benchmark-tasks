"""Counterfactual that moves only one input and must fail the centre constraint."""
from pathlib import Path
from yam_arena.task_file import load
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_CLOSED as C
_base=load(Path(__file__).with_name('task.py'))
SCENE=_base.scene;OBJECTS=_base.objects;SUCCESS_HOLD_S=1.;SCORE_OVER='end'
score=_base.score;success=_base.success;distance=_base.distance
observe=_base.module.observe;reset_history=_base.module.reset_history
def demo():
 plans=_base.demo()
 start=plans['left'][1].pos
 plans['left']=plans['left'][:3]+[Waypoint(12.,start,C)]
 return plans
