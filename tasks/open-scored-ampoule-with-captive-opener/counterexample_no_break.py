"""Cover the cap but do not perform the transverse break motion. Native control.

This file changes only robot waypoints. It does not alter apparatus physics.
"""
from task import *
import task as original
def demo():
    plans=original.demo()
    right=[]
    for waypoint in plans['right']:
        right.append(waypoint)
        if waypoint.label=='seat cap inside opener':
            right.append(Waypoint(6.,waypoint.pos,C,'covered but not fractured'))
            break
    else:
        raise RuntimeError('Missing fixed insertion waypoint')
    return dict(left=plans['left'],right=right)
