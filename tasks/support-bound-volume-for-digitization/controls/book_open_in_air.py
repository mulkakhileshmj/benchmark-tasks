"""Execute a 125 degree opening high above the imaging cradle."""
import math
import numpy as np
from task import *
from task import _tcp


def demo():
    plans = demo_plan()
    for side, arm in enumerate(("left", "right")):
        start = g.START_ANGLES[side]
        goal = (27.5, 152.5)[side]
        clear_height = g.SPINE_HEIGHT + .085
        staging = _tcp(side, start, .265, clear_height)
        flat = _tcp(side, goal, .265, clear_height)
        plans[arm] = plans[arm][:5] + [
            W(2., staging, C, "raise clear of support faces", roll=math.radians(start - 90)),
            W(3., flat, C, "open past specimen limit", roll=math.radians(goal - 90)),
            W(2., flat, C, "hold open", roll=math.radians(goal - 90)),
        ]
    return plans
