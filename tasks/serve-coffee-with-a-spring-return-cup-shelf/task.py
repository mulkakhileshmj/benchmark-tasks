"""Make one coffee and return the filled mug to its own saucer.
Leave the machine and both mugs released.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from geometry import (
    BUTTON_BOX,
    BUTTON_DAMPING,
    BUTTON_WEIGHT_KG,
    BUTTON_NAME,
    BUTTON_REST,
    BUTTON_SPRING_N_PER_M,
    BUTTON_TRAVEL_M,
    COLOR_BUTTON,
    COLOR_CHROME,
    COLOR_CRACK,
    COLOR_MUG,
    COLOR_SAUCER,
    CRACK_RIM,
    CRACK_STRIP,
    HANDLE_BOXES,
    MACHINE_BOXES,
    MACHINE_FRICTION,
    MACHINE_MASS_KG,
    MACHINE_NAME,
    MUG_CRACKED_NAME,
    MUG_CRACKED_START,
    MUG_FLOOR,
    MUG_FRICTION,
    MUG_GOOD_NAME,
    MUG_GOOD_START,
    MUG_WEIGHT_KG,
    MUG_RODS,
    NOZZLE,
    NOZZLE_XY,
    SAUCER_CRACKED_NAME,
    SAUCER_CRACKED_XY,
    SAUCER_GOOD_NAME,
    SAUCER_GOOD_XY,
    SAUCER_HEIGHT_M,
    SAUCER_RADIUS_M,
    SHELF_BOXES,
    SHELF_DAMPING,
    SHELF_WEIGHT_KG,
    SHELF_NAME,
    SHELF_REST,
    SHELF_SPRING_N_PER_M,
    SHELF_TRAVEL_M,
    SUCCESS_HOLD_TIME_S,
    TABLE_TOP,
)
from parts import Slider, part
from yam_arena.task_file import Cylinder
import demo_plan
import strong_runtime as scoring

SCENE = "station"
TOP = TABLE_TOP
SUCCESS_HOLD_S = SUCCESS_HOLD_TIME_S
SCORE_OVER = "end"

# Slide 1: the cup shelf, pulled toward the robot (-x) up to 120 mm against a spring that returns it to its stop under the nozzle.
SHELF = Slider(
    SHELF_NAME,
    rest=SHELF_REST,
    boxes=[(c, s, colour) for (c, s), colour in SHELF_BOXES],
    travel=SHELF_TRAVEL_M,
    rate=SHELF_SPRING_N_PER_M,
    damping=SHELF_DAMPING,
    mass=SHELF_WEIGHT_KG,
    axis="X",
)

# Slide 2: the brew button, pushed toward +x (into the machine) 8 mm against its spring.
BUTTON = Slider(
    BUTTON_NAME,
    rest=BUTTON_REST,
    boxes=[(BUTTON_BOX[0], BUTTON_BOX[1], COLOR_BUTTON)],
    travel=BUTTON_TRAVEL_M,
    rate=BUTTON_SPRING_N_PER_M,
    damping=BUTTON_DAMPING,
    mass=BUTTON_WEIGHT_KG,
    axis="+X",
)

# One loose 25 kg body (it stays put: friction 1.2) that carries both slides. Its origin is on the bench under the nozzle.
MACHINE = part(
    MACHINE_NAME,
    (NOZZLE_XY[0], NOZZLE_XY[1], TOP),
    boxes=[(c, s, colour) for (c, s), colour in MACHINE_BOXES],
    cylinders=[(NOZZLE[0], NOZZLE[1], NOZZLE[2], COLOR_CHROME, "Z")],
    sliders=[SHELF, BUTTON],
    mass=MACHINE_MASS_KG,
    friction=MACHINE_FRICTION,
)

SAUCER_GOOD = Cylinder(
    name=SAUCER_GOOD_NAME,
    pos=(SAUCER_GOOD_XY[0], SAUCER_GOOD_XY[1], TOP + SAUCER_HEIGHT_M / 2),
    color=COLOR_SAUCER,
    mass=None,
    radius=SAUCER_RADIUS_M,
    height=SAUCER_HEIGHT_M,
)
SAUCER_CRACKED = Cylinder(
    name=SAUCER_CRACKED_NAME,
    pos=(SAUCER_CRACKED_XY[0], SAUCER_CRACKED_XY[1], TOP + SAUCER_HEIGHT_M / 2),
    color=COLOR_SAUCER,
    mass=None,
    radius=SAUCER_RADIUS_M,
    height=SAUCER_HEIGHT_M,
)


def _mug(name, start, cracked, yaw=0.0):
    boxes = [(c, s, COLOR_MUG) for c, s in HANDLE_BOXES]
    if cracked:
        boxes += [(CRACK_STRIP[0], CRACK_STRIP[1], COLOR_CRACK), (CRACK_RIM[0], CRACK_RIM[1], COLOR_CRACK)]
    cylinders = [(MUG_FLOOR[0], MUG_FLOOR[1], MUG_FLOOR[2], COLOR_MUG, "Z")]
    cylinders += [(c, r, h, COLOR_MUG, "Z") for c, r, h in MUG_RODS]
    return part(
        name,
        (start[0], start[1], TOP + start[2]),
        boxes=boxes,
        cylinders=cylinders,
        mass=MUG_WEIGHT_KG,
        friction=MUG_FRICTION,
        yaw=yaw,
    )


MUG_GOOD = _mug(MUG_GOOD_NAME, MUG_GOOD_START, False)
# the cracked mug stands turned 180 degrees: handle toward +x (away from the robot and from the intact mug), crack facing the robot
MUG_CRACKED = _mug(MUG_CRACKED_NAME, MUG_CRACKED_START, True, yaw=math.pi)

OBJECTS = [MACHINE, SAUCER_GOOD, SAUCER_CRACKED, MUG_GOOD, MUG_CRACKED]

success = scoring.success
score = scoring.score
distance = scoring.distance
demo = demo_plan.plans
