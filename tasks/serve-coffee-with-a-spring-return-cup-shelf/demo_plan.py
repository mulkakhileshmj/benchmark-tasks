"""Scripted demo for coffee_machine_cup_shelf. The left arm pinches the shelf's grip bar and pulls the shelf 12 cm out against its
spring (commanded 15 mm past the stop, so the stop sets the position); the right arm takes the intact mug by its handle from its
saucer and sets it in the cup ring; the right arm withdraws, the left lets go and the shelf springs back under the nozzle; the right
arm presses the brew button with closed fingers and keeps clear while the stream runs; then the left arm pulls the shelf out again,
the right arm takes the hot mug by the handle back to its saucer, and both arms let go and leave. Positions are commanded gripper
centres (open loop, times only; both grips are yaw 0, fingers closing along y). Every x is at most 0.30 m, every z at most 0.16 m."""

from __future__ import annotations

import os

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TABLE_HEIGHT
from yam_arena.demo import Waypoint

from geometry import (
    BUTTON_FACE_X,
    GRIP_BAR_REST_X,
    GRIP_BAR_Z,
    HANDLE_OFFSET_X,
    MUG_GRIP_PRE_FINGER,
    MUG_GRIP_Z,
    NOZZLE_XY,
    PLATE_TOP_Z,
    SHELF_TRAVEL_M,
)

TOP = TABLE_HEIGHT
SCALE = float(os.getenv("DEMO_TIME_SCALE", "1.0"))


def _s(t: float) -> float:
    return max(0.1, t * SCALE)


L_READY = (0.10, 0.14, TOP + 0.12)
R_READY = (0.10, -0.14, TOP + 0.12)

# Shelf grip bar: 8 x 2.5 x 2 cm along x at rest centre (0.285, 0.17), top z 0.086; the pads close across y. The shelf stops 120 mm out
# (bar centre x 0.165); the command goes 15 mm beyond so that the stop, not the arm, decides the position.
BAR_X, BAR_Y, BAR_Z = GRIP_BAR_REST_X, 0.17, GRIP_BAR_Z + 0.008
BAR_PULL_X = GRIP_BAR_REST_X - SHELF_TRAVEL_M - 0.015
BAR_HI_Z = 0.130

# Mug on its saucer at (0.26, -0.19): the handle bar is 10.5 cm toward the robot (x 0.155), at z 3.5 to 7.5 cm above the mug base (base 1.0 cm
# up). The gripper turns 90 degrees (fingers close along x): fingers part-open (70 mm) go down with one pad through the loop and one
# outside the bar, then close on the 1.6 cm bar. Grip height 5.2 cm above the base.
MUG_X, MUG_Y = 0.26, -0.19
GRASP_OFFSET_X = -.075
GRASP_HEIGHT_Z = .055
PICK_X = MUG_X + GRASP_OFFSET_X
PICK_Z = 0.010 + GRASP_HEIGHT_Z + 0.002
CARRY_Z = 0.170                      # mug carried with its base 8.7 cm up (23 mm over the ring wall)
APPROACH_Z = 0.156                   # open fingers over a mug standing in the ring: pad bottoms 17 mm over the handle top
YAW_MUG = 0.0
ENTRY_BACK = .070
ENTRY_UP = 0.018756443470178588
PRE = MUG_GRIP_PRE_FINGER
PINCH = FINGER_CLOSED                # a full close (about 50 N per pad); a 4 mm tighter squeeze did worse in the screen
# Mug in the ring while the shelf is out (ring centre x 0.31, plate top 5.6 cm).
RING_X = NOZZLE_XY[0] - SHELF_TRAVEL_M + GRASP_OFFSET_X
RING_Z = PLATE_TOP_Z + GRASP_HEIGHT_Z + 0.002
REST_RING_X = NOZZLE_XY[0] + GRASP_OFFSET_X    # handle bar x when the mug stands in the ring at rest (0.325, under the head)

# Brew plunger face at x 0.227 (z 0.3 to 4.3 cm, y -0.07 to -0.03): closed fingers; the front of a pad is 12.5 mm ahead of the centre, so
# contact starts at x 0.2145; the plunger bottoms out after 8 mm and the command goes 30 mm past the face (the hand lands about 30 mm short at the far edge).
BTN_Y = -0.05
BTN_Z = 0.040
BTN_NEAR_X = BUTTON_FACE_X - 0.078
BTN_PUSH_X = BUTTON_FACE_X + 0.030


def w(t, pos, finger, label, yaw=0.0):
    """A waypoint with z given above the table top."""
    return Waypoint(_s(t), (pos[0], pos[1], TOP + pos[2]), finger, label, yaw=yaw)


def wm(t, pos, finger, label):
    """A waypoint for the right arm's mug grip (yaw 90)."""
    return w(t, pos, finger, label, yaw=YAW_MUG)


def hold(wps, t):
    """Stay where the last waypoint ended for t seconds (t is already in plan seconds)."""
    last = wps[-1]
    return Waypoint(max(0.1, t), last.pos, last.finger, "hold", yaw=last.yaw)


def total(wps):
    return sum(x.duration for x in wps)


def sync(left, right):
    """Pad the shorter plan with a hold so that both arms reach the next phase together."""
    dl, dr = total(left), total(right)
    if dl > dr:
        right.append(hold(right, dl - dr))
    elif dr > dl:
        left.append(hold(left, dr - dl))


def make_settle_plan() -> dict[str, list[Waypoint]]:
    return {
        "left": [Waypoint(_s(8.0), L_READY, FINGER_OPEN, "left settle")],
        "right": [Waypoint(_s(8.0), R_READY, FINGER_OPEN, "right settle")],
    }


def _left_pull():
    return [
        w(2.5, (BAR_X, BAR_Y, BAR_HI_Z), FINGER_OPEN, "over the grip bar"),
        w(0.6, (BAR_X, BAR_Y, BAR_HI_Z), FINGER_OPEN, "settle above the bar"),
        w(1.5, (BAR_X, BAR_Y, BAR_Z), FINGER_OPEN, "lower the open fingers beside the bar"),
        w(0.6, (BAR_X, BAR_Y, BAR_Z), FINGER_OPEN, "settle"),
        w(0.8, (BAR_X, BAR_Y, BAR_Z), FINGER_CLOSED, "pinch the bar"),
        w(3.0, (BAR_PULL_X, BAR_Y, BAR_Z), FINGER_CLOSED, "pull the shelf out to its stop"),
        w(1.0, (BAR_PULL_X, BAR_Y, BAR_Z), FINGER_CLOSED, "hold it out"),
    ]


def _left_release():
    return [
        w(0.8, (BAR_PULL_X, BAR_Y, BAR_Z), FINGER_OPEN, "let go: the spring returns the shelf"),
        w(0.8, (BAR_PULL_X, BAR_Y, BAR_HI_Z), FINGER_OPEN, "rise clear of the bar"),
        Waypoint(_s(2.0), L_READY, FINGER_OPEN, "back to ready", yaw=0.0),
    ]


def _right_take_mug():
    return [
        wm(2.5, (PICK_X-ENTRY_BACK, MUG_Y, PICK_Z+ENTRY_UP), FINGER_OPEN, "in front of mug handle, fingers open"),
        wm(0.8, (PICK_X-ENTRY_BACK, MUG_Y, PICK_Z+ENTRY_UP), PRE, "part-close fingers in free space"),
        wm(2.0, (PICK_X, MUG_Y, PICK_Z), PRE, "enter handle along finger approach axis"),
        wm(1.8, (PICK_X, MUG_Y, PICK_Z), PRE, "settle"),
        wm(0.8, (PICK_X, MUG_Y, PICK_Z), PINCH, "pinch the handle bar"),
        wm(1.5, (PICK_X, MUG_Y, PICK_Z), PINCH, "preload handle grip before lifting"),
        wm(2.5, (PICK_X, MUG_Y, CARRY_Z), PINCH, "lift the mug clear of the ring wall"),
    ]


def _right_set_in_ring():
    return [
        wm(3.0, (RING_X, 0.0, CARRY_Z), PINCH, "carry the mug over the cup ring"),
        wm(0.6, (RING_X, 0.0, CARRY_Z), PINCH, "settle"),
        wm(1.5, (RING_X, 0.0, RING_Z), PINCH, "lower the mug into the ring"),
        wm(0.6, (RING_X, 0.0, RING_Z), PINCH, "settle"),
        wm(0.8, (RING_X-.010, 0.0, RING_Z+.008), PRE, "release the handle"),
        wm(1.0, (RING_X-ENTRY_BACK, 0.0, RING_Z+ENTRY_UP), PRE, "retreat along finger axis"),
        wm(0.6, (RING_X, 0.0, 0.158), FINGER_OPEN, "open the fingers"),
        w(1.5, (RING_X, -0.12, 0.158), FINGER_OPEN, "slide clear toward the right"),
        Waypoint(_s(2.0), R_READY, FINGER_OPEN, "back to ready", yaw=0.0),
    ]


def _right_press_button():
    return [
        w(2.0, (BTN_NEAR_X, BTN_Y, 0.12), FINGER_CLOSED, "closed fingers over the button, in front of it"),
        w(1.5, (BTN_NEAR_X, BTN_Y, BTN_Z), FINGER_CLOSED, "down to the button's height"),
        w(0.6, (BTN_NEAR_X, BTN_Y, BTN_Z), FINGER_CLOSED, "settle"),
        w(1.5, (BTN_PUSH_X, BTN_Y, BTN_Z), FINGER_CLOSED, "press the button through its 8 mm stroke"),
        w(1.0, (BTN_PUSH_X, BTN_Y, BTN_Z), FINGER_CLOSED, "hold it pressed: the brew starts"),
        w(0.8, (BTN_NEAR_X, BTN_Y, BTN_Z), FINGER_CLOSED, "back off the button"),
        w(1.0, (BTN_NEAR_X, BTN_Y, 0.12), FINGER_OPEN, "rise clear"),
        Waypoint(_s(2.0), R_READY, FINGER_OPEN, "back to ready", yaw=0.0),
    ]


def _right_return_mug():
    return [
        wm(2.5, (RING_X-ENTRY_BACK, 0.0, RING_Z+ENTRY_UP), FINGER_OPEN, "in front of mug handle in ring"),
        wm(0.8, (RING_X-ENTRY_BACK, 0.0, RING_Z+ENTRY_UP), PRE, "part-close fingers in front of hot handle"),
        wm(2.0, (RING_X, 0.0, RING_Z), PRE, "enter hot handle along finger axis"),
        wm(1.8, (RING_X, 0.0, RING_Z), PRE, "settle"),
        wm(0.8, (RING_X, 0.0, RING_Z), PINCH, "pinch the handle bar (never the hot wall)"),
        wm(1.5, (RING_X, 0.0, RING_Z), PINCH, "preload handle grip before lifting"),
        wm(2.5, (RING_X, 0.0, CARRY_Z), PINCH, "lift the mug out of the ring"),
        wm(3.0, (PICK_X, MUG_Y, CARRY_Z), PINCH, "carry it over its saucer"),
        wm(0.6, (PICK_X, MUG_Y, CARRY_Z), PINCH, "settle"),
        wm(1.5, (PICK_X, MUG_Y, PICK_Z), PINCH, "lower it onto the saucer"),
        wm(0.6, (PICK_X, MUG_Y, PICK_Z), PINCH, "settle"),
        wm(0.8, (PICK_X-.010, MUG_Y, PICK_Z+.008), PRE, "release the handle"),
        wm(1.0, (PICK_X-ENTRY_BACK, MUG_Y, PICK_Z+ENTRY_UP), PRE, "retreat along finger axis"),
        wm(0.6, (PICK_X, MUG_Y, CARRY_Z), FINGER_OPEN, "open the fingers"),
        Waypoint(_s(2.0), R_READY, FINGER_OPEN, "back to ready", yaw=0.0),
    ]


def make_demo_plan(ablation: str | None = None) -> dict[str, list[Waypoint]]:
    if ablation == "left":
        # left arm idle: the right arm runs its main-plan sequence unchanged (so every point stays within reach), but with no
        # arm holding the shelf out the mug cannot be set in the ring and no stream is ever started with a mug under it
        main = make_demo_plan(None)
        return {"left": [Waypoint(total(main["right"]), L_READY, FINGER_OPEN, "ablation: left arm idle")], "right": main["right"]}
    if ablation == "right":
        left = _left_pull()
        left.append(hold(left, _s(12.0)))
        left += _left_release() + [Waypoint(_s(8.0), L_READY, FINGER_OPEN, "wait", yaw=0.0)]
        return {"left": left, "right": [Waypoint(total(left), R_READY, FINGER_OPEN, "ablation: right arm idle")]}

    left, right = [], []
    # 1. the left arm pulls the shelf out and holds it; meanwhile the right arm takes the intact mug off its saucer
    left += _left_pull()
    right += _right_take_mug()
    sync(left, right)
    # 2. the right arm sets the mug in the ring and clears; the left holds the shelf out throughout
    right += _right_set_in_ring()
    left.append(hold(left, total(right) - total(left)))
    # 3. the left arm lets go (the shelf returns under the nozzle in 2 to 3 s); the right arm waits at ready until it is home
    left += _left_release()
    sync(left, right)
    # 4. the right arm presses the brew button, backs away and waits out the 8 s stream; the left stays at ready
    right += _right_press_button()
    right.append(Waypoint(_s(26.0), R_READY, FINGER_OPEN, "keep clear until the stream has stopped", yaw=0.0))
    sync(left, right)
    # 5. the left arm pulls the shelf out again; the right arm takes the hot mug by its handle back to its saucer
    left += _left_pull()
    sync(left, right)
    right += _right_return_mug()
    left.append(hold(left, total(right) - total(left)))
    # 6. the left arm lets go (the empty shelf returns), both leave, and the demo waits for stillness
    left += _left_release()
    left.append(Waypoint(_s(8.0), L_READY, FINGER_OPEN, "wait for stillness", yaw=0.0))
    right.append(hold(right, total(left) - total(right)))
    return {"left": left, "right": right}


def make_grasp_test():
    right=_right_take_mug()
    right.append(hold(right,6.0))
    right += [wm(2.0,(PICK_X,MUG_Y,PICK_Z),PINCH,'lower onto original saucer'),
              wm(1.0,(PICK_X,MUG_Y,PICK_Z),PRE,'release after hold test'),
              wm(2.0,(PICK_X,MUG_Y,CARRY_Z),PRE,'retreat')]
    return {'left':[Waypoint(total(right),L_READY,FINGER_OPEN,'stationary shelf grasp test')], 'right':right}


def plans():
    if os.getenv('GRASP_TEST'):
        return make_grasp_test()
    if os.getenv("SETTLE_ONLY"):
        return make_settle_plan()
    if os.getenv("ABLATION"):
        return make_demo_plan("right" if os.getenv("ABLATION_ARM", "left") == "right" else "left")
    return make_demo_plan(None)
