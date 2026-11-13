"""Geometry for coffee_machine_cup_shelf (metres; x forward and y left from the arm midpoint; the table top stands for a bench).

Frames: a part's shapes are given in the part frame (z above the bench). The machine part's origin is on the bench under the
nozzle axis; a mug's origin is the centre of its base. Every number here is repeated in task.json (tests check it).
"""

from __future__ import annotations

import math

TABLE_TOP = 0.7475

COLOR_MACHINE = (0.125, 0.133, 0.15)
COLOR_CHROME = (0.78, 0.79, 0.81)
COLOR_TANK = (0.62, 0.78, 0.9)
COLOR_WINDOW = (0.03, 0.04, 0.05)
COLOR_BUTTON = (0.89, 0.627, 0.169)
COLOR_SAUCER = (0.906, 0.89, 0.855)
COLOR_MUG = (0.93, 0.91, 0.85)
COLOR_CRACK = (0.10, 0.07, 0.06)
COLOR_STREAM = (0.28, 0.16, 0.08)
COLOR_LIQUID = (0.30, 0.17, 0.08)
COLOR_STEAM = (0.93, 0.95, 0.97)
COLOR_LAMP_GREEN = (0.2, 0.9, 0.3)
COLOR_LAMP_AMBER = (1.0, 0.65, 0.1)

# -- machine: ONE loose part (25 kg stand-in for a 3 kg machine, friction 1.2) that carries two sprung slides ---------------
MACHINE_NAME = "cm_machine"
NOZZLE_XY = (0.43, 0.0)                 # the nozzle axis; also the machine part's origin (on the bench)
MACHINE_MASS_KG = 3.0
MACHINE_FRICTION = 1.2
MACHINE_BASE = ((0.01, 0.0, 0.02), (0.30, 0.17, 0.04))        # x 0.29..0.59, top at z 0.04
MACHINE_COLUMN = ((0.13, 0.0, 0.18), (0.06, 0.17, 0.28))      # x 0.53..0.59, z 0.04..0.32
MACHINE_HEAD = ((0.04, 0.0, 0.285), (0.24, 0.17, 0.07))       # x 0.35..0.59, underside z 0.25
MACHINE_TANK = ((0.18, 0.0, 0.14), (0.04, 0.12, 0.20))        # glass water tank behind (x 0.59..0.63)
MACHINE_WINDOW = ((-0.0815, -0.03, 0.285), (0.003, 0.06, 0.03))   # dark display window on the head's front face
MACHINE_BOXES = (
    (MACHINE_BASE, COLOR_MACHINE),
    (MACHINE_COLUMN, COLOR_MACHINE),
    (MACHINE_HEAD, COLOR_MACHINE),
    (MACHINE_TANK, COLOR_TANK),
    (MACHINE_WINDOW, COLOR_WINDOW),
)
NOZZLE = ((0.0, 0.0, 0.235), 0.012, 0.03)                     # chrome cylinder, z 0.22..0.25
NOZZLE_TIP_Z = 0.22
HEAD_FRONT_X = 0.35
BASE_TOP_Z = 0.04
LAMP_XYZ = (0.3475, 0.05, 0.285)                              # display lamp (marker) on the head's front face
LAMP_SIZE = (0.004, 0.02, 0.02)

# -- cup shelf: slide 1 of the machine, 120 mm toward the robot (-x), spring 8 N/m, damping 5 N s/m ---------------------------
SHELF_NAME = "shelf"
SHELF_TRAVEL_M = 0.12
SHELF_SPRING_N_PER_M = 8.0
SHELF_DAMPING = 5.0
SHELF_WEIGHT_KG = 0.25
SHELF_REST = (0.0, 0.0, 0.052)
PLATE = ((0.0, 0.0, 0.052), (0.18, 0.16, 0.008))               # x +-0.09, y +-0.08, z 0.048..0.056 (8 mm above the base)
PLATE_TOP_Z = 0.056
RING_INNER_HALF_M = 0.060                                      # square drip-tray ring, inner width 12.0 cm
RING_WALL_M = 0.006
RING_HEIGHT_M = 0.008
_RC = RING_INNER_HALF_M + RING_WALL_M / 2
RING_WALLS = (
    ((_RC, 0.0, PLATE_TOP_Z + RING_HEIGHT_M / 2), (RING_WALL_M, 2 * _RC + RING_WALL_M, RING_HEIGHT_M)),
    ((-_RC, 0.0, PLATE_TOP_Z + RING_HEIGHT_M / 2), (RING_WALL_M, 2 * _RC + RING_WALL_M, RING_HEIGHT_M)),
    ((0.0, _RC, PLATE_TOP_Z + RING_HEIGHT_M / 2), (2 * RING_INNER_HALF_M, RING_WALL_M, RING_HEIGHT_M)),
    ((0.0, -_RC, PLATE_TOP_Z + RING_HEIGHT_M / 2), (2 * RING_INNER_HALF_M, RING_WALL_M, RING_HEIGHT_M)),
)
RING_TOP_Z = PLATE_TOP_Z + RING_HEIGHT_M
GRIP_Y = 0.17                                                  # grip bar centre y
WING = ((-0.1275, 0.125, 0.052), (0.115, 0.11, 0.008))        # x -0.185..-0.07, y 0.07..0.18
GRIP_POSTS = (
    ((-0.170, GRIP_Y, 0.061), (0.016, 0.016, 0.010)),
    ((-0.120, GRIP_Y, 0.061), (0.016, 0.016, 0.010)),
)
GRIP_BAR = ((-0.145, GRIP_Y, 0.076), (0.08, 0.025, 0.02))      # 8 cm along x, 2.5 cm across y, z 0.066..0.086
GRIP_BAR_REST_X = NOZZLE_XY[0] + GRIP_BAR[0][0]                # 0.285
GRIP_BAR_Z = 0.076
SHELF_BOXES = (
    (PLATE, COLOR_CHROME),
    *((w, COLOR_CHROME) for w in RING_WALLS),
    (WING, COLOR_CHROME),
    *((p, COLOR_CHROME) for p in GRIP_POSTS),
    (GRIP_BAR, COLOR_BUTTON),
)

# -- brew button: slide 2 of the machine, pushed toward +x, 8 mm, 100 N/m --------------------------------------------------
BUTTON_NAME = "button"
BUTTON_TRAVEL_M = 0.008
BUTTON_SPRING_N_PER_M = 100.0
BUTTON_DAMPING = 5.0
BUTTON_WEIGHT_KG = 0.04
BUTTON_REST = (-0.146, -0.05, 0.023)
BUTTON_BOX = ((-0.172, -0.05, 0.023), (0.062, 0.04, 0.04))     # a 62 mm long plunger, face at x 0.227 (62 mm proud of the base front at 0.29), z 0.003..0.043 (3 mm off the bench)
BUTTON_FACE_X = NOZZLE_XY[0] + BUTTON_BOX[0][0] - BUTTON_BOX[1][0] / 2    # 0.278
BUTTON_XY = (NOZZLE_XY[0] + BUTTON_BOX[0][0], NOZZLE_XY[1] + BUTTON_BOX[0][1])

# -- mugs: hollow, 350 g, ceramic; origin = centre of the base --------------------------------------------------------------
MUG_GOOD_NAME = "cm_mug_good"
MUG_CRACKED_NAME = "cm_mug_cracked"
MUG_WEIGHT_KG = 0.35
MUG_FRICTION = 0.6
MUG_RADIUS_M = 0.0425                      # 8.5 cm across
MUG_HEIGHT_M = 0.095
MUG_FLOOR_M = 0.004
MUG_FLOOR = ((0.0, 0.0, MUG_FLOOR_M / 2), MUG_RADIUS_M, MUG_FLOOR_M)
MUG_ROD_COUNT = 96                         # overlapping vertical rods; conservative hollow interior
MUG_ROD_R = 0.003
MUG_ROD_CIRCLE_R = 0.0395
MUG_ROD_H = MUG_HEIGHT_M - MUG_FLOOR_M
MUG_INNER_R = MUG_ROD_CIRCLE_R - MUG_ROD_R  # 36.5 mm
MUG_RODS = tuple(
    (
        (
            MUG_ROD_CIRCLE_R * math.cos(2 * math.pi * i / MUG_ROD_COUNT),
            MUG_ROD_CIRCLE_R * math.sin(2 * math.pi * i / MUG_ROD_COUNT),
            MUG_FLOOR_M + MUG_ROD_H / 2,
        ),
        MUG_ROD_R,
        MUG_ROD_H,
    )
    for i in range(MUG_ROD_COUNT)
)
# Ordinary-scale curved C handle, in the XZ plane. Short overlapping
# ceramic boxes approximate its cross section; exact curves remain simplified.
HANDLE_BAR = ((-.077,0.,.055),(.008,.010,.009))
HANDLE_BOXES = tuple(((-.045-.032*math.cos(a),0.,.055+.026*math.sin(a)),(.008,.010,.009))
                     for a in [-math.pi/2+i*math.pi/24 for i in range(25)])
HANDLE_OFFSET_X = -.077
MUG_GRIP_Z = .052
MUG_GRIP_PRE_FINGER = -.034  # 40 mm total opening

# The crack: a dark strip up the wall (and a dash on the rim), in the mug frame at 55 degrees from the handle's opposite side; the cracked
# mug stands turned 180 degrees (handle toward +x, away from the robot and from the intact mug), so the crack faces the robot.
_CA = math.radians(55.0)
CRACK_STRIP = (
    (0.0435 * math.cos(_CA), 0.0435 * math.sin(_CA), 0.050),
    (0.003, 0.003, 0.060),
)
CRACK_RIM = (
    (0.0375 * math.cos(_CA), 0.0375 * math.sin(_CA), MUG_HEIGHT_M + 0.0005),
    (0.003, 0.012, 0.002),
)

# -- saucers: fixed, white ceramic, 13 cm across, 1 cm tall --------------------------------------------------------------------
SAUCER_RADIUS_M = 0.065
SAUCER_HEIGHT_M = 0.010
SAUCER_GOOD_NAME = "cm_saucer_good"
SAUCER_CRACKED_NAME = "cm_saucer_cracked"
SAUCER_GOOD_XY = (0.26, -0.19)
SAUCER_CRACKED_XY = (0.39, -0.19)
MUG_GOOD_START = (SAUCER_GOOD_XY[0], SAUCER_GOOD_XY[1], SAUCER_HEIGHT_M)       # base centre (x, y, z above the bench)
MUG_CRACKED_START = (SAUCER_CRACKED_XY[0], SAUCER_CRACKED_XY[1], SAUCER_HEIGHT_M)

# -- the brew (ledger rules; the same numbers are in task.json) -----------------------------------------------------------------
BREW_STEPS = 500                            # 25 s at 20 Hz
BREW_LEVEL_MAX = 0.90                       # level reached by a full brew (fraction of the mug height)
FILLED_MIN_LEVEL = 0.85
PARTIAL_MIN_LEVEL = 0.10
BUTTON_PRESS_M = 0.006
BUTTON_RELEASE_M = 0.002
SHELF_REST_TOL_M = 0.003
BREW_MUG_RADIUS_M = 0.025                   # mug axis within 25 mm of the nozzle axis (the ring guides it to <= 24.7 mm)
MUG_ON_PLATE_Z_M = 0.006                    # a mug counts as sitting in the ring when its base is within 6 mm of the plate top
MUG_ON_PLATE_TILT_DEG = 8.0
# Bypass rule: coffee only lands in a mug that was set on the plate while the shelf was out (the mug rides in on the shelf).
SHELF_LOAD_MIN_OUT_M = 0.06                 # shelf pulled at least 6 cm when the mug is set in the ring
LOAD_RING_RADIUS_M = 0.03                   # mug base centre within 30 mm of the ring centre (the ring leaves at most 24.7 mm)
LOAD_RESET_DIST_M = 0.08                    # a mug taken 8 cm from the ring centre, or lifted 2 cm off the plate, must be loaded again
LOAD_RESET_LIFT_M = 0.02
STEAM_STEPS = 400                           # 20 s
HOT_STEPS = 1200                            # 60 s
SPILL_STEPS_PER_TILE = 4
SPILL_FULL_MAX = 10
SPILL_MAX = 40

# Keep-out zone round the nozzle axis while the stream is shown.
KEEPOUT_RADIUS_M = 0.04
KEEPOUT_FINGER_MARGIN_M = 0.03              # a gripper centre this much outside the radius still has a finger in the zone
KEEPOUT_Z_MIN_M = PLATE_TOP_Z
KEEPOUT_Z_MAX_M = 0.25

# Scoring tolerances.
SAUCER_MAX_OFFCENTRE_M = 0.020
SAUCER_MAX_TILT_DEG = 3.0
SAUCER_MAX_Z_ERR_M = 0.003
MACHINE_MAX_SHIFT_M = 0.005
HOT_MARGIN_M = 0.005                        # a pad this close to a hot mug wall counts as touching it
PAD_HALF_X_M = 0.025
PAD_HALF_Y_M = 0.025
PAD_BELOW_TCP_M = 0.013
PAD_ABOVE_TCP_M = 0.030
RELEASE_MIN_DIST_M = 0.03
TCP_BELOW_LINK_M = 0.138
FINGERS_OPEN_FRACTION = 0.9
STILL_STEPS = 20                            # 1 s at 20 Hz
STILL_MOVE_M = 0.001
SUCCESS_HOLD_TIME_S = 1.0

# Display (markers only): 40 spill tiles on a 7 x 6 grid round the nozzle axis.
TILE_COUNT = SPILL_MAX
TILE_SIZE = (0.014, 0.014, 0.002)
TILE_GRID = tuple((((i - 3) * 0.016), ((j - 2.5) * 0.016)) for j in range(6) for i in range(7))[:TILE_COUNT]
STEAM_PUFFS = 5
STREAM_RADIUS_M = 0.004
LIQUID_RADIUS_M = .035
LIQUID_THICK_M = 0.003
