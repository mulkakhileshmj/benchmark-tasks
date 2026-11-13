"""Rigid contact geometry of a dry pop-up waste service rig. SI units."""
import math
import numpy as np

TOP = .7475
CENTER = (.295, .015, TOP + .095)
STOP = (0., 0., 0.)
BALL = (0., -.025, 0.)
PARK = .016
ARM = .065
TIP = .034
RADIUS = .002
OPEN_ANGLE = math.radians(29)


def box(pos, size, color=(.52, .55, .58)):
    return dict(pos=pos, size=size, color=color, contact_offset=.00005)


def cylinder(pos, radius, height, axis="Z", color=(.7, .7, .72)):
    size = [radius*2]*3
    size["XYZ".index(axis)] = height
    return dict(pos=pos, size=tuple(size), shape="cylinder", radius=radius,
                height=height, axis=axis, color=color, contact_offset=.00005)


def ring(z, inner, outer, thickness, count=24):
    parts = []
    for i in range(count):
        angle = 2*math.pi*i/count
        mid = (inner+outer)/2
        piece = box((mid*math.cos(angle), mid*math.sin(angle), z),
                    (outer-inner, 2*outer*math.tan(math.pi/count), thickness))
        piece['rot_z'] = angle
        parts.append(piece)
    return parts


def plug():
    # The opening is empty. Four peripheral solids form its boundaries.
    return [
        box((-.00375, 0., 0.), (.0025,.0025,.0115)),
        box((.00375, 0., 0.), (.0025,.0025,.0115)),
        box((0.,0.,.005),(.010,.0025,.002)),
        box((0.,0.,-.005),(.010,.0025,.002)),
        box((0.,0.,.016),(.005,.003,.020)),
        cylinder((0.,0.,.0275),.0175,.003),
        cylinder((0.,0.,.034),.007,.010),
    ]


def rod():
    return [cylinder((0.,(TIP-ARM)/2,0.), RADIUS, TIP+ARM, 'Y'),
            cylinder((0.,-ARM,0.), .005,.015,'Y',(.15,.4,.65))]


def point_on_rod(angle, axial_position, slide=0.):
    return np.array(CENTER)+BALL+[0.,slide+axial_position*math.cos(angle),
                                  axial_position*math.sin(angle)]


def gauge(angle, slide, stopper_lift, pivot_height=0.):
    """Margins at both eye faces prevent a short pin from passing the gauge."""
    if not all(map(math.isfinite,[angle,slide,stopper_lift])):
        return False
    cy=BALL[1]+slide
    distal_y=cy+TIP*math.cos(angle)
    if distal_y < .00125+.002 or cy >= -.00125:
        return False
    for y in (-.00125,.00125):
        dz=pivot_height+(y-cy)*math.tan(angle)-stopper_lift
        if abs(dz)+RADIUS/math.cos(angle) > .004+.00015:
            return False
    return True
