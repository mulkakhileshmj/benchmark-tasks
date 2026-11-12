"""Original nominal bound-volume support geometry. SI units throughout."""
import math
import numpy as np

LENGTH=.240
LEAF_WIDTH=.180
LEAF_THICKNESS=.014
HINGE_OFFSET=.007
SOURCE_X=.100
TARGET_X=.360
SPINE_HEIGHT=.030
START_ANGLES=(60.,120.)
GOAL_ANGLES=(35.,145.)
OPEN_LIMIT_DEG=115.
CONTACT_GAP_M=.0002

def rx(degrees):
    a=math.radians(degrees);c,s=math.cos(a),math.sin(a)
    return np.array([[1.,0.,0.],[0.,c,-s],[0.,s,c]])

def hinge(side):
    return np.array([0.,HINGE_OFFSET if side==0 else -HINGE_OFFSET,0.])

def leaf_point(side,angle,s=.170,z=0.):
    return hinge(side)+rx(angle)@np.array([0.,s,z])

def underside(side,angle,s):
    return leaf_point(side,angle,s,(-1 if side==0 else 1)*LEAF_THICKNESS/2)

def support_piece(side,angle,xcentre,width):
    """A convex trapezoid with a genuinely inclined bearing surface."""
    a=underside(side,angle,0.);b=underside(side,angle,LEAF_WIDTH)
    a[2]+=SPINE_HEIGHT;b[2]+=SPINE_HEIGHT
    # Each strip is a convex solid. The binding channel remains empty.
    points=[]
    for x in (xcentre-width/2,xcentre+width/2):
        points.extend([[x,a[1],0.],[x,b[1],0.],[x,b[1],b[2]],[x,a[1],a[2]]])
    points=np.asarray(points);lo=points.min(0);hi=points.max(0);centre=(lo+hi)/2
    return dict(shape='mesh',pos=centre.tolist(),size=(hi-lo).tolist(),
                vertices=(points-centre).tolist(),
                faces=[[3,2,1,0],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]])

def goal_leaf_pose(side,table_top):
    return np.array([TARGET_X,0.,table_top+SPINE_HEIGHT])+hinge(side),rx(GOAL_ANGLES[side])

def opening(angles):
    return float(angles[1]-angles[0])

def relative_opening(leaf_a_rotation,leaf_b_rotation):
    relative=np.asarray(leaf_a_rotation).T@np.asarray(leaf_b_rotation)
    return math.degrees(math.acos(np.clip(relative[1,1],-1.,1.)))
