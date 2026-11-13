"""Native-state scripted YAM demo. Only joint targets are commanded.

The demo follows the observed mug pose when acquiring its handle. It keeps the
mechanically supported grasp attitude during transport and uses bounded position
feedback. Dream receives the ordinary camera/state observations, never this servo.
"""
import os,json
import numpy as np
from inspect_robots.policy import PolicyBase,PolicyInfo
from inspect_robots.types import Action,ActionChunk
from inspect_robots_yam.config import action_box
from inspect_robots_yam.packing import pack
from yam_arena import ik,station
from yam_arena.constants import READY,FINGER_OPEN,TCP_OFFSET
from yam_arena.demo import ArmPlan,grip_rotation
from yam_arena.urdf import axis_angle_matrix
import parts,demo_plan,strong_runtime as scoring
from engine import rotation
from geometry import MUG_GOOD_NAME,NOZZLE_XY,SHELF_TRAVEL_M,PLATE_TOP_Z,TABLE_TOP

FRONT=axis_angle_matrix(np.array([0.,-1.,0.]),np.radians(75.))@grip_rotation(np.pi/2)

def arr(v):
    if hasattr(v,'torch'):return arr(v.torch)
    if hasattr(v,'detach'):return v.detach().cpu().numpy()
    if hasattr(v,'numpy'):return v.numpy()
    return np.asarray(v)

class Servo(PolicyBase):
    def __init__(self):
        low,high=np.r_[ik.LOWER,0.],np.r_[ik.UPPER,1.]
        self.info=PolicyInfo(name='coffee_servo',action_space=action_box(pack(low,low),pack(high,high)))
    def bind(self,info):self.dt=1/float(info.control_hz or 20)
    def reset(self,scene):
        plans=demo_plan.plans()
        self.left=ArmPlan(plans['left'],station.BASES['left'])
        self.right=ArmPlan(plans['right'],station.BASES['right'])
        self.q=np.array(READY,dtype=float);self.active=False;self.acquiring=False;self.holding=False
        self.phase_started=0.;self.reference=np.array([.26,-.19,TABLE_TOP+.01])
        self.position_bias=np.zeros(3);self.tcp_bias=np.zeros(3)
        self.hold_R=None;self.release_R=None;self.release_offset=np.zeros(3);self.returning_to_saucer=False
    def act(self,observation):
        t=float(observation.state_time)+self.dt;left,_=self.left.action7(t)
        p,R,f,label=self.right.target(t);p=np.asarray(p);nominal=p.copy()
        entry=('in front of mug handle' in label)
        if entry and not self.active:
            self.active=True;self.acquiring=True;self.phase_started=t
            self.reference=np.array([NOZZLE_XY[0]-SHELF_TRAVEL_M,0.,TABLE_TOP+PLATE_TOP_Z]) if 'in ring' in label else np.array([.26,-.19,TABLE_TOP+.01])
            self.position_bias[:]=0.;self.tcp_bias[:]=0.;self.release_R=None
        if 'back to ready' in label:
            self.returning_to_saucer=False
            self.active=False;self.acquiring=False;self.holding=False;self.tcp_bias[:]=0.
        if self.active:R=ik.slerp_matrix(R,FRONT,min(1.,max(0.,t-self.phase_started)/2.5))
        env=parts.env();cup=None;tcp=None;actual_R=None
        if env is not None and self.active:
            cup=parts._read_part_state(env,parts._PARTS[MUG_GOOD_NAME])
            robot=env.unwrapped.scene['right_robot'];poses=arr(robot.data.body_link_pose_w)[0]
            pose=poses[list(robot.body_names).index('gripper')]
            actual_R=np.asarray(rotation(pose[3:]));tcp=pose[:3]+actual_R@np.asarray(TCP_OFFSET)
            if self.acquiring:
                Rc=cup.rotation();blend=min(1.,max(0.,t-self.phase_started)/1.5)
                target=cup.pos+Rc@(nominal-self.reference)
                p=nominal+blend*(target-nominal)
                R=ik.slerp_matrix(R,Rc@R,blend)
        normal={} if scoring.TENSOR_CONTACTS is None else scoring.TENSOR_CONTACTS.diagnostics.get(MUG_GOOD_NAME,{}).get('per_hand_body_normal_n',{})
        contacts=sum(1 for k,v in normal.items() if '/RightArm/' in k and k.endswith(('/tip_left','/tip_right')) and v>.2)
        closing=('pinch the handle' in label or 'preload handle grip' in label)
        if closing and abs(f)<.001 and contacts==2 and not self.holding:
            self.holding=True;self.acquiring=False;self.hold_R=R.copy();self.position_bias[:]=0.
        if f<-.005 and self.holding:self.holding=False
        gripping=self.holding and abs(f)<.001
        if any(k in label for k in ('over its saucer','onto the saucer')):self.returning_to_saucer=True
        if gripping and cup is not None:
            R=self.hold_R
            desired_center=nominal-np.array([demo_plan.GRASP_OFFSET_X,0.,demo_plan.GRASP_HEIGHT_Z+.002])
            if self.returning_to_saucer:desired_center+=np.array([.005,-.003,0.])
            self.position_bias=np.clip(self.position_bias+.35*self.dt*(desired_center-cup.pos),-.025,.025)
            candidate=desired_center+(tcp-cup.pos)+self.position_bias
            correction=candidate-nominal;size=np.linalg.norm(correction)
            p=nominal+correction*min(1.,.060/max(size,1e-12))
            self.release_R=R.copy();self.release_offset=p-nominal
        elif self.release_R is not None and any(k in label for k in ('release','rise straight up','retreat')):
            R=self.release_R;p=nominal+self.release_offset
        elif self.acquiring and tcp is not None:
            error=p-tcp
            if any(k in label for k in ('settle','pinch','preload')):self.tcp_bias=np.clip(self.tcp_bias+.5*self.dt*error,-.035,.035)
            correction=.75*error+self.tcp_bias;size=np.linalg.norm(correction)
            p=p+correction*min(1.,.035/max(size,1e-12))
        self.q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),self.q,base_pos=station.BASES['right'],iters=80)
        right=np.r_[self.q,np.clip(f/FINGER_OPEN,0.,1.)]
        trace=os.getenv('COFFEE_CONTROL_TRACE')
        if trace:
            with open(trace,'a') as out:out.write(json.dumps(dict(t=t,label=label,acquiring=self.acquiring,grasp_servo=gripping,two_finger_contact=contacts==2,ik_error_m=pe,ik_error_rad=re))+'\n')
        stop=t>max(self.left.duration,self.right.duration)+1.
        if os.getenv('GRASP_TEST'):
            if scoring.ENGINE.faults:stop=True
            if os.getenv('GRASP_FILLED') and scoring.ENGINE.volume.get(MUG_GOOD_NAME,0.)<110.:stop=True
        return ActionChunk(actions=[Action(data=pack(left,right),meta={'request_stop':True} if stop else {})],control_hz=20)

def policy(**kwargs):return Servo()
