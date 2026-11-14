"""Native-contact abrasion surrogate for an inert ampoule scoring collar."""
import math
import numpy as np

BINS = 36
MIN_BIN_TRAVEL_M = 0.00025
MIN_TOTAL_TRAVEL_M = 0.015
UNSCORED_TORQUE_NM = 0.20
SCORED_TORQUE_NM = 0.020
_monitor = None

def array(value):
    if hasattr(value,'torch'):value=value.torch
    if hasattr(value,'detach'):return value.detach().cpu().numpy()
    return np.asarray(value.numpy() if hasattr(value,'numpy') else value)

class Collar:
    def __init__(self,base):
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics
        self.stage=get_current_stage()
        bodies=[p for p in self.stage.Traverse() if '/env_0/' in str(p.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        body=[p for p in bodies if 'ampoule_body' in str(p.GetPath()).split('/')]
        files=[p for p in bodies if p.GetName().startswith('file_')]
        if len(body)!=1 or len(files)!=4:raise RuntimeError('Scoring collar bodies missing')
        sim=PhysxManager.get_physics_sim_view()
        self.body=sim.create_rigid_body_view(str(body[0].GetPath()))
        self.files=[sim.create_rigid_body_view(str(p.GetPath())) for p in files]
        self.views=[sim.create_rigid_contact_view(str(p.GetPath()),filter_patterns=[str(body[0].GetPath())],max_contact_data_count=64) for p in files]
        joints=[p for p in self.stage.Traverse() if p.GetName()=='neck_breakaway' and '/env_0/' in str(p.GetPath())]
        if len(joints)!=1:raise RuntimeError('Expected one native breakaway neck')
        self.joint=UsdPhysics.FixedJoint(joints[0])
        self.progress=np.zeros(BINS)
        self.last=[None]*4
        self.scored=False
        self.travel=0.
        self.contacts=0
        self.marked=set()
        self.witness={}
        for prim in self.stage.Traverse():
            path=str(prim.GetPath())
            name=prim.GetName()
            if '/ampoule_body/' in path and name.startswith('p') and name[1:].isdigit():
                index=int(name[1:])-1
                if 0<=index<BINS:self.witness[index]=prim

    def step(self,dt):
        from yam_arena.urdf import quat_xyzw_to_matrix
        pose=array(self.body.get_transforms())[0]
        rotation=quat_xyzw_to_matrix(pose[3:])
        neck=pose[:3]+rotation@np.array([0.,0.,.020])
        self.contacts=0
        for index,view in enumerate(self.views):
            force,points,normal,separation,counts,starts=view.get_contact_data(dt=dt)
            forces=array(force).reshape(-1)
            points=array(points).reshape(-1,3);separation=array(separation).reshape(-1)
            counts=array(counts).reshape(-1);starts=array(starts).reshape(-1)
            valid=[]
            for count,start in zip(counts,starts):
                for j in range(int(start),int(start+count)):
                    local=rotation.T@(points[j]-neck)
                    if forces[j]>1e-5 and separation[j]<=.0002 and abs(local[2]+.0002)<.001 and .0025<=np.linalg.norm(local[:2])<=.0038:
                        valid.append(local)
            if not valid:
                self.last[index]=None
                continue
            point=np.mean(valid,axis=0)
            file_local=rotation.T@(array(self.files[index].get_transforms())[0,:3]-neck)
            angle=math.atan2(file_local[1],file_local[0])
            previous=self.last[index]
            self.contacts+=1
            if previous is not None:
                delta=(angle-previous+math.pi)%(2*math.pi)-math.pi
                distance=.003*abs(delta)
                if 0.000001<distance<.0005:
                    # Integrate measured file motion across every traversed sector.
                    # Contact-centroid solver jitter is not abrasive travel.
                    count=max(1,math.ceil(distance/.000025))
                    for k in range(count):
                        theta=previous+delta*(k+.5)/count
                        slot=int((theta%(2*math.pi))/(2*math.pi)*BINS)%BINS
                        self.progress[slot]+=distance/count
                    self.travel+=distance
            self.last[index]=angle
        from pxr import UsdGeom,Gf
        for index in np.flatnonzero(self.progress>=MIN_BIN_TRAVEL_M):
            if int(index) not in self.marked and int(index) in self.witness:
                UsdGeom.Gprim(self.witness[int(index)]).GetDisplayColorAttr().Set([Gf.Vec3f(.08,.08,.08)])
                self.marked.add(int(index))
        qualified=int(np.count_nonzero(self.progress>=MIN_BIN_TRAVEL_M))
        if not self.scored and qualified>=34 and self.travel>=MIN_TOTAL_TRAVEL_M:
            # This changes an uncalibrated material parameter, never body pose or a score.
            self.joint.GetBreakTorqueAttr().Set(SCORED_TORQUE_NM)
            self.scored=True

    def report(self):
        return dict(scored=bool(self.scored),qualified_sectors=int(np.count_nonzero(self.progress>=MIN_BIN_TRAVEL_M)),
                    sector_travel_m=self.progress.tolist(),total_travel_m=float(self.travel),file_contacts=self.contacts,
                    native_break_torque_nm=float(self.joint.GetBreakTorqueAttr().Get()))

def reset():
    global _monitor
    _monitor=None

def bind(base):
    global _monitor
    if _monitor is not None:return
    _monitor=Collar(base)
    from isaaclab_physx.physics.physx_manager import PhysxManager
    old=PhysxManager.step.__func__
    def step(cls):
        old(cls)
        if _monitor is not None:_monitor.step(float(cls.get_physics_dt()))
    PhysxManager.step=classmethod(step)

def report():
    return _monitor.report() if _monitor is not None else dict(scored=False)
