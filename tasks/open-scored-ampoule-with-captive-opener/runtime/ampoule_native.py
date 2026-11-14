"""Native breakaway neck and passive spring jaws. No runtime body-state writes.

The cap is an independent rigid body. A fixed joint is authored before physics
initialization and breaks through PhysX's own torque rule. It is never disabled
by the controller or scorer. One episode per process is mandatory.
"""
from dataclasses import dataclass
from pathlib import Path
import math
import os
from yam_articulated_base import Compound
import yam_articulated as articulated
import yam_articulated_base as common

BREAK_TORQUE_NM = .20
BREAK_FORCE_N = 20.

@dataclass(frozen=True)
class BreakawayCap(Compound):
    body_name: str = 'ampoule_body'
    body_anchor: tuple = (0.,0.,.020)
    cap_anchor: tuple = (0.,0.,0.)

_CAPS={}

def _spawn_cap(prim_path,cfg,translation=None,orientation=None,**kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf,Sdf,UsdGeom,UsdPhysics,PhysxSchema
    spec=_CAPS[cfg.cap_name]
    stage=get_current_stage()
    # Let the shared compound author create only original cap geometry.
    file=common._usd(spec,Path(os.environ.get('YAM_GEOMETRY_DIR','generated_geometry')))
    prim=stage.DefinePrim(prim_path,'Xform')
    prim.GetReferences().AddReference(str(file.resolve()))
    xform=UsdGeom.Xformable(prim)
    if translation is not None:xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    if orientation is not None:
        # Arena supplies xyzw while the USD quaternion constructor takes w first.
        x,y,z,w=(float(component) for component in orientation)
        xform.AddOrientOp().Set(Gf.Quatf(w,Gf.Vec3f(x,y,z)))
    # Resolve the body already authored in this environment rather than assuming
    # an Arena version-specific asset prefix. Fail loudly on order/ambiguity.
    env_prefix=prim_path.split('/env_')[0]+'/env_'+prim_path.split('/env_')[1].split('/')[0]+'/'
    bodies=[p for p in stage.Traverse() if p.GetPath().pathString.startswith(env_prefix)
            and spec.body_name in p.GetPath().pathString.split('/') and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(bodies)!=1:raise RuntimeError('Ampoule body must be authored before its cap: '+str([str(p.GetPath()) for p in bodies]))
    # The 1 g cap and millimetre neck need a converged fixed constraint while
    # its retaining jaws first make contact. These are native solver settings.
    for rigid in (bodies[0],prim):
        tuning=PhysxSchema.PhysxRigidBodyAPI.Apply(rigid)
        tuning.CreateSolverPositionIterationCountAttr(128)
        tuning.CreateSolverVelocityIterationCountAttr(16)
    transforms=UsdGeom.XformCache()
    body_frame=transforms.GetLocalToWorldTransform(bodies[0])
    cap_frame=transforms.GetLocalToWorldTransform(prim)
    anchor_error=(body_frame.Transform(Gf.Vec3d(*spec.body_anchor))-
                  cap_frame.Transform(Gf.Vec3d(*spec.cap_anchor))).GetLength()
    axis_error=max((body_frame.TransformDir(axis)-cap_frame.TransformDir(axis)).GetLength()
                   for axis in (Gf.Vec3d(1,0,0),Gf.Vec3d(0,1,0),Gf.Vec3d(0,0,1)))
    if anchor_error>1e-5 or axis_error>1e-4:
        raise RuntimeError(f'Breakaway neck starts misaligned: {anchor_error=} {axis_error=}, '
                           f'body={body_frame}, cap={cap_frame}, supplied orientation={orientation}')
    joint=UsdPhysics.FixedJoint.Define(stage,prim_path+'/neck_breakaway')
    joint.CreateBody0Rel().SetTargets([bodies[0].GetPath()])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(prim_path)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*spec.body_anchor))
    joint.CreateLocalPos1Attr(Gf.Vec3f(*spec.cap_anchor))
    joint.CreateBreakForceAttr(BREAK_FORCE_N)
    joint.CreateBreakTorqueAttr(BREAK_TORQUE_NM)
    joint.CreateExcludeFromArticulationAttr(True)
    joint.CreateCollisionEnabledAttr(True)
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    return prim

def install():
    from yam_arena import scene
    articulated._install()
    if getattr(scene.task_object,'_ampoule_cap',False):return
    previous=scene.task_object
    def spawn(spec):
        if not isinstance(spec,BreakawayCap):return previous(spec)
        from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
        from isaaclab.sim.utils import clone
        from isaaclab.utils import configclass
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.utils.pose import Pose
        @configclass
        class CapSpawnerCfg(SpawnerCfg):
            func: object=clone(_spawn_cap)
            cap_name: str=''
        _CAPS[spec.name]=spec
        # Register bounds in the same compound wrapper by its ordinary builder.
        previous(spec)
        return Object(name=spec.name,object_type=scene.ObjectType.RIGID,
                      spawner_cfg=CapSpawnerCfg(cap_name=spec.name),
                      initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),rotation_xyzw=(0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2))))
    spawn._ampoule_cap=True
    spawn._articulated=True
    spawn._industrial=True
    scene.task_object=spawn

def embodiment(**kwargs):
    base_factory=articulated.embodiment
    result=base_factory(**kwargs)
    previous=result._ensure_sim
    def ensure(goal):
        # Launch through the known wrapper first. The scene extension needs to
        # install before Arena creates task objects, so wrap _install temporarily.
        old=articulated._install
        def combined():
            old()
            articulated._install=old
            try:install()
            finally:articulated._install=combined
        articulated._install=combined
        try:return previous(goal)
        finally:articulated._install=old
    result._ensure_sim=ensure
    return result

