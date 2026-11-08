import numpy as np
_base=None;_robot=None;_support=None;_touching=False;_forces=np.zeros(3);_error=None
def reset():
 global _base,_robot,_support,_touching,_forces,_error
 _base=None;_robot=None;_support=None;_touching=False;_forces=np.zeros(3);_error=None
def array(value):
 if hasattr(value,'detach'):return value.detach().cpu().numpy()
 return np.asarray(value.numpy() if hasattr(value,'numpy') else value)
def poll(manager):
 global _touching,_forces,_error
 if _robot is None:return
 try:
  _,_,_,sep,counts,starts=_robot.get_contact_data(dt=manager.get_physics_dt())
  sep=array(sep).reshape(-1);counts=array(counts).reshape(-1);starts=array(starts).reshape(-1)
  _touching=any(count and np.min(sep[int(start):int(start+count)])<=.001 for count,start in zip(counts,starts))
  _forces=np.abs(array(_support.get_contact_force_matrix(dt=manager.get_physics_dt())).reshape(3,3)[:,2])
 except Exception as error:_error=repr(error)
def bind(base):
 global _base,_robot,_support
 if _base is base and _robot is not None:return
 import carb
 from isaaclab.sim.utils.stage import get_current_stage
 from isaaclab_physx.physics.physx_manager import PhysxManager
 from pxr import UsdPhysics
 carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
 sensors=[];objects={name:[] for name in ('mount_plate','mount_cone','mount_groove','mount_flat')}
 for prim in get_current_stage().Traverse():
  path=prim.GetPath().pathString
  if '/env_0/' not in path:continue
  if any(name in path for name in ('/LeftArm/','/RightArm/')) and 'PhysxContactReportAPI' in prim.GetAppliedSchemas():sensors.append(path)
  elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
   for name in objects:
    if name in path.split('/'):objects[name].append(path)
 if not sensors or any(len(paths)!=1 for paths in objects.values()):raise RuntimeError('Mount contact bodies do not match authored datums')
 physics=PhysxManager.get_physics_sim_view();plate=objects['mount_plate'];datums=[objects[name][0] for name in ('mount_cone','mount_groove','mount_flat')]
 _robot=physics.create_rigid_contact_view(sensors,filter_patterns=[plate]*len(sensors),max_contact_data_count=128*len(sensors))
 _support=physics.create_rigid_contact_view(plate,filter_patterns=[datums],max_contact_data_count=768)
 _base=base
 if not getattr(PhysxManager.step,'_mount_contact_hook',False):
  original=PhysxManager.step.__func__
  def step(cls):original(cls);poll(cls)
  step._mount_contact_hook=True;PhysxManager.step=classmethod(step)
def clear():
 if _error:raise RuntimeError('Mount contact monitor failed: '+_error)
 return not _touching
def forces():
 if _error:raise RuntimeError('Mount contact monitor failed: '+_error)
 return _forces.copy()
