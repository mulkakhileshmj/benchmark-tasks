"""Measure the released bar's physical load on the selected support block."""
import numpy as np
_base=None;_robot=None;_support=None;_touching=False;_loaded=False;_error=None;_details={}
def reset():
 global _base,_robot,_support,_touching,_loaded,_error
 _base=None;_robot=None;_support=None;_touching=False;_loaded=False;_error=None
def array(value):
 if hasattr(value,'detach'):return value.detach().cpu().numpy()
 return np.asarray(value.numpy() if hasattr(value,'numpy') else value)
def poll(manager):
 global _touching,_loaded,_error,_details
 if _robot is None:return
 try:
  _,_,_,sep,counts,starts=_robot.get_contact_data(dt=manager.get_physics_dt())
  sep=array(sep).reshape(-1);counts=array(counts).reshape(-1);starts=array(starts).reshape(-1)
  _touching=any(count and np.min(sep[int(start):int(start+count)])<=.001 for count,start in zip(counts,starts))
  force,_,normal,sep,counts,starts=_support.get_contact_data(dt=manager.get_physics_dt())
  force=array(force).reshape(-1);normal=array(normal).reshape(-1,3);sep=array(sep).reshape(-1);counts=array(counts).reshape(-1);starts=array(starts).reshape(-1)
  _loaded=any(any(force[i]>.02 and abs(normal[i,2])>.5 and sep[i]<=.001 for i in range(int(start),int(start+count))) for count,start in zip(counts,starts) if count)
  matrix=array(_support.get_contact_force_matrix(dt=manager.get_physics_dt())).reshape(-1,3)
  vertical=float(np.max(np.abs(matrix[:,2]))) if len(matrix) else 0.
  _details=dict(contact_count=int(counts.sum()),max_force=float(force.max()) if len(force) else 0.,vertical_force=vertical)
  _loaded=_loaded or vertical>.02
 except Exception as error:_error=repr(error)
def bind(base):
 global _base,_robot,_support
 if _base is base and _robot is not None:return
 import carb
 from isaaclab.sim.utils.stage import get_current_stage
 from isaaclab_physx.physics.physx_manager import PhysxManager
 from pxr import UsdPhysics,PhysxSchema
 carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
 sensors=[];bar=[];block=[]
 for prim in get_current_stage().Traverse():
  path=prim.GetPath().pathString
  if '/env_0/' not in path:continue
  if any(name in path for name in ('/LeftArm/','/RightArm/')) and 'PhysxContactReportAPI' in prim.GetAppliedSchemas():sensors.append(path)
  elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
   parts=path.split('/')
   if 'sine_fixture' in parts and parts[-1]=='bar':bar.append(path)
   if 'support_block_65' in parts:block.append(path);PhysxSchema.PhysxContactReportAPI.Apply(prim)
 if not sensors or len(bar)!=1 or len(block)!=1:raise RuntimeError('Sine bar contact bodies do not match the authored fixture')
 physics=PhysxManager.get_physics_sim_view();targets=bar+block
 _robot=physics.create_rigid_contact_view(sensors,filter_patterns=[targets]*len(sensors),max_contact_data_count=128*len(sensors)*2)
 _support=physics.create_rigid_contact_view(block,filter_patterns=[bar],max_contact_data_count=256)
 _base=base
 if not getattr(PhysxManager.step,'_sine_contact_hook',False):
  original=PhysxManager.step.__func__
  def step(cls):original(cls);poll(cls)
  step._sine_contact_hook=True;PhysxManager.step=classmethod(step)
def clear():
 if _error:raise RuntimeError('Support contact monitor failed: '+_error)
 return not _touching
def supported():
 if _error:raise RuntimeError('Support contact monitor failed: '+_error)
 return _loaded
