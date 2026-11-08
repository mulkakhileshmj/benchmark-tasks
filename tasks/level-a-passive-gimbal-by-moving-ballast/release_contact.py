"""Reject direct output manipulation and verify release through native contacts."""
import numpy as np
_base=None;_views=[];_touching=[False,False];_error=None
def reset():
 global _base,_views,_touching,_error
 _base=None;_views=[];_touching=[False,False];_error=None
def array(value):
 if hasattr(value,'detach'):return value.detach().cpu().numpy()
 return np.asarray(value.numpy() if hasattr(value,'numpy') else value)
def poll(manager):
 global _error
 try:
  for index,view in enumerate(_views):
   _,_,_,separation,counts,starts=view.get_contact_data(dt=manager.get_physics_dt())
   separation=array(separation).reshape(-1);counts=array(counts).reshape(-1);starts=array(starts).reshape(-1)
   _touching[index]=any(count and np.min(separation[int(start):int(start+count)])<=.001 for count,start in zip(counts,starts))
 except Exception as error:_error=repr(error)
def bind(base):
 global _base,_views
 if _base is base and _views:return
 import carb
 from isaaclab.sim.utils.stage import get_current_stage
 from isaaclab_physx.physics.physx_manager import PhysxManager
 from pxr import UsdPhysics
 carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
 sensors=[];outputs=[];inputs=[]
 for prim in get_current_stage().Traverse():
  path=prim.GetPath().pathString
  if '/env_0/' not in path:continue
  if any(name in path for name in ('/LeftArm/','/RightArm/')) and 'PhysxContactReportAPI' in prim.GetAppliedSchemas():sensors.append(path)
  elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
   parts=path.split('/')
   if 'gimbal_fixture' in parts and parts[-1] in ('roll_frame','platform','x_weight','y_weight'):outputs.append(path)
   if 'gimbal_fixture' in parts and parts[-1] in ('root','roll_brake','pitch_brake'):inputs.append(path)
 if not sensors or len(outputs)!=4 or len(inputs)!=3:raise RuntimeError('Gimbal contact bodies do not match the authored mechanism')
 physics=PhysxManager.get_physics_sim_view()
 _views=[physics.create_rigid_contact_view(sensors,filter_patterns=[targets]*len(sensors),max_contact_data_count=128*len(sensors)*len(targets)) for targets in (outputs,outputs+inputs)]
 _base=base
 if not getattr(PhysxManager.step,'_differential_contact_hook',False):
  original=PhysxManager.step.__func__
  def step(cls):original(cls);poll(cls)
  step._differential_contact_hook=True;PhysxManager.step=classmethod(step)
def output_clear():
 if _error:raise RuntimeError('Contact monitor failed: '+_error)
 return not _touching[0]
def all_clear():
 if _error:raise RuntimeError('Contact monitor failed: '+_error)
 return not _touching[1]
