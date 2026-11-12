"""Measure robot clearance from the moving handle and locking pin."""
import numpy as np
_base=None
_view=None
_touching=True
_error=None

def reset():
    global _base,_view,_touching,_error
    _base=None;_view=None;_touching=True;_error=None

def array(value):
    if hasattr(value,'detach'):
        return value.detach().cpu().numpy()
    return np.asarray(value.numpy() if hasattr(value,'numpy') else value)

def poll(manager):
    global _touching,_error
    if _view is None:
        return
    try:
        _,_,_,separation,counts,starts=_view.get_contact_data(dt=manager.get_physics_dt())
        separation=array(separation).reshape(-1)
        counts=array(counts).reshape(-1)
        starts=array(starts).reshape(-1)
        _touching=any(count and np.min(separation[int(start):int(start+count)])<=.001
                      for count,start in zip(counts,starts))
    except Exception as error:
        _error=repr(error)

def bind(base):
    global _base,_view
    if _base is base and _view is not None:
        return
    import carb
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics.physx_manager import PhysxManager
    from pxr import UsdPhysics
    carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
    sensors=[];targets=[]
    for prim in get_current_stage().Traverse():
        name=prim.GetPath().pathString
        if '/env_0/' not in name:
            continue
        if any(arm in name for arm in ('/LeftArm/','/RightArm/')) and 'PhysxContactReportAPI' in prim.GetAppliedSchemas():
            sensors.append(name)
        elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
            parts=name.split('/')
            if ('adjustable_handle' in parts and parts[-1]=='handle') or ('locking_pin' in parts and parts[-1]=='pin'):
                targets.append(name)
    if not sensors or len(targets)!=2:
        raise RuntimeError('Release monitor does not match the authored handle and pin')
    _view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
        sensors,filter_patterns=[targets]*len(sensors),max_contact_data_count=256*len(sensors))
    _base=base
    if not getattr(PhysxManager.step,'_handle_release_hook',False):
        original=PhysxManager.step.__func__
        def step(cls):
            original(cls)
            poll(cls)
        step._handle_release_hook=True
        PhysxManager.step=classmethod(step)

def clear():
    if _error:
        raise RuntimeError('Handle release monitor failed: '+_error)
    return not _touching
