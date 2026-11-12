"""Independent native clearance checks. No force thresholds."""
import numpy as np

_base=None;_support=None;_robot=None;_spine=None;_contacts=np.zeros(4,dtype=bool)
_gaps=np.full(4,np.inf);_clear=False;_spine_clear=False;_error=None

def reset():
    global _base,_support,_robot,_spine,_contacts,_gaps,_clear,_spine_clear,_error
    _base=None;_support=None;_robot=None;_contacts=np.zeros(4,dtype=bool)
    _spine=None;_gaps=np.full(4,np.inf);_clear=False;_spine_clear=False;_error=None

def array(x):
    if hasattr(x,'detach'):return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x,'numpy') else x)

def grouped(separations,counts,starts,threshold):
    sep=np.asarray(separations,dtype=float).reshape(-1)
    counts=np.asarray(counts).reshape(-1);starts=np.asarray(starts).reshape(-1)
    if len(counts)!=len(starts):raise ValueError('Contact groups differ')
    gaps=np.full(len(counts),np.inf)
    for i,(count,start) in enumerate(zip(counts,starts)):
        count=int(count);start=int(start)
        if min(count,start)<0 or start+count>len(sep):raise ValueError('Invalid contact group')
        values=sep[start:start+count]
        if len(values):
            if not np.isfinite(values).all():raise ValueError('Nonfinite contact separation')
            gaps[i]=values.min()
    return gaps<=threshold,gaps

def poll(manager):
    global _contacts,_gaps,_clear,_spine_clear,_error
    if _support is None:return
    try:
        _,_,_,sep,counts,starts=_support.get_contact_data(dt=manager.get_physics_dt())
        _contacts,_gaps=grouped(array(sep),array(counts),array(starts),.0002)
        if len(_contacts)!=4:raise RuntimeError('Expected four independent bearing patches')
        _,_,_,sep,counts,starts=_robot.get_contact_data(dt=manager.get_physics_dt())
        touching,_=grouped(array(sep),array(counts),array(starts),.001)
        _clear=not bool(touching.any())
        _,_,_,sep,counts,starts=_spine.get_contact_data(dt=manager.get_physics_dt())
        touching,_=grouped(array(sep),array(counts),array(starts),.0002)
        _spine_clear=not bool(touching.any())
    except Exception as error:_error=repr(error)

def bind(base):
    global _base,_support,_robot,_spine
    if _base is base and _support is not None:return
    import carb
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics.physx_manager import PhysxManager
    from pxr import UsdPhysics
    carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
    found={n:[] for n in ['book','target_0_0','target_0_1','target_1_0','target_1_1']}
    robots=[]
    for prim in get_current_stage().Traverse():
        path=prim.GetPath().pathString
        if '/env_0/' not in path:continue
        if any(n in path for n in ['/LeftArm/','/RightArm/']) and 'PhysxContactReportAPI' in prim.GetAppliedSchemas():robots.append(path)
        elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
            for name in found:
                if name in path.split('/'):found[name].append(path)
    leaves=[]
    for name in ('leaf_pos','leaf_neg'):
        paths=[p for p in found['book'] if p.split('/')[-1]==name]
        if len(paths)!=1:raise RuntimeError('Book leaf body missing or ambiguous: '+name)
        leaves.append(paths[0])
    targets=[]
    for side in (0,1):
        row=[]
        for k in (0,1):
            paths=found[f'target_{side}_{k}']
            if len(paths)!=1:raise RuntimeError('Cradle support body missing or ambiguous')
            row.append(paths[0])
        targets.append(row)
    if not robots:raise RuntimeError('Native robot contact sensors unavailable')
    physics=PhysxManager.get_physics_sim_view()
    _support=physics.create_rigid_contact_view(leaves,filter_patterns=targets,max_contact_data_count=2048)
    _robot=physics.create_rigid_contact_view(robots,filter_patterns=[found['book']]*len(robots),max_contact_data_count=128*len(robots))
    roots=[p for p in found['book'] if p.split('/')[-1]=='root']
    if len(roots)!=1:raise RuntimeError('Spine rigid body missing or ambiguous')
    _spine=physics.create_rigid_contact_view(roots,filter_patterns=[[p for row in targets for p in row]],max_contact_data_count=512)
    _base=base
    if not getattr(PhysxManager.step,'_book_support_hook',False):
        original=PhysxManager.step.__func__
        def step(cls):original(cls);poll(cls)
        step._book_support_hook=True;PhysxManager.step=classmethod(step)

def measured():
    if _error:raise RuntimeError('Book contact monitor failed: '+_error)
    return _contacts.copy(),_gaps.copy(),_clear,_spine_clear
