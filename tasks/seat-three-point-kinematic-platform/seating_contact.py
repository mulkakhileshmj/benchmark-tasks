"""Contact and release checks. No force sensor or force threshold is used."""
import numpy as np

CONTACT_GAP_M = 0.0002
_base = None
_robot = None
_support = None
_touching = True
_contacts = np.zeros(3, dtype=bool)
_gaps = np.full(3, np.inf)
_error = None

def reset():
    global _base, _robot, _support, _touching, _contacts, _gaps, _error
    _base = None; _robot = None; _support = None; _touching = True
    _contacts = np.zeros(3, dtype=bool); _gaps = np.full(3, np.inf); _error = None

def array(value):
    if hasattr(value, 'detach'): return value.detach().cpu().numpy()
    return np.asarray(value.numpy() if hasattr(value, 'numpy') else value)

def grouped_contacts(separations, counts, starts, gap):
    """Each filter group must independently contain a finite close contact."""
    separations = np.asarray(separations, dtype=float).reshape(-1)
    counts = np.asarray(counts).reshape(-1)
    starts = np.asarray(starts).reshape(-1)
    if len(counts) != len(starts): raise ValueError('Contact group lengths differ')
    gaps = np.full(len(counts), np.inf)
    for i, (count, start) in enumerate(zip(counts, starts)):
        count = int(count); start = int(start)
        if count < 0 or start < 0 or start + count > len(separations):
            raise ValueError('Invalid native contact group bounds')
        values = separations[start:start + count]
        if len(values) and np.isfinite(values).all(): gaps[i] = values.min()
    return gaps <= gap, gaps

def poll(manager):
    global _touching, _contacts, _gaps, _error
    if _robot is None: return
    try:
        _, _, _, sep, counts, starts = _robot.get_contact_data(dt=manager.get_physics_dt())
        touching, _ = grouped_contacts(array(sep), array(counts), array(starts), .001)
        _touching = bool(touching.any())
        _, _, _, sep, counts, starts = _support.get_contact_data(dt=manager.get_physics_dt())
        _contacts, _gaps = grouped_contacts(array(sep), array(counts), array(starts), CONTACT_GAP_M)
        if len(_contacts) != 3: raise RuntimeError('Expected three independent datum filters')
    except Exception as error: _error = repr(error)

def bind(base):
    global _base, _robot, _support
    if _base is base and _robot is not None: return
    import carb
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics.physx_manager import PhysxManager
    from pxr import UsdPhysics
    carb.settings.get_settings().set_bool('/physics/disableContactProcessing', False)
    sensors = []; objects = {name: [] for name in ('mount_plate','mount_cone','mount_groove','mount_flat')}
    for prim in get_current_stage().Traverse():
        path = prim.GetPath().pathString
        if '/env_0/' not in path: continue
        if any(name in path for name in ('/LeftArm/','/RightArm/')) and 'PhysxContactReportAPI' in prim.GetAppliedSchemas():
            sensors.append(path)
        elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
            for name in objects:
                if name in path.split('/'): objects[name].append(path)
    if not sensors or any(len(paths) != 1 for paths in objects.values()):
        raise RuntimeError('Mount contact bodies do not match authored datums')
    physics = PhysxManager.get_physics_sim_view()
    plate = objects['mount_plate']
    datums = [objects[name][0] for name in ('mount_cone','mount_groove','mount_flat')]
    _robot = physics.create_rigid_contact_view(sensors, filter_patterns=[plate]*len(sensors), max_contact_data_count=128*len(sensors))
    _support = physics.create_rigid_contact_view(plate, filter_patterns=[datums], max_contact_data_count=768)
    _base = base
    if not getattr(PhysxManager.step, '_mount_contact_hook', False):
        original = PhysxManager.step.__func__
        def step(cls): original(cls); poll(cls)
        step._mount_contact_hook = True
        PhysxManager.step = classmethod(step)

def clear():
    if _error: raise RuntimeError('Mount contact monitor failed: ' + _error)
    return not _touching

def contacts():
    if _error: raise RuntimeError('Mount contact monitor failed: ' + _error)
    return _contacts.copy()

def gaps():
    if _error: raise RuntimeError('Mount contact monitor failed: ' + _error)
    return _gaps.copy()
