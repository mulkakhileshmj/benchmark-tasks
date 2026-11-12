"""Physics-step contacts. Force values are never used by the task scorer."""
import numpy as np
_base=None;_robot=None;_body=None;_cap=None;_physics=None;_error=None
_rim=False;_cap_touch=False;_touching=True
_subscription=None;_broken=False;_escape=False;_tool=None;_cap_view=None;_vertices=None;_polled=False
_max_unbroken_gap=0.;_max_unbroken_angle=0.;_gap_exceed_total_s=0.;_gap_exceed_run_s=0.;_max_gap_exceed_run_s=0.
_first_escape=None
_rest=None;_rest_views=None;_rest_geometry=None;_rest_roots=None;_fingers=None

def reset():
    global _base,_robot,_body,_cap,_physics,_error,_rim,_cap_touch,_touching
    global _subscription,_broken,_escape,_tool,_cap_view,_vertices,_polled
    global _max_unbroken_gap,_max_unbroken_angle,_gap_exceed_total_s,_gap_exceed_run_s,_max_gap_exceed_run_s
    global _first_escape
    global _rest,_rest_views,_rest_geometry,_rest_roots,_fingers
    _base=_robot=_body=_cap=_physics=None;_error=None;_rim=False;_cap_touch=False;_touching=True
    _subscription=None;_broken=False;_escape=False;_tool=None;_cap_view=None;_vertices=None;_polled=False
    _max_unbroken_gap=0.;_max_unbroken_angle=0.;_gap_exceed_total_s=0.;_gap_exceed_run_s=0.;_max_gap_exceed_run_s=0.
    _first_escape=None
    _rest=_rest_views=_rest_geometry=_rest_roots=_fingers=None

def a(x):
    if hasattr(x,'detach'):return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x,'numpy') else x)

def bind(base,cap_vertices,motion_geometry,rest_roots):
    global _base,_robot,_body,_cap,_physics
    global _subscription,_tool,_cap_view,_vertices
    global _rest,_rest_views,_rest_geometry,_rest_roots,_fingers
    if _base is base:return
    import carb
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics.physx_manager import PhysxManager
    from pxr import UsdPhysics
    carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
    names={'ampoule_body':[],'ampoule_cap':[],'opener':[]};sensors=[]
    for p in get_current_stage().Traverse():
        path=p.GetPath().pathString
        if '/env_0/' not in path:continue
        if any(n in path for n in ('/LeftArm/','/RightArm/')) and 'PhysxContactReportAPI' in p.GetAppliedSchemas():sensors.append(path)
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            for name in names:
                if name in path.split('/'):names[name].append(path)
    if not sensors or len(names['ampoule_body'])!=1 or len(names['ampoule_cap'])!=1 or not names['opener']:
        raise RuntimeError('Ampoule contact filter bodies are ambiguous or missing')
    _physics=PhysxManager.get_physics_sim_view()
    _body=_physics.create_rigid_body_view(names['ampoule_body'][0])
    _cap_view=_physics.create_rigid_body_view(names['ampoule_cap'][0])
    roots=[p for p in names['opener'] if p.endswith('/root')]
    if len(roots)!=1:raise RuntimeError('Expected one native opener root')
    _tool=_physics.create_rigid_body_view(roots[0]);_vertices=np.array(cap_vertices,dtype=float)
    from observed_rest import ObservedRest
    from yam_arena.constants import FINGER_JOINTS
    _rest=ObservedRest();_rest_geometry=motion_geometry;_rest_roots=rest_roots
    _rest_views={}
    for key in motion_geometry:
        object_name,leaf=key.split(':')
        matches=[p for p in names[object_name] if not leaf or p.endswith('/'+leaf)]
        if len(matches)!=1:raise RuntimeError('Ambiguous rest body '+key)
        _rest_views[key]=_physics.create_rigid_body_view(matches[0])
    _fingers=[(base.scene[n].root_physx_view,base.scene[n].find_joints(FINGER_JOINTS,preserve_order=True)[0])
              for n in ('left_robot','right_robot')]
    import omni.physx
    from omni.physx.bindings._physx import SimulationEvent
    from pxr import PhysicsSchemaTools
    def native_event(event):
        global _broken,_error
        try:
            if event.type!=int(SimulationEvent.JOINT_BREAK):return
            values=event.payload['jointPath']
            path=str(PhysicsSchemaTools.decodeSdfPath(values[0],values[1]))
            if '/ampoule_cap/' in path and path.endswith('/neck_breakaway'):_broken=True
        except Exception as e:_error='Joint break monitor: '+repr(e)
    _subscription=omni.physx.get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(native_event)
    filters=names['ampoule_body']+names['ampoule_cap']+names['opener']
    _robot=_physics.create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=512*len(sensors))
    _base=base
    if not getattr(PhysxManager.step,'_ampoule_contacts',False):
        old=PhysxManager.step.__func__
        def step(cls):old(cls);poll(cls)
        step._ampoule_contacts=True;PhysxManager.step=classmethod(step)
    # Seed from the current native frame so the first observed task sample does
    # not confuse a newly attached monitor with a monitor failure.
    poll(PhysxManager)

def poll(manager):
    global _rim,_cap_touch,_touching,_error,_escape,_polled
    global _max_unbroken_gap,_max_unbroken_angle,_gap_exceed_total_s,_gap_exceed_run_s,_max_gap_exceed_run_s
    global _first_escape
    if _robot is None:return
    try:
        _,points,_,separation,counts,starts=_robot.get_contact_data(dt=manager.get_physics_dt())
        points=a(points);separation=a(separation).reshape(-1);counts=a(counts);starts=a(starts)
        pose=a(_body.get_transforms())[0]
        from yam_arena.urdf import quat_xyzw_to_matrix
        cap=a(_cap_view.get_transforms())[0];tool=a(_tool.get_transforms())[0]
        if not all(np.isfinite(p).all() and abs(np.linalg.norm(p[3:])-1.)<.01 for p in (pose,cap,tool)):
            raise RuntimeError('Invalid physics transform')
        neck=pose[:3]+quat_xyzw_to_matrix(pose[3:])@np.array([0,0,.020])
        rc=quat_xyzw_to_matrix(cap[3:]);rt=quat_xyzw_to_matrix(tool[3:])
        gap=float(np.linalg.norm(cap[:3]-neck))
        if not _broken:
            _max_unbroken_gap=max(_max_unbroken_gap,gap)
            rb=quat_xyzw_to_matrix(pose[3:])
            angle=float(np.arccos(np.clip((np.trace(rb.T@rc)-1.)/2.,-1.,1.)))
            _max_unbroken_angle=max(_max_unbroken_angle,angle)
            if gap>.0005:
                _gap_exceed_run_s+=float(manager.get_physics_dt())
                _gap_exceed_total_s+=float(manager.get_physics_dt())
                _max_gap_exceed_run_s=max(_max_gap_exceed_run_s,_gap_exceed_run_s)
            else:_gap_exceed_run_s=0.
        # A submillimetre compliance guard also detects a missing event. Once
        # movement proves separation there is no recovery from an escaped cap.
        if _broken or gap>.0005:
            local=(cap[:3]-tool[:3])@rt
            vertices=(_vertices@rc.T+cap[:3]-tool[:3])@rt
            protected=(np.max(np.abs(local[:2]))<=.0015 and -.041<=local[2]<=-.033
                and np.max(np.abs(vertices[:,:2]))<=.006 and vertices[:,2].min()>=-.039 and vertices[:,2].max()<=.040)
            if not protected:
                _escape=True
                if _first_escape is None:
                    _first_escape=dict(native_break=bool(_broken),neck_gap_m=gap,
                        cap_local=local.tolist(),cap_min=vertices.min(0).tolist(),cap_max=vertices.max(0).tolist())
        any_touch=False
        for sensor in range(counts.shape[0]):
            for item in range(counts.shape[1]):
                n=int(counts[sensor,item]);start=int(starts[sensor,item])
                if n<0 or start<0 or start+n>len(separation):raise RuntimeError('Invalid contact buffer')
                close=separation[start:start+n]<=.0002
                if n and not np.isfinite(separation[start:start+n]).all():raise RuntimeError('Invalid contact separation')
                if n and not np.isfinite(points[start:start+n]).all():raise RuntimeError('Invalid contact points')
                if not close.any():continue
                any_touch=True
                if item==1:_cap_touch=True
                if item==0 and np.any(np.linalg.norm(points[start:start+n][close]-neck,axis=1)<=.006):_rim=True
        _touching=any_touch
        # Read every movable rigid body directly from PhysX after every step,
        # including the four compliant jaws. Never use 20 Hz cached poses.
        motion_vertices=[];motion_quats=[]
        for key,view in _rest_views.items():
            p=a(view.get_transforms())[0];r=quat_xyzw_to_matrix(p[3:])
            motion_vertices.extend(np.asarray(_rest_geometry[key])@r.T+p[:3])
            motion_quats.append(p[3:])
        origin=a(_base.scene.env_origins)[0]
        body_target=np.asarray(_rest_roots['body'])+origin
        tool_target=np.asarray(_rest_roots['tool'])+origin
        rb=quat_xyzw_to_matrix(pose[3:])
        located=(np.max(np.abs(pose[:3]-body_target))<.002 and rb[2,2]>np.cos(np.deg2rad(5.))
                 and np.max(np.abs(tool[:3]-tool_target))<.003 and rt[2,2]>np.cos(np.deg2rad(5.)))
        open_hands=all(np.min(np.abs(a(view.get_dof_positions())[0,ids]))>.035 for view,ids in _fingers)
        _rest.update(motion_vertices,motion_quats,float(manager.get_physics_dt()),
                     _broken and not _escape and located and open_hands and not any_touch)
        _polled=True
    except Exception as e:_error=repr(e)

def read():
    if _error:raise RuntimeError('Ampoule contact monitor failed: '+_error)
    return dict(rim=_rim,cap=_cap_touch,objects_clear=not _touching,
                neck_broken=_broken,cap_escape=_escape,valid=bool(_subscription is not None and _polled),
                max_unbroken_neck_gap_m=_max_unbroken_gap,max_unbroken_relative_angle_rad=_max_unbroken_angle,
                unbroken_gap_exceed_total_s=_gap_exceed_total_s,max_unbroken_gap_exceed_run_s=_max_gap_exceed_run_s,
                first_escape=_first_escape,observed_rest=_rest.diagnostics())
