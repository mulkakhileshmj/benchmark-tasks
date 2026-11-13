"""Native adapter for the independent scorer; unvalidated until a GPU pilot.

Uses attributed upstream fixture code. Never writes a cup/shelf/button pose.
Only display markers and explicitly modeled liquid mass properties are changed.
"""
import json
import os
from pathlib import Path
import numpy as np
import parts
from engine import Engine, Config, Cup, Capsule, Sample
from geometry import MACHINE_NAME, MUG_GOOD_NAME, MUG_CRACKED_NAME, SHELF_NAME, BUTTON_NAME

ENGINE=Engine()
CONTACTS=set()
SUBSCRIPTION=None
MASS_MODELS={}
MASS_VOLUME={}
MASS_OK=False
LAST_DIAGNOSTIC={}
RESET_TIME=0.
DISPLAY_LEDGER=None
TENSOR_CONTACTS=None
CAMERA_SHAPES={}


def trace(payload):
    path=os.environ.get('TASK_TRACE')
    if path:
        with open(path,'a',encoding='utf8') as stream:
            stream.write(json.dumps(payload,allow_nan=False)+'\n')


def array(value):
    if hasattr(value,'detach'):return value.detach().cpu().numpy()
    if hasattr(value,'numpy'):return value.numpy()
    if hasattr(value,'torch'):return array(value.torch)
    return np.asarray(value)



# A physically bolted 3 kg fixture, rather than a loose 25 kg surrogate.
_original_author=parts.author_part
def _bolted_author(stage,prim_path,spec,*args,**kwargs):
    prim=_original_author(stage,prim_path,spec,*args,**kwargs)
    if spec.name in (MUG_GOOD_NAME,MUG_CRACKED_NAME):
        from pxr import PhysxSchema
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(prim_path+'/body')).CreateThresholdAttr(0.)
    if spec.name==MACHINE_NAME:
        from pxr import UsdPhysics,Gf
        joint=UsdPhysics.FixedJoint.Define(stage,prim_path+'/bench_bolt')
        joint.CreateBody1Rel().SetTargets([prim_path+'/body'])
        translation=kwargs.get('translation')
        if translation is None:translation=spec.pos
        joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,translation)))
    return prim
parts.author_part=_bolted_author


def _contacts(environment):
    global SUBSCRIPTION
    if SUBSCRIPTION is not None:return
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools
    def receive(headers,data):
        from omni.physx.bindings._physx import ContactEventType
        for event in headers:
            a,b=(str(PhysicsSchemaTools.intToSdfPath(p)) for p in (event.collider0,event.collider1))
            key=tuple(sorted((a,b)))
            if event.type==ContactEventType.CONTACT_LOST:CONTACTS.discard(key)
            elif event.num_contact_data>0:CONTACTS.add(key)
    SUBSCRIPTION=get_physx_simulation_interface().subscribe_contact_report_events(receive)
    if SUBSCRIPTION is None:raise RuntimeError('Native contacts are unavailable')


def _enable_reports(environment):
    from pxr import PhysxSchema
    import omni.usd
    stage=omni.usd.get_context().get_stage()
    for name in (MUG_GOOD_NAME,MUG_CRACKED_NAME):
        asset=environment.unwrapped.scene[name]
        for path in asset.root_physx_view.prim_paths:
            prim=stage.GetPrimAtPath(path)
            if not prim.IsValid():raise RuntimeError('Cannot enable cup contact reports')
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)


def _hand_capsules(environment):
    from engine import rotation
    from hand_envelopes import envelopes
    base=environment.unwrapped;origin=array(base.scene.env_origins[0]);result=[]
    for arm in ('left','right'):
        robot=base.scene[arm+'_robot'];poses=array(robot.data.body_link_pose_w)[0]
        for name,pieces in envelopes().items():
            pose=poses[list(robot.body_names).index(name)];R=np.asarray(rotation(tuple(pose[3:])));p=pose[:3]-origin
            for a,b,radius in pieces:result.append(Capsule(tuple(p+R@a),tuple(p+R@b),radius))
    return tuple(result)


def _native_contacts():
    if TENSOR_CONTACTS is None:raise RuntimeError('Native GPU contact views not initialized')
    return TENSOR_CONTACTS.read(float(TENSOR_CONTACTS.environment.unwrapped.physics_dt))
    walls=set();saucers=set()
    for a,b in CONTACTS:
        for cup,other in ((a,b),(b,a)):
            name=next((n for n in (MUG_GOOD_NAME,MUG_CRACKED_NAME) if n in cup.split('/')),None)
            if name is None:continue
            # cyl_0 is the floor; cyl_1 onward are the wall. Handle boxes are safe.
            if 'robot' in other.lower() or 'arm' in other.lower():
                if '/cyl_' in cup and not cup.endswith('/cyl_0'):walls.add(name)
            own='cm_saucer_good' if name==MUG_GOOD_NAME else 'cm_saucer_cracked'
            if own in other.split('/') and cup.endswith('/cyl_0'):saucers.add(name)
    return frozenset(walls),frozenset(saucers)


def _payload_native(environment,name,volume_ml):
    """Experimental tensor-API mass/COM/inertia update with mandatory readback.

    Native API compatibility is an explicit GPU gate. No exception is swallowed.
    Base inertia is expressed in its native COM orientation; the combined tensor
    is diagonalized into a new principal frame. No velocity/pose is imposed.
    """
    import warp as wp
    from engine import rotation
    from yam_arena.ik import matrix_to_quat_xyzw
    asset=environment.unwrapped.scene[name];view=asset.root_physx_view
    if name in MASS_VOLUME and abs(MASS_VOLUME[name]-volume_ml)<1e-8:
        return float(array(view.get_masses()).reshape(-1)[0])
    if name not in MASS_MODELS:
        MASS_MODELS[name]=(array(view.get_masses()).copy(),array(view.get_coms()).copy(),array(view.get_inertias()).copy())
    base_mass,base_com,base_inertia=MASS_MODELS[name]
    m0=float(base_mass.reshape(-1)[0]);mf=volume_ml/1000.
    if abs(m0-.35)>1e-4:raise RuntimeError('Unexpected dry cup mass')
    old=base_com.reshape(-1,7)[0]
    r=np.asarray(rotation(tuple(old[3:])));i0=base_inertia.reshape(-1,3,3)[0]
    h=volume_ml/1e6/(np.pi*ENGINE.cfg.inner_radius**2)
    cf=np.array([0.,0.,ENGINE.cfg.floor+h/2]);c=(m0*old[:3]+mf*cf)/(m0+mf)
    def parallel(m,v):return m*(np.dot(v,v)*np.eye(3)-np.outer(v,v))
    radius=ENGINE.cfg.inner_radius
    inertia=i0+parallel(m0,old[:3]-c)+parallel(mf,cf-c)
    inertia+=np.diag([mf*(3*radius**2+h*h)/12]*2+[mf*radius**2/2])
    # Tensor API accepts the full COM-centred inertia in the body-prim frame.
    # set_inertias computes its principal axes, so do not pre-diagonalize it.
    new_com=np.r_[c,old[3:]].astype(np.float32).reshape(base_com.shape)
    new_i=inertia.astype(np.float32).reshape(base_inertia.shape)
    new_mass=np.full_like(base_mass,m0+mf,dtype=np.float32)
    ids=wp.array([0],dtype=wp.int32,device='cpu')
    # PhysX versions differ on whether these updates are legal under DirectGPU.
    # Unsupported native setters deliberately abort the pilot instead of
    # allowing a full-score marker-only liquid demonstration.
    view.set_coms(wp.from_numpy(new_com,dtype=wp.float32,device='cpu'),ids)
    view.set_inertias(wp.from_numpy(new_i,dtype=wp.float32,device='cpu'),ids)
    view.set_masses(wp.from_numpy(new_mass,dtype=wp.float32,device='cpu'),ids)
    measured=float(array(view.get_masses()).reshape(-1)[0])
    com=array(view.get_coms()).reshape(-1,7)[0]
    inert=array(view.get_inertias()).reshape(-1,9)[0]
    if abs(measured-(m0+mf))>1e-5 or np.linalg.norm(com[:3]-c)>1e-5 or abs(float(np.linalg.norm(com[3:]))-1.)>1e-5 or np.max(np.abs(inert-new_i.reshape(-1,9)[0]))>1e-7:
        Path('/home/ubuntu/coffee-readback.json').write_text(json.dumps({'expected_mass':m0+mf,'mass':measured,'com':com.tolist(),'target_com':new_com.reshape(-1,7)[0].tolist(),'inertia':inert.tolist(),'target_inertia':new_i.reshape(-1,9)[0].tolist()}))
        raise RuntimeError('Liquid payload readback did not match command')
    MASS_VOLUME[name]=volume_ml
    return measured


def _payload(environment,name,volume_ml):
    try:return _payload_native(environment,name,volume_ml)
    except BaseException:
        import traceback
        Path('/home/ubuntu/coffee-payload-error.txt').write_text(traceback.format_exc())
        raise

def reset(environment):
    global ENGINE,MASS_OK,RESET_TIME,DISPLAY_LEDGER,TENSOR_CONTACTS,CAMERA_SHAPES
    from isaaclab.sim import SimulationContext
    from display_state import new_ledger
    from display import DISPLAY
    ENGINE=Engine();CONTACTS.clear();MASS_OK=False
    CAMERA_SHAPES={}
    RESET_TIME=float(environment.unwrapped._sim_step_counter)*float(environment.unwrapped.physics_dt)
    _contacts(environment);_enable_reports(environment)
    from native_contacts import CupContacts
    TENSOR_CONTACTS=CupContacts(environment,(MUG_GOOD_NAME,MUG_CRACKED_NAME))
    for name in (MUG_GOOD_NAME,MUG_CRACKED_NAME):
        _payload(environment,name,1.)
        _payload(environment,name,0.)
    MASS_OK=True
    if os.getenv('GRASP_FILLED'):
        if not os.getenv('GRASP_TEST'):raise RuntimeError('Filled fixture is allowed only for grasp diagnostics')
        v=ENGINE.cfg.target_ml
        ENGINE.volume[MUG_GOOD_NAME]=v;ENGINE.emitted=v
        ENGINE.fluid_temp[MUG_GOOD_NAME]=87.;ENGINE.wall_temp[MUG_GOOD_NAME]=60.
        _payload(environment,MUG_GOOD_NAME,v)
    DISPLAY_LEDGER=new_ledger()
    DISPLAY.reset(environment,DISPLAY_LEDGER)
    if DISPLAY._off:raise RuntimeError('Required coffee display failed to initialize')


def step(environment):
    global LAST_DIAGNOSTIC
    try:
        _contacts(environment)
        if not CAMERA_SHAPES and not os.getenv('DIAGNOSTIC_NO_CAMERAS'):
            CAMERA_SHAPES.update({n:list(environment.unwrapped.scene[n].data.output['rgb'].shape) for n in ('top_cam','left_cam','right_cam')})
        states={n:parts._read_part_state(environment,parts._PARTS[n])
                for n in (MACHINE_NAME,MUG_GOOD_NAME,MUG_CRACKED_NAME)}
        machine=states[MACHINE_NAME]
        walls,saucers=_native_contacts()
        masses={n:float(array(environment.unwrapped.scene[n].root_physx_view.get_masses()).reshape(-1)[0])
                for n in (MUG_GOOD_NAME,MUG_CRACKED_NAME)}
        from isaaclab.sim import SimulationContext
        s=Sample(t=float(environment.unwrapped._sim_step_counter)*float(environment.unwrapped.physics_dt)-RESET_TIME,
            cups={n:Cup(tuple(v.pos),tuple(v.quat_xyzw),tuple(v.lin_vel),tuple(v.ang_vel))
                  for n,v in states.items() if n!=MACHINE_NAME},
            shelf_out=-float(machine.sliders[SHELF_NAME]),button_depth=float(machine.sliders[BUTTON_NAME]),
            machine_position=tuple(machine.pos),hands=_hand_capsules(environment),
            wall_contacts=walls,saucer_contacts=saucers,
            fingers_open=all(parts.gripper_open(environment,a,.9) for a in ('left','right')),
            contacts_available=SUBSCRIPTION is not None,mass_readback_available=MASS_OK,
            mass_kg=masses,mass_volume_ml=dict(MASS_VOLUME))
        ENGINE.update(s)
        masses={name:_payload(environment,name,volume) for name,volume in ENGINE.volume.items()}
        from display import DISPLAY
        DISPLAY_LEDGER['step']=round(s.t*20)
        DISPLAY_LEDGER['target']=ENGINE.target
        DISPLAY_LEDGER['brew_step']=round((ENGINE.cfg.target_ml-ENGINE.remaining)/ENGINE.cfg.target_ml*500) if ENGINE.remaining>1e-9 else None
        DISPLAY_LEDGER['level']={n:v/1e6/(np.pi*ENGINE.cfg.inner_radius**2*(ENGINE.cfg.height-ENGINE.cfg.floor)) for n,v in ENGINE.volume.items()}
        DISPLAY_LEDGER['tiles']=['plate']*min(40,int(ENGINE.spilled/.5))
        DISPLAY_LEDGER['steam_mug']=MUG_GOOD_NAME
        DISPLAY_LEDGER['steam_until']=DISPLAY_LEDGER['step']+1 if ENGINE.fluid_temp[MUG_GOOD_NAME]>55 else -1
        DISPLAY.update(environment,DISPLAY_LEDGER,{n:(tuple(v.pos),tuple(v.quat_xyzw)) for n,v in states.items() if n!=MACHINE_NAME},s.shelf_out<=.003)
        if DISPLAY._off:raise RuntimeError('Coffee display failed during trial')
        LAST_DIAGNOSTIC=ENGINE.report()
        robot_diagnostics={}
        for arm in ('left','right'):
            robot=environment.unwrapped.scene[arm+'_robot']
            pose=array(robot.data.body_link_pose_w)[0]
            robot_diagnostics[arm]={'joint_names':list(robot.joint_names),'q':array(robot.data.joint_pos)[0].tolist(),
                'hands':{n:pose[list(robot.body_names).index(n)].tolist() for n in ('gripper','tip_left','tip_right')}}
        trace(dict(t=s.t,native_camera_shapes=CAMERA_SHAPES,diagnostic_camera_off=bool(os.getenv('DIAGNOSTIC_NO_CAMERAS')),diagnostic_filled_fixture=bool(os.getenv('GRASP_FILLED')),native_hand_capsule_count=len(s.hands),native_force_readback=TENSOR_CONTACTS.diagnostics,robots=robot_diagnostics,contacts_available=s.contacts_available,mass_readback_available=s.mass_readback_available,
                   mass_kg=masses,mass_volume_ml=dict(MASS_VOLUME),shelf_out_m=s.shelf_out,
                   button_depth_m=s.button_depth,
                   cups={n:dict(position=v.position,quaternion=v.quaternion,linear_velocity=v.linear_velocity,
                                angular_velocity=v.angular_velocity) for n,v in s.cups.items()},
                   native_contacts=sorted(CONTACTS),**LAST_DIAGNOSTIC))
    except Exception as error:
        import traceback
        Path('/home/ubuntu/coffee-step-error.txt').write_text(traceback.format_exc())
        ENGINE.invalidate('native_adapter_failure')
        trace(dict(error=type(error).__name__,message=str(error),**ENGINE.report()))
        raise RuntimeError('Coffee native pilot failed closed') from error


parts.on_reset(reset)
parts.on_step(step)
def score(objects):return ENGINE.value
def success(objects):return ENGINE.value==1.
def distance(objects):return 1.-ENGINE.value
def diagnostics():return LAST_DIAGNOSTIC
