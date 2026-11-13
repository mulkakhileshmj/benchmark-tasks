"""Filtered native GPU contact forces and points, independent of policy actions."""
import numpy as np

def arr(v):
    if hasattr(v,'detach'):return v.detach().cpu().numpy()
    if hasattr(v,'numpy'):return v.numpy()
    if hasattr(v,'torch'):return arr(v.torch)
    return np.asarray(v)

class CupContacts:
    def __init__(self,environment,names):
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdPhysics
        import omni.usd
        self.sim=PhysxManager.get_physics_sim_view();self.views={};self.paths={};self.environment=environment
        stage=omni.usd.get_context().get_stage()
        hands=[str(p.GetPath()) for p in stage.Traverse() if p.GetName() in ('gripper','tip_left','tip_right') and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(hands)!=6:raise RuntimeError('Expected six native hand rigid bodies')
        for name in names:
            body=environment.unwrapped.scene[name].root_physx_view.prim_paths[0]
            own='cm_saucer_good' if name=='cm_mug_good' else 'cm_saucer_cracked'
            prefix=body.split('/'+name+'/')[0]
            hand=self.sim.create_rigid_contact_view([body],filter_patterns=[hands],max_contact_data_count=512)
            saucer_shapes=[str(p.GetPath()) for p in stage.Traverse() if own in str(p.GetPath()).split('/') and p.HasAPI(UsdPhysics.CollisionAPI)]
            if not saucer_shapes:raise RuntimeError('No actual saucer collider paths found')
            saucer=self.sim.create_rigid_contact_view([body],filter_patterns=[saucer_shapes],max_contact_data_count=128)
            if hand.sensor_count!=1 or saucer.sensor_count!=1:raise RuntimeError('Cup contact views did not resolve one body')
            self.views[name]=(hand,saucer)
            self.paths[name]={'body':body,'hands':hands,'saucer_colliders':saucer_shapes}
        self.hand_paths=hands
        import json
        from pathlib import Path
        Path('/home/ubuntu/coffee-contact-paths.json').write_text(json.dumps(self.paths,indent=2))
        self.diagnostics={}
    def read(self,dt):
        from engine import rotation
        walls=set();saucers=set();self.diagnostics={}
        for name,(hand,saucer) in self.views.items():
            force=arr(hand.get_contact_force_matrix(dt=dt));sf=arr(saucer.get_contact_force_matrix(dt=dt))
            if not np.isfinite(force).all() or not np.isfinite(sf).all():raise RuntimeError('Nonfinite native contact tensor')
            hn=float(np.linalg.norm(force.reshape(-1,3),axis=1).max(initial=0.));sn=float(np.linalg.norm(sf.reshape(-1,3),axis=1).max(initial=0.))
            self.diagnostics[name]={'hand_normal_n':hn,'saucer_normal_n':sn,
                'per_hand_body_normal_n':dict(zip(self.hand_paths,np.linalg.norm(force.reshape(-1,3),axis=1).astype(float).tolist()))}
            if sn>.001:saucers.add(name)
            if hn>.001:
                _,points,_,_,counts,starts=hand.get_contact_data(dt=dt)
                pts=arr(points);counts=arr(counts).reshape(-1).astype(int);starts=arr(starts).reshape(-1).astype(int)
                valid=np.concatenate([pts[s:s+n] for s,n in zip(starts,counts) if n>0]) if counts.sum()>0 else np.empty((0,3))
                if not len(valid):raise RuntimeError('Nonzero hand force without native contact points')
                asset=self.environment.unwrapped.scene[name];pose=arr(asset.data.root_link_pose_w)[0];R=np.asarray(rotation(tuple(pose[3:])))
                local=(valid-pose[:3])@R;radius=np.linalg.norm(local[:,:2],axis=1)
                if np.any((radius>=.032)&(radius<=.046)&(local[:,2]>=.003)&(local[:,2]<=.100)):walls.add(name)
                self.diagnostics[name]['hand_contact_points']=valid.tolist()
        return frozenset(walls),frozenset(saucers)
