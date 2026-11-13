"""Official YAM simulation with explicit native sensor resolution and diagnostic camera-off mode."""
import os,atexit
import numpy as np

def embodiment(**kwargs):
    from yam_arena import inspect_embodiment as core
    class CoffeeYam(core.YamArenaEmbodiment):
        def _ensure_sim(self,goal):
            diagnostic=bool(os.getenv('DIAGNOSTIC_NO_CAMERAS'))
            if diagnostic and not (os.getenv('GRASP_TEST') or os.getenv('DIAGNOSTIC_FULL_SEQUENCE')):raise RuntimeError('Camera-off mode is restricted to grasp diagnostics')
            if core._APP is None:
                from isaaclab.app import AppLauncher
                os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
                count=int(os.environ.get('YAM_LIMIT_CPU_THREADS','4'))
                if not 1<=count<=8:raise ValueError('Invalid CPU thread bound')
                opts=dict(headless=self.headless,device=self.device,enable_cameras=not diagnostic,limit_cpu_threads=count,
                    kit_args=f'--/plugins/carb.tasking.plugin/threadCount={count} --/plugins/omni.tbb.globalcontrol/maxThreadCount={count} --/persistent/physics/numThreads={count}')
                core._APP=AppLauncher(**opts).app;atexit.register(core._close_app_at_exit)
            from yam_arena import embodiment as native
            if not getattr(native.BimanualYamEmbodiment,'_explicit_camera_resolution',False):
                original=native.BimanualYamEmbodiment;width=self.cam_width;height=self.cam_height
                class SensorSizedYam(original):
                    _explicit_camera_resolution=True
                    def __init__(self,*args,**opts):
                        opts.update(enable_cameras=not diagnostic,cam_width=width,cam_height=height)
                        super().__init__(*args,**opts)
                native.BimanualYamEmbodiment=SensorSizedYam
            return super()._ensure_sim(goal)
        def _observation(self):
            if not os.getenv('DIAGNOSTIC_NO_CAMERAS'):return super()._observation()
            arms=[]
            for name in core.ARMS:
                arm_ids,finger_ids=self._ids[name];q=self._base.scene[name].data.joint_pos.torch[0]
                joints=q[arm_ids].detach().cpu().numpy().astype(np.float64)
                grip=float(np.clip(q[finger_ids].mean().item()/core.FINGER_OPEN,0.,1.))
                arms.append(np.r_[joints,grip])
            return core.Observation(images={},state={core.STATE_KEY:core.pack(*arms)},instruction=self._instruction,
                image_times={},state_time=self._t)
    return CoffeeYam(**kwargs)
