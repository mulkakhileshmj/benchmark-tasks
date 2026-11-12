"""Rest measured by every physical vertex and orientation at native substeps.

Bounds are conservative: the diagonal of each vertex's accumulated position
box bounds every pairwise displacement. A failed gate or excursion restarts
the entire observation interval. No filtering or velocity/state writes.
"""
import math
import numpy as np

class ObservedRest:
    def __init__(self, duration=1., displacement=.0005, angle=math.radians(1.)):
        self.duration=duration;self.displacement=displacement;self.angle=angle
        self.reset()
    def reset(self):
        self.elapsed=0.;self.low=self.high=None;self.quats=[]
        self.excursion=0.;self.rotation=0.;self.settled=False
    def update(self, vertices, quaternions, dt, eligible):
        v=np.asarray(vertices,dtype=float);q=np.asarray(quaternions,dtype=float)
        if not (np.isfinite(v).all() and np.isfinite(q).all() and 0.<dt<=.02):
            raise ValueError('Invalid native rest sample')
        norms=np.linalg.norm(q,axis=1)
        if np.any(abs(norms-1.)>.01):raise ValueError('Invalid rest quaternion')
        q=q/norms[:,None]
        if not eligible:self.reset();return False
        if self.low is None:
            self.low=v.copy();self.high=v.copy();self.quats=[q];return False
        lo=np.minimum(self.low,v);hi=np.maximum(self.high,v)
        excursion=float(np.linalg.norm(hi-lo,axis=1).max())
        dots=np.abs(np.einsum('nb,knb->kn',q,np.array(self.quats)))
        rotation=float(2*np.arccos(np.clip(dots.min(),-1.,1.)))
        if excursion>self.displacement or rotation>self.angle:
            self.reset();self.low=v.copy();self.high=v.copy();self.quats=[q]
            return False
        self.low=lo;self.high=hi;self.quats.append(q)
        # Keep bounded history after qualification: restarting a fresh interval
        # does not allow a moving object to pass because extrema remain intact.
        if len(self.quats)>2000:
            self.reset();self.low=v.copy();self.high=v.copy();self.quats=[q]
            return False
        self.elapsed+=dt;self.excursion=excursion;self.rotation=rotation
        self.settled=self.elapsed>=self.duration-1e-9
        return self.settled
    def diagnostics(self):
        return dict(settled=self.settled,elapsed_s=self.elapsed,
                    vertex_excursion_bound_m=self.excursion,rotation_excursion_rad=self.rotation)
