"""Independent, measurement-driven service scorer. No simulator state writes."""
from dataclasses import dataclass, field
from collections import deque
import math


def add(a, b): return tuple(x+y for x, y in zip(a, b))
def sub(a, b): return tuple(x-y for x, y in zip(a, b))
def mul(a, k): return tuple(x*k for x in a)
def dot(a, b): return sum(x*y for x, y in zip(a, b))
def norm(a): return math.sqrt(dot(a, a))


def rotation(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('Invalid quaternion')
    n = norm(q)
    if abs(n-1) > .001:
        raise ValueError('Quaternion is not normalized')
    x,y,z,w = (v/n for v in q)
    return ((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
            (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
            (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))


def apply(r, p): return tuple(dot(row, p) for row in r)
def column(r, i): return tuple(row[i] for row in r)


@dataclass(frozen=True)
class Cup:
    position: tuple
    quaternion: tuple = (0.,0.,0.,1.)
    linear_velocity: tuple = (0.,0.,0.)
    angular_velocity: tuple = (0.,0.,0.)


@dataclass(frozen=True)
class Capsule:
    a: tuple
    b: tuple
    radius: float


@dataclass(frozen=True)
class Sample:
    t: float
    cups: dict
    shelf_out: float
    button_depth: float
    machine_position: tuple
    hands: tuple
    wall_contacts: frozenset = frozenset()
    saucer_contacts: frozenset = frozenset()
    fingers_open: bool = False
    # These must be populated by verified native readback, not by a demo flag.
    contacts_available: bool = False
    mass_readback_available: bool = False
    mass_kg: dict = field(default_factory=dict)
    mass_volume_ml: dict = field(default_factory=dict)


@dataclass(frozen=True)
class Config:
    good: str = 'cm_mug_good'
    cracked: str = 'cm_mug_cracked'
    nozzle: tuple = (.43,0.,.7475+.22)
    saucer: tuple = (.26,-.19,.7475+.01)
    inner_radius: float = .0365
    outer_radius: float = .0425
    height: float = .095
    floor: float = .004
    target_ml: float = 120.
    duration: float = 25.
    dry_mass: float = .35


def capacity_ml(cup, cfg, up=(0.,0.,1.)):
    """Quasi-static horizontal free surface limited by the lowest mouth point.

    Polar midpoint quadrature integrates cylindrical cross sections. This is
    deliberately not advertised as fluid/slosh simulation.
    """
    r = rotation(cup.quaternion)
    nz = dot(column(r,2),up)
    if nz <= 0:
        return 0.
    h = cfg.height-cfg.floor
    radius = cfg.inner_radius
    if nz > 1-1e-10:
        return math.pi*radius**2*h*1e6
    spill_z = nz*h-radius*math.sqrt(max(0.,1-nz*nz))
    total = 0.
    for i in range(12):
        # Equal-area radial bins: each sample has the same disk area weight.
        rho = radius*math.sqrt((i+.5)/12)
        for j in range(48):
            theta = 2*math.pi*(j+.5)/48
            z = dot(column(r,0),up)*rho*math.cos(theta)+dot(column(r,1),up)*rho*math.sin(theta)
            total += max(0.,min(h,(spill_z-z)/nz))
    return total/(12*48)*math.pi*radius**2*1e6


def stream_intersection(cup, cfg):
    """Distance down the actual stream ray to an unobstructed cup opening."""
    r = rotation(cup.quaternion)
    axis = column(r,2)
    if axis[2] <= .05:
        return None
    mouth = add(cup.position,mul(axis,cfg.height))
    distance = dot(sub(cfg.nozzle,mouth),axis)/axis[2]
    if distance < 0:
        relative=sub(cfg.nozzle,cup.position)
        depth=dot(relative,axis)
        radial=norm(sub(relative,mul(axis,depth)))
        return 0. if cfg.floor<=depth<=cfg.height and radial<cfg.inner_radius-.001 else None
    hit = sub(cfg.nozzle,(0.,0.,distance))
    local = sub(hit,mouth)
    if norm(local) > cfg.inner_radius-.001:
        return None
    return distance


def stream_hits_capsule(capsule, nozzle, bottom):
    """Exact segment/vertical-cylinder intersection for a spherical capsule."""
    a,b = capsule.a,capsule.b
    d = sub(b,a)
    lo,hi = 0.,1.
    low,high = bottom-capsule.radius,nozzle[2]+capsule.radius
    if abs(d[2]) < 1e-12:
        if not low <= a[2] <= high:return False
    else:
        x,y = (low-a[2])/d[2],(high-a[2])/d[2]
        lo,hi = max(lo,min(x,y)),min(hi,max(x,y))
        if lo > hi:return False
    xy = (a[0]-nozzle[0],a[1]-nozzle[1])
    denom = d[0]**2+d[1]**2
    t = lo if denom < 1e-18 else max(lo,min(hi,-(xy[0]*d[0]+xy[1]*d[1])/denom))
    return math.hypot(xy[0]+t*d[0],xy[1]+t*d[1]) <= capsule.radius+.005


class Engine:
    def __init__(self,cfg=Config()):
        self.cfg=cfg
        self.volume={cfg.good:0.,cfg.cracked:0.}
        self.fluid_temp={cfg.good:22.,cfg.cracked:22.}
        self.wall_temp={cfg.good:22.,cfg.cracked:22.}
        self.emitted=0.;self.spilled=0.;self.remaining=0.
        self.faults=set();self.previous=None;self.armed=True;self.brews=0
        self.history=deque(maxlen=41)
        self.value=0.;self.machine_start=None
        self.target=None

    def invalidate(self,reason):
        self.faults.add(reason);self.value=0.

    def _valid(self,s):
        if len(s.machine_position)!=3:return False
        values=[s.t,s.shelf_out,s.button_depth,*s.machine_position]
        if set(s.cups)!={self.cfg.good,self.cfg.cracked} or len(s.hands)<6:
            return False
        for cup in s.cups.values():
            if any(len(v)!=3 for v in (cup.position,cup.linear_velocity,cup.angular_velocity)):return False
            values.extend((*cup.position,*cup.linear_velocity,*cup.angular_velocity))
            rotation(cup.quaternion)
        for hand in s.hands:
            if len(hand.a)!=3 or len(hand.b)!=3:return False
            values.extend((*hand.a,*hand.b,hand.radius))
            if hand.radius <= 0:return False
        for mapping in (s.mass_kg,s.mass_volume_ml):
            if any(not math.isfinite(v) or v<0 for v in mapping.values()):return False
        if not all(math.isfinite(v) for v in values):return False
        if s.shelf_out < -.001 or s.shelf_out > .121:return False
        if not -.001 <= s.button_depth <= .009:return False
        return abs(s.t-.05)<1e-5 if self.previous is None else abs(s.t-self.previous.t-.05)<1e-5

    def _thermal(self,name,incoming,dt):
        # ml * J/(ml K): density is 1 kg/litre. Parameters are estimates.
        old=self.volume[name];cp=4.18
        if incoming:
            self.fluid_temp[name]=(old*self.fluid_temp[name]+incoming*87.)/(old+incoming)
        self.volume[name]+=incoming
        if self.volume[name]:
            fluid_c=self.volume[name]*cp;wall_c=self.cfg.dry_mass*840.
            difference=self.fluid_temp[name]-self.wall_temp[name]
            exchange=math.copysign(min(abs(50.*difference*dt),abs(difference)/(1/fluid_c+1/wall_c)),difference)
            self.fluid_temp[name]-=exchange/fluid_c
            self.wall_temp[name]+=exchange/wall_c
        self.wall_temp[name]+=(22.-self.wall_temp[name])*1.5*dt/(self.cfg.dry_mass*840.)

    def update(self,s):
        try:
            if not self._valid(s):raise ValueError('invalid or incomplete sample')
        except (ValueError,TypeError,OverflowError,AttributeError):
            self.invalidate('invalid_telemetry');return 0.
        if self.machine_start is None:self.machine_start=s.machine_position
        if norm(sub(s.machine_position,self.machine_start))>.002:
            self.invalidate('machine_mount_moved')
        if s.button_depth<=.002:self.armed=True
        if self.armed and s.button_depth>=.006:
            self.armed=False
            if self.remaining<=1e-9:
                self.remaining=self.cfg.target_ml;self.brews+=1
        incoming={name:0. for name in self.volume}
        self.target=None
        if self.remaining>0:
            flow=min(self.remaining,self.cfg.target_ml/self.cfg.duration*.05)
            self.remaining-=flow;self.emitted+=flow
            catches=[(distance,name) for name,cup in s.cups.items()
                     if (distance:=stream_intersection(cup,self.cfg)) is not None]
            distance,name=min(catches) if catches else (self.cfg.nozzle[2]-.7475,None)
            if name is None:self.spilled+=flow
            else:incoming[name]=flow;self.target=name
            bottom=self.cfg.nozzle[2]-distance
            if name is not None:
                cup=s.cups[name];axis=column(rotation(cup.quaternion),2)
                floor_center=add(cup.position,mul(axis,self.cfg.floor))
                floor_distance=dot(sub(self.cfg.nozzle,floor_center),axis)/axis[2]
                bottom=max(.7475,self.cfg.nozzle[2]-max(0.,floor_distance))
            if any(stream_hits_capsule(h,self.cfg.nozzle,bottom) for h in s.hands):
                self.invalidate('hand_in_hot_stream')
            if self.previous:
                for old,new in zip(self.previous.hands,s.hands):
                    extent=max(norm(sub(new.a,old.a)),norm(sub(new.b,old.b)))
                    count=max(1,math.ceil(extent/.002))
                    if count>200:self.invalidate('unresolved_fast_hand_motion');break
                    for k in range(1,count):
                        f=k/count
                        swept=Capsule(add(old.a,mul(sub(new.a,old.a),f)),add(old.b,mul(sub(new.b,old.b),f)),max(old.radius,new.radius))
                        if stream_hits_capsule(swept,self.cfg.nozzle,bottom):self.invalidate('hand_in_hot_stream')
        for name,cup in s.cups.items():
            self._thermal(name,incoming[name],.05)
            up=(0.,0.,1.)
            if self.previous:
                acceleration=mul(sub(cup.linear_velocity,self.previous.cups[name].linear_velocity),20.)
                effective_up=add(acceleration,(0.,0.,9.81))
                if norm(effective_up)>.01:up=mul(effective_up,1/norm(effective_up))
                else:up=(0.,0.,-1.)
            overflow=max(0.,self.volume[name]-capacity_ml(cup,self.cfg,up))
            self.volume[name]-=overflow;self.spilled+=overflow
            if name==self.cfg.cracked and incoming[name]>0:self.invalidate('cracked_cup_used')
            if name in s.wall_contacts and self.wall_temp[name]>=43.:
                self.invalidate('hot_wall_contact')
            if cup.position[2]<.74:self.invalidate('cup_off_worktop')
        if abs(self.emitted-self.spilled-sum(self.volume.values()))>1e-6:
            self.invalidate('liquid_mass_not_conserved')
        self.history.append(s);self.previous=s
        self.value=self.evaluate(s)
        return self.value

    def evaluate(self,s):
        if self.faults:return 0.
        volume=self.volume[self.cfg.good]
        if volume<20:return 0.
        if volume<110:return .25
        cup=s.cups[self.cfg.good];r=rotation(cup.quaternion)
        placed=(norm(sub(cup.position[:2],self.cfg.saucer[:2]))<=.015 and
                abs(cup.position[2]-self.cfg.saucer[2])<=.003 and r[2][2]>=math.cos(math.radians(3)) and
                self.cfg.good in s.saucer_contacts)
        if not placed:return .5
        if not(self.remaining<=1e-9 and 110<=volume<=130 and self.spilled<=2 and self.brews==1 and s.shelf_out<=.003 and s.fingers_open):return .75
        # Full credit requires geometry, contacts and liquid-weight native readback.
        if not(s.contacts_available and s.mass_readback_available):return .75
        for name in self.volume:
            if name not in s.mass_kg or name not in s.mass_volume_ml:return .75
            if abs(s.mass_kg[name]-(self.cfg.dry_mass+s.mass_volume_ml[name]/1000))>1e-5:return .75
            # Native mass is updated after each sample. At most one emitted step
            # may separate the measured mass and current liquid ledger.
            if abs(s.mass_volume_ml[name]-self.volume[name])>.25:return .75
        if len(self.history)<41:return .75
        for row in self.history:
            c=row.cups[self.cfg.good]
            if norm(sub(c.position,cup.position))>.001 or norm(c.linear_velocity)>.005 or norm(c.angular_velocity)>.05:return .75
            if abs(dot(c.quaternion,cup.quaternion))<math.cos(math.radians(1)/2):return .75
            if self.cfg.good not in row.saucer_contacts or not row.fingers_open:return .75
            # Conservative analytic finger/palm clearance to the whole mug.
            for h in row.hands:
                for k in range(9):
                    p=add(h.a,mul(sub(h.b,h.a),k/8))
                    local=apply(tuple(zip(*r)),sub(p,cup.position))
                    radial=max(0.,math.hypot(local[0],local[1])-self.cfg.outer_radius)
                    vertical=max(0.,-local[2],local[2]-self.cfg.height)
                    if math.hypot(radial,vertical)<h.radius+.03:return .75
                    for j in range(25):
                        a=-math.pi/2+j*math.pi/24
                        handle=(-.045-.032*math.cos(a),0.,.055+.026*math.sin(a))
                        if norm(sub(local,handle))<h.radius+.007+.03:return .75
                    # Final release also covers the machine and shelf grip bar.
                    for low,high in [((.29,-.085,.7475),(.63,.085,1.0675)),
                                     ((.245-row.shelf_out,.1575,.8135),(.325-row.shelf_out,.1825,.8335))]:
                        separation=tuple(max(lo-v,0.,v-hi) for v,lo,hi in zip(p,low,high))
                        if norm(separation)<h.radius+.03:return .75
        return 1.

    def report(self):
        return dict(score=self.value,retained_ml=dict(self.volume),emitted_ml=self.emitted,
                    spilled_ml=self.spilled,remaining_ml=self.remaining,brews=self.brews,
                    wall_temperature_c=dict(self.wall_temp),faults=sorted(self.faults))
