"""Score stopper retention and a handle-driven open/close stroke."""
from dataclasses import dataclass
import math
from design import gauge


@dataclass
class Sample:
    time: float
    slide: float
    angle: float
    height: float
    plug_clear: bool
    all_clear: bool
    speed: float


class Score:
    def __init__(self):
        self.opened = False
        self.hold = None
        self.value = 0.
        self.last = None
        self.clean_stroke = False

    def update(self, s):
        if not all(map(math.isfinite, [s.time,s.height,s.angle,s.slide,s.speed])):
            raise ValueError("Invalid drain telemetry")
        if self.last is not None and not 0 < s.time-self.last <= .101:
            raise ValueError("Invalid control interval")
        self.last = s.time
        retained = s.slide >= -.00075 and gauge(s.angle,s.slide,s.height)
        if not retained:
            self.opened=False
        if not (retained and s.plug_clear):
            self.clean_stroke=False
            self.opened=False
        elif s.height <= .0006 and abs(s.angle) <= math.radians(4):
            self.clean_stroke=True
        if self.clean_stroke and s.height >= .008 and s.angle >= math.radians(20):
            self.opened=True
        closed = retained and self.opened and s.height <= .0006 and abs(s.angle) <= math.radians(4)
        if closed and s.all_clear and s.speed < .002:
            if self.hold is None:
                self.hold=s.time
        else:
            self.hold=None
        self.value = (1. if self.hold is not None and s.time-self.hold >= 1.-1e-8
                      else .65 if self.opened else .25 if retained else 0.)
        return self.value
