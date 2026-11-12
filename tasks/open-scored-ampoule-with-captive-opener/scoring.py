"""Pure measured-state scorer. No force or torque sensor is a scoring tool."""
from dataclasses import dataclass
import math
import numpy as np

@dataclass
class Sample:
    time: float
    neck_gap_m: float
    cap_in_opener: bool
    cap_covered: bool
    body_upright_deg: float
    body_in_holder: bool
    opener_in_rack: bool
    bodies_slow: bool
    hands_open_clear: bool
    rim_contact: bool=False
    cap_contact: bool=False
    dropped: bool=False
    opener_outside_table: bool=False
    neck_broken: bool=False
    cap_escape: bool=False
    monitor_valid: bool=True

class Scorer:
    def __init__(self):
        self.failed=False
        self.opened=False
        self.fractured=False
        self.hold_start=None
        self.last_time=None
        self.reason='start'
    def update(self,s):
        if (not all(math.isfinite(float(v)) for v in (s.time,s.neck_gap_m,s.body_upright_deg))
                or (self.last_time is not None and (s.time<self.last_time or s.time-self.last_time>.100001))):
            self.failed=True;self.reason='invalid observation'
        self.last_time=s.time
        if (s.rim_contact or s.cap_contact or s.cap_escape or s.dropped or s.opener_outside_table
                or not s.monitor_valid or s.body_upright_deg>15.):
            self.failed=True;self.reason='handling constraint violated'
        if self.failed:self.hold_start=None;return 0.
        # Capture is required from the first detected break. Twelve millimetres
        # is only the progress clearance, never the start of the safety rule.
        if s.neck_broken or s.neck_gap_m>.0005:self.fractured=True
        if self.fractured and (not s.cap_in_opener or not s.cap_covered):
            self.failed=True;self.reason='cap escaped after fracture';return 0.
        if s.neck_gap_m>=.012 and s.neck_broken:
            if not s.cap_in_opener or not s.cap_covered:
                self.failed=True;self.reason='uncaptured break';return 0.
            self.opened=True
        if self.opened and (not s.cap_in_opener or not s.cap_covered):
            self.failed=True;self.reason='cap escaped';return 0.
        if not self.opened:
            self.reason='cap covered' if s.cap_in_opener and s.cap_covered else 'not opened'
            return .25 if s.cap_in_opener and s.cap_covered else 0.
        if not (s.body_in_holder and s.opener_in_rack):
            self.hold_start=None;self.reason='opened and retained';return .5
        if not (s.bodies_slow and s.hands_open_clear):
            self.hold_start=None;self.reason='racked but not released';return .75
        if self.hold_start is None:self.hold_start=s.time
        self.reason='released hold'
        return 1. if s.time-self.hold_start>=1.-1e-9 else .9
