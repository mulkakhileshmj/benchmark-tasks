"""Pure measured-state scorer. No controller or simulator imports."""
import math

class SupportScore:
    def __init__(self):
        self.bad=False;self.reason=None;self.since=None;self.last=None;self.value=0.

    def update(self, *, t, opening_deg, angle_errors_deg, root_error_m,
               root_tilt_deg, root_yaw_deg, contacts, released, at_rest,
               on_table, spine_clear=False, finite=True):
        values=[t,opening_deg,root_error_m,root_tilt_deg,root_yaw_deg,*angle_errors_deg]
        valid=finite and all(math.isfinite(float(v)) for v in values)
        if not valid or not on_table or opening_deg>115.:
            self.bad=True
            self.reason='nonfinite state' if not valid else 'off table' if not on_table else 'opening limit exceeded'
        if self.last is not None and (not valid or t<self.last or t-self.last>.100001):self.since=None
        self.last=t if valid else None
        if self.bad:self.since=None;self.value=0.;return self.value
        # A generic lift or transport of a closed volume earns no credit.
        aligned=(root_error_m<=.005 and root_tilt_deg<=3. and root_yaw_deg<=5.)
        spread=all(abs(v)<=3. for v in angle_errors_deg) and 104.<=opening_deg<=115.
        support=list(contacts)
        if len(support)!=4:raise ValueError('Four independent support patches are required')
        if not aligned or not spread or not spine_clear:self.since=None;self.value=0.;return self.value
        self.value=.25 if any(support) else 0.
        if any(support[:2]) and any(support[2:]):self.value=.5
        if all(support):self.value=.75
        qualified=all(support) and released and at_rest
        if qualified:
            if self.since is None:self.since=t
            if t-self.since>=1.-1e-9:self.value=1.
        else:self.since=None
        return self.value
