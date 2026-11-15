"""Pure geometry checks for the IV task's aseptic field."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def point_segment_distance(point, a, b) -> float:
    point, a, b = (np.asarray(v, float) for v in (point, a, b))
    ab = b - a
    if float(ab @ ab) < 1e-12:
        return float(np.linalg.norm(point - a))
    t = float(np.clip(((point - a) @ ab) / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(point - (a + t * ab)))


@dataclass
class AsepsisMonitor:
    """Latch contamination after either sterile interface becomes exposed."""

    contaminated: bool = False
    gripper_clearance: float = 0.007
    tray_clearance: float = 0.0005

    def observe(
        self,
        *,
        port_open: bool,
        spike_exposed: bool,
        spike_locked: bool,
        port_center,
        port_radius: float,
        spike_base,
        spike_tip,
        spike_radius: float,
        tcps,
        tray_top: float,
    ) -> None:
        if self.contaminated:
            return
        port_center = np.asarray(port_center, float)
        spike_base, spike_tip = np.asarray(spike_base, float), np.asarray(spike_tip, float)
        if port_open and not spike_locked:
            if port_center[2] - port_radius <= tray_top + self.tray_clearance:
                self.contaminated = True
            if any(np.linalg.norm(np.asarray(t, float) - port_center) <= self.gripper_clearance for t in tcps):
                self.contaminated = True
        if spike_exposed and not spike_locked:
            if min(spike_base[2], spike_tip[2]) - spike_radius <= tray_top + self.tray_clearance:
                self.contaminated = True
            if any(point_segment_distance(t, spike_base, spike_tip) <= self.gripper_clearance for t in tcps):
                self.contaminated = True
