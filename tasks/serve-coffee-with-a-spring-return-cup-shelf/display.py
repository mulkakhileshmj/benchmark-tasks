"""The coffee machine's display state (stream, liquid level, steam, spill tiles and the lamp), drawn with Isaac Lab's
VisualizationMarkers so the cameras and the video show it. Display only: the markers have no collision and no physics, they never
feed the scorer, and any exception turns them off with one warning while the episode carries on.

The markers are created once at reset and only MOVED afterwards; unused ones are parked 10 m under the floor. Switching a marker's
visibility on and off mid-episode made the renderer wash the whole frame grey, so this module never changes visibility.
Outside the simulator (unit tests), nothing is drawn; :func:`display_plan` is pure and is what the tests check."""

from __future__ import annotations

import math
import warnings
from typing import Any, Mapping

import numpy as np

from geometry import (
    BASE_TOP_Z,
    BREW_LEVEL_MAX,
    COLOR_LAMP_AMBER,
    COLOR_LAMP_GREEN,
    COLOR_LIQUID,
    COLOR_STEAM,
    COLOR_STREAM,
    LAMP_SIZE,
    LAMP_XYZ,
    LIQUID_RADIUS_M,
    LIQUID_THICK_M,
    MUG_FLOOR_M,
    MUG_HEIGHT_M,
    NOZZLE_TIP_Z,
    NOZZLE_XY,
    PLATE_TOP_Z,
    STEAM_PUFFS,
    STEAM_STEPS,
    STREAM_RADIUS_M,
    TABLE_TOP,
    TILE_COUNT,
    TILE_GRID,
    TILE_SIZE,
)
from display_state import CRACKED, GOOD, stream_on

PARK_Z = -10.0
LAYERS = ("stream", "level", "steam", "spill", "lamp_green", "lamp_amber")
COUNTS = {"stream": 1, "level": 2, "steam": STEAM_PUFFS, "spill": TILE_COUNT, "lamp_green": 1, "lamp_amber": 1}
_Row = tuple  # ((x, y, z) world metres relative to the env origin, (sx, sy, sz)) or None for a parked marker


def _surface_z(ledger: Mapping, mugs: Mapping) -> float:
    """Height (above the bench) where the stream ends: the liquid in the mug under it, else the plate or the base."""
    target = ledger["target"]
    if target is not None and target in mugs:
        mug_z = float(mugs[target][0][2]) - TABLE_TOP
        return mug_z + MUG_FLOOR_M + float(ledger["level"][target]) * (MUG_HEIGHT_M - MUG_FLOOR_M)
    return PLATE_TOP_Z if ledger.get("shelf_home", True) else BASE_TOP_Z


def display_plan(ledger: Mapping, mugs: Mapping, shelf_home: bool) -> dict[str, list]:
    """What each marker layer shows now: a list with one row per marker, (position, scale) or None when parked.

    mugs maps a mug name to (pos, quat_xyzw) in the env frame; shelf_home says whether the shelf is at rest (the stream then
    lands on the plate, otherwise on the base)."""
    plan: dict[str, list] = {k: [None] * COUNTS[k] for k in LAYERS}
    step = int(ledger["step"])
    brewing = stream_on(ledger) or ledger["target"] is not None
    if brewing:
        led = dict(ledger)
        led["shelf_home"] = shelf_home
        z_bot = _surface_z(led, mugs) + TABLE_TOP
        z_top = TABLE_TOP + NOZZLE_TIP_Z
        length = max(0.005, z_top - z_bot)
        plan["stream"][0] = ((NOZZLE_XY[0], NOZZLE_XY[1], z_bot + length / 2), (1.0, 1.0, length))
    for i, name in enumerate((GOOD, CRACKED)):
        lvl = float(ledger["level"].get(name, 0.0))
        if lvl > 0.0 and name in mugs:
            mx, my, mz = (float(v) for v in mugs[name][0])
            z = mz + MUG_FLOOR_M + min(lvl, BREW_LEVEL_MAX) * (MUG_HEIGHT_M - MUG_FLOOR_M) - LIQUID_THICK_M / 2
            plan["level"][i] = ((mx, my, z), (1.0, 1.0, 1.0))
    mug = ledger.get("steam_mug")
    if mug in mugs and step < ledger["steam_until"]:
        age = STEAM_STEPS - (ledger["steam_until"] - step)
        fade = max(0.3, 1.0 - 0.7 * age / STEAM_STEPS)
        mx, my, mz = (float(v) for v in mugs[mug][0])
        for i in range(STEAM_PUFFS):
            frac = i / (STEAM_PUFFS - 1)
            rise = (frac + 0.01 * step) % 1.0 if i % 2 else frac
            wobble = 0.006 * math.sin(0.15 * step + 1.7 * i)
            plan["steam"][i] = (
                (mx + wobble, my - wobble, mz + MUG_HEIGHT_M + 0.03 + 0.12 * rise),
                (fade * (0.6 + 0.9 * rise),) * 3,
            )
    for k, surface in enumerate(ledger["tiles"][:TILE_COUNT]):
        dx, dy = TILE_GRID[k]
        z = TABLE_TOP + (PLATE_TOP_Z if surface == "plate" else BASE_TOP_Z) + TILE_SIZE[2] / 2
        plan["spill"][k] = ((NOZZLE_XY[0] + dx, NOZZLE_XY[1] + dy, z), (1.0, 1.0, 1.0))
    lamp = ((LAMP_XYZ[0], LAMP_XYZ[1], TABLE_TOP + LAMP_XYZ[2]), (1.0, 1.0, 1.0))
    plan["lamp_amber" if brewing else "lamp_green"][0] = lamp
    return plan


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


class _Layer:
    def __init__(self, name: str) -> None:
        self.name = name
        self.markers: Any = None
        self.last: Any = None

    def make(self) -> None:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        def material(colour, opacity=None):
            kwargs = {"opacity": opacity} if opacity is not None else {}
            return sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(_linear(c) for c in colour), roughness=0.6, **kwargs)

        if self.name == "stream":
            proto = sim_utils.CylinderCfg(radius=STREAM_RADIUS_M, height=1.0, axis="Z", visual_material=material(COLOR_STREAM))
        elif self.name == "level":
            proto = sim_utils.CylinderCfg(radius=LIQUID_RADIUS_M, height=LIQUID_THICK_M, axis="Z", visual_material=material(COLOR_LIQUID))
        elif self.name == "steam":
            proto = sim_utils.SphereCfg(radius=0.02, visual_material=material(COLOR_STEAM, 0.6))
        elif self.name == "spill":
            proto = sim_utils.CuboidCfg(size=TILE_SIZE, visual_material=material(COLOR_LIQUID))
        elif self.name == "lamp_green":
            proto = sim_utils.CuboidCfg(size=LAMP_SIZE, visual_material=material(COLOR_LAMP_GREEN))
        else:
            proto = sim_utils.CuboidCfg(size=LAMP_SIZE, visual_material=material(COLOR_LAMP_AMBER))
        cfg = VisualizationMarkersCfg(prim_path=f"/Visuals/cm_{self.name}", markers={"m": proto})
        self.markers = VisualizationMarkers(cfg)

    def apply(self, rows: list, origin: np.ndarray) -> None:
        key = tuple(None if r is None else (tuple(round(v, 4) for v in r[0]), tuple(round(v, 3) for v in r[1])) for r in rows)
        if key == self.last:
            return
        n = len(rows)
        pts = np.tile([0.0, 0.0, PARK_Z], (n, 1)).astype(np.float64)
        scales = np.full((n, 3), 0.01, dtype=np.float64)
        for i, r in enumerate(rows):
            if r is not None:
                pts[i] = np.asarray(r[0], dtype=float) + origin
                scales[i] = np.asarray(r[1], dtype=float)
        quats = np.tile([0.0, 0.0, 0.0, 1.0], (n, 1))
        self.markers.visualize(translations=pts.astype(np.float32), orientations=quats.astype(np.float32), scales=scales.astype(np.float32))
        self.last = key


class _Display:
    def __init__(self) -> None:
        self._layers = {k: _Layer(k) for k in LAYERS}
        self._off = False

    def reset(self, environment, ledger: Mapping) -> None:
        """Create every marker (once) and show the idle state: all parked except the green lamp."""
        if self._off:
            return
        try:
            origin = _origin(environment)
            for layer in self._layers.values():
                if layer.markers is None:
                    layer.make()
                layer.last = None
            self._draw(display_plan(ledger, {}, True), origin)
        except ImportError:
            self._off = True  # no simulator: unit tests
        except Exception as error:  # cosmetic only
            self._disable(error)

    def update(self, environment, ledger: Mapping, mugs: Mapping, shelf_home: bool) -> None:
        if self._off or self._layers["stream"].markers is None:
            return
        try:
            self._draw(display_plan(ledger, mugs, shelf_home), _origin(environment))
        except ImportError:
            self._off = True
        except Exception as error:  # cosmetic only
            self._disable(error)

    def _draw(self, plan: dict, origin: np.ndarray) -> None:
        for name, layer in self._layers.items():
            layer.apply(plan[name], origin)

    def _disable(self, error: Exception) -> None:
        self._off = True
        warnings.warn(f"coffee display markers turned off ({type(error).__name__}: {error}); scoring is unaffected")


def _origin(environment) -> np.ndarray:
    import parts

    if environment is None:
        return np.zeros(3)
    return parts._to_numpy(environment.unwrapped.scene.env_origins[0])


DISPLAY = _Display()
