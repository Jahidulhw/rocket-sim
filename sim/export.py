"""JSON export for the browser viewer.

The simulator's own steps are irregular (dt plus extra samples at events), and
there are ~18k of them in a typical flight. The viewer only needs smooth
playback, so we resample onto a uniform 60 Hz grid with linear interpolation,
plus one exact sample at landing. Columns are stored as parallel arrays
("structure of arrays"), which is compact JSON and easy to index in JS.
Phases are stored as small integers indexing meta.phases.

Coordinates in the file are the simulator's: x = east, y = north, z = up (m).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from .flight import PHASES, FlightResult

SCHEMA_VERSION = 1
SAMPLE_RATE_HZ = 60
_END_GUARD_S = 1e-3   # drop grid points closer than this to the landing sample

UNITS = {"time": "s", "position": "m", "velocity": "m/s", "mass": "kg", "angle": "deg",
         "frame": "x = east, y = north, z = up; origin at the launch pad"}


def _r(a, nd):
    return [round(float(x), nd) for x in a]


def resample_times(flight_time: float, rate_hz: float = SAMPLE_RATE_HZ) -> np.ndarray:
    n_grid = math.ceil((flight_time - _END_GUARD_S) * rate_hz)
    return np.append(np.arange(n_grid) / rate_hz, flight_time)


def flight_to_dict(result: FlightResult, label: str = "", description: str = "",
                   rate_hz: float = SAMPLE_RATE_HZ) -> dict:
    if not result.landed:
        raise ValueError("can only export a flight that landed")
    t = resample_times(result.flight_time_s, rate_hz)
    sim_idx = np.searchsorted(result.t, t, side="right") - 1
    phase_idx = [PHASES.index(result.phase[i]) for i in sim_idx]

    traj = {"t": _r(t, 6)}
    for k, name in enumerate("xyz"):
        traj[name] = _r(np.interp(t, result.t, result.position[:, k]), 4)
    for k, name in enumerate(("vx", "vy", "vz")):
        traj[name] = _r(np.interp(t, result.t, result.velocity[:, k]), 4)
    traj["phase"] = phase_idx

    events = sorted(result.events.values(), key=lambda e: e.t)
    motor_info = result.motor.info() if hasattr(result.motor, "info") else {"name": result.motor.name}
    return {
        "schema_version": SCHEMA_VERSION,
        "kind": "flight",
        "meta": {"label": label, "description": description, "units": UNITS,
                 "phases": PHASES, "sample_rate_hz": rate_hz,
                 "config": result.config.to_dict(), "motor": motor_info},
        "summary": _clean(result.summary()),
        "trajectory": traj,
        "events": [_clean(e.to_dict()) for e in events],
    }


def _clean(obj, nd: int = 6):
    """Round floats and convert numpy scalars so json output is small and portable."""
    if isinstance(obj, dict):
        return {k: _clean(v, nd) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_clean(v, nd) for v in obj]
    if isinstance(obj, (bool, np.bool_)):
        return bool(obj)
    if isinstance(obj, (float, np.floating)):
        return round(float(obj), nd)
    if isinstance(obj, np.integer):
        return int(obj)
    return obj


def write_json(data: dict, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, separators=(",", ":"), allow_nan=False)
    return path


def montecarlo_to_dict(mc_result, label: str = "", description: str = "") -> dict:
    """Dispersion dataset: all landing points, apogees, stats, ellipse, a few paths."""
    from .montecarlo import mc_config_dict

    nom = mc_result.nominal
    t = resample_times(nom.flight_time_s, 4.0)
    nominal_path = {"t": _r(t, 3), **{c: _r(np.interp(t, nom.t, nom.position[:, k]), 2)
                                       for k, c in enumerate("xyz")}}
    return {
        "schema_version": SCHEMA_VERSION,
        "kind": "montecarlo",
        "meta": {"label": label, "description": description, "units": UNITS,
                 "config": mc_result.base_config.to_dict(), "montecarlo": mc_config_dict(mc_result.mc)},
        "stats": _clean(mc_result.stats),
        "runs": {"landing_x": _r(mc_result.landing_xy[:, 0], 2),
                 "landing_y": _r(mc_result.landing_xy[:, 1], 2),
                 "apogee_m": _r(mc_result.apogee_m, 2),
                 "flight_time_s": _r(mc_result.flight_time_s, 2),
                 "deploy_timing": list(mc_result.deploy_timing)},
        "nominal": {"apogee_m": round(nom.apogee_m, 3),
                    "landing_m": _r(nom.landing_point[:2], 3), "path": nominal_path},
        "sample_paths": [{"run": p["run"], "t": _r(p["t"], 3),
                          **{c: _r(p[c], 2) for c in "xyz"}} for p in mc_result.sample_paths],
    }
