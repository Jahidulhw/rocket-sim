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
import re
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


# ------------------------------------------------------------------ SIL --

SIL_RATE_HZ = 50   # FC log is 100 Hz; every 2nd tick is plenty for the chart


def _num_or_none(v, nd):
    v = float(v)
    return round(v, nd) if math.isfinite(v) else None


def sil_flight_to_dict(sil_result, fc_mode: str, label: str = "", description: str = "",
                       rate_hz: float = SAMPLE_RATE_HZ, sil_rate_hz: float = SIL_RATE_HZ) -> dict:
    """Flight dataset plus a "sil" section: FC state per sample, estimated vs
    true altitude, delivered barometer, fault intervals, FC deploy decision
    (time, reason, mechanism) and FC health events. The viewer's timeline
    starts at ignition, so pre-launch pad frames are omitted. Ticks without
    an FC reply (dropped frame, dead FC) have fc_state -1 and null estimates."""
    from .protocol import FC_STATES
    from .requirements import true_apogee_t

    res = sil_result
    d = flight_to_dict(res.flight, label=label, description=description, rate_hz=rate_hz)
    T = res.flight.flight_time_s
    log = res.log
    keep = np.flatnonzero((log["t"] >= -1e-9) & (log["t"] <= T + 1e-9))
    keep = keep[:: max(1, round(100 / sil_rate_hz))]
    state_idx = [FC_STATES.index(s) if s else -1 for s in log["fc_state"][keep]]
    series = {"t": _r(log["t"][keep], 3),
              "true_alt": _r(log["z_true"][keep], 2),
              "est_alt": [_num_or_none(v, 2) for v in log["est_alt"][keep]],
              "est_vel": [_num_or_none(v, 2) for v in log["est_vel"][keep]],
              "true_vz": _r(log["vz_true"][keep], 2),
              "baro": [_num_or_none(v, 2) for v in log["baro"][keep]],
              "fc_state": state_idx,
              "sent": [int(bool(v)) for v in log["sent"][keep]]}
    transitions, prev = [], None
    for t, s in zip(log["t"], log["fc_state"]):
        if s and s != prev and t >= -1e-9:
            transitions.append({"t": round(float(t), 3), "state": str(s)})
        if s:
            prev = s
    faults = []
    for f in res.faults:
        start = max(0.0, f.start_s)
        end = T if (f.kind == "hang" or math.isinf(f.duration_s)) else min(T, f.start_s + f.duration_s)
        if end > start:
            faults.append({"label": f.label, "kind": f.kind, "start": round(start, 3), "end": round(end, 3),
                           "spec": f.to_dict()})
    health = []
    for line in res.fc_health_events:
        m = re.search(r"t=(-?[\d.]+) HEALTH (.*)", line)
        if m:
            health.append({"t": float(m.group(1)), "text": m.group(2)})
    ta = true_apogee_t(res)
    d["sil"] = {
        "fc_mode": fc_mode, "fc_states": list(FC_STATES), "series": series, "transitions": transitions,
        "faults": faults, "health": health,
        "deploy": {"fc_t": res.fc_deploy_t, "reason": res.fc_deploy_reason, "mechanism": res.deploy_mechanism,
                   "true_apogee_t": round(ta, 4),
                   "dt_s": None if res.fc_deploy_t is None else round(res.fc_deploy_t - ta, 4)},
        "fc_failed": {"failed": res.fc_failed, "t": res.failure_t, "reason": res.failure_reason},
        "protocol_errors": res.protocol_errors,
    }
    return _clean(d)


# ---------------------------------------------------------------- fleet --

LONG_FLIGHT_S = 120.0     # longer flights are exported at reduced rates (site size)


def fleet_flight_to_dict(sil_result, preset, label: str = "", description: str = "") -> dict:
    """SIL flight of a fleet preset for the viewer: the SIL dataset plus the
    rocket's geometry (procedural 3D model) and, for two-stage rockets, the
    spent booster's own track. Flights longer than 2 minutes (the high-power
    rockets, ~7 min under parachute) are exported at 20 Hz trajectory / 25 Hz
    FC series instead of 60 / 50 Hz: playback is still smooth and the
    datasets stay ~1 MB."""
    long = sil_result.flight.flight_time_s > LONG_FLIGHT_S
    d = sil_flight_to_dict(sil_result, "kalman", label=label, description=description,
                           rate_hz=20.0 if long else SAMPLE_RATE_HZ, sil_rate_hz=25.0 if long else SIL_RATE_HZ)
    d["rocket"] = {"id": preset.id, "category": preset.category, "two_stage": preset.two_stage,
                   "geometry": preset.geometry,
                   "body_diameter_m": preset.flight.rocket.body_diameter_m}
    b = sil_result.booster
    if b is not None:
        t = resample_times(b.flight_time_s, 20.0 if long else SAMPLE_RATE_HZ)
        d["booster"] = {"t": _r(t, 3), **{c: _r(np.interp(t, b.t, b.position[:, k]), 2) for k, c in enumerate("xyz")},
                        **{v: _r(np.interp(t, b.t, b.velocity[:, k]), 2) for k, v in enumerate(("vx", "vy", "vz"))},
                        "events": [_clean(e.to_dict()) for e in sorted(b.events.values(), key=lambda e: e.t)]}
    return _clean(d)
