"""Monte Carlo SIL campaign: dispersed flights, nominal or with one random fault,
flown by the C++ FC in both detector modes; plus REQ-002 pad sits.

Reproducibility: every random choice (dispersions, which fault, its timing
and size, the per-run sensor seed) is drawn up front from one seeded PCG64
stream in a fixed order, exactly like sim/montecarlo.py. A worker receives a
fully specified job, so results are identical however many processes run
them (tested), and any run can be replayed from its record alone.
"""

from __future__ import annotations

import hashlib
import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field

import numpy as np

from .config import FlightConfig
from .faults import Fault
from .montecarlo import PERTURBATION_KEYS, Dispersion, perturbed_config, sample_perturbations
from .requirements import evaluate, true_apogee_t
from .sil import SilConfig, run_pad_sit, run_sil, run_sil_preset

FAULT_CLASSES = ("dropout_blackout", "dropout_random", "stuck_baro", "stuck_accel", "spike_baro",
                 "spike_accel", "drift_baro", "drift_accel", "hang")
MODES = ("kalman", "baseline")


@dataclass
class SilMcConfig:
    n_runs: int = 500
    seed: int = 20261003
    fault_fraction: float = 0.6        # share of runs with one random fault (the rest nominal)
    modes: tuple = MODES               # every run is flown by each detector, same inputs
    dispersion: Dispersion = field(default_factory=Dispersion)
    pre_launch_s: float = 5.0
    post_landing_s: float = 12.0       # >= 10 s so REQ-010 is checkable
    # Fleet campaigns: fly this preset (its own FC config and sensor model).
    preset_id: str | None = None
    # Fault start times/durations were chosen for the C6's ~7.4 s climb; they
    # scale with the rocket's own apogee time so faults land in the same
    # flight phases on a 25 s high-power flight. 1.0 for the C6.
    fault_time_scale: float = 1.0
    # Wall-clock watchdog. 1.0 s proved too tight: under heavy machine load a
    # healthy FC once missed it (a false trip, found by replay). See run_job.
    watchdog_timeout_s: float = 3.0


def sample_fault(rng: np.random.Generator, cls: str, time_scale: float = 1.0) -> Fault:
    """One random fault of the given class (ranges are engineering choices,
    deliberately harsher than typical real sensor behaviour). Times scale
    with `time_scale` (rocket apogee time / C6 apogee time)."""
    k = time_scale

    def u(lo, hi):
        return rng.uniform(lo, hi)
    sign = 1.0 if rng.random() < 0.5 else -1.0
    if cls == "dropout_blackout":
        return Fault("dropout", k * u(0.0, 8.5), u(0.2, 1.5))
    if cls == "dropout_random":
        return Fault("dropout", k * u(0.0, 3.0), k * 15.0, probability=u(0.1, 0.5))
    if cls in ("stuck_baro", "stuck_accel"):
        t0 = u(-2.0, 8.0)          # one draw, as for the C6: pad-time starts unscaled, in-flight scaled
        return Fault("stuck", t0 * k if t0 > 0 else t0, sensor=cls.split("_")[1])
    if cls == "spike_baro":
        return Fault("spike", k * u(0.0, 4.0), k * u(2.0, 15.0), "baro", u(10.0, 100.0), u(0.02, 0.2))
    if cls == "spike_accel":
        return Fault("spike", k * u(0.0, 4.0), k * u(2.0, 15.0), "accel", u(20.0, 150.0), u(0.02, 0.2))
    if cls == "drift_baro":
        return Fault("drift", k * u(0.0, 4.0), sensor="baro", magnitude=sign * u(0.5, 3.0))
    if cls == "drift_accel":
        return Fault("drift", k * u(0.0, 4.0), sensor="accel", magnitude=sign * u(0.1, 1.0))
    if cls == "hang":
        t0 = u(-2.0, 10.0)
        return Fault("hang", t0 * k if t0 > 0 else t0)
    raise ValueError(cls)


def make_jobs(base: FlightConfig, mc: SilMcConfig) -> list[dict]:
    rng = np.random.default_rng(mc.seed)
    perts = sample_perturbations(rng, mc.dispersion, mc.n_runs)
    is_faulted = rng.random(mc.n_runs) < mc.fault_fraction
    classes = rng.integers(0, len(FAULT_CLASSES), mc.n_runs)
    sensor_seeds = rng.integers(0, 2**31 - 1, mc.n_runs)
    jobs = []
    base_dict = base.to_dict()
    for i in range(mc.n_runs):
        # Always draw the fault (even for nominal runs) so the stream stays aligned.
        f = sample_fault(rng, FAULT_CLASSES[classes[i]], mc.fault_time_scale)
        jobs.append({"run": i, "base": base_dict, "pert": {k: float(perts[k][i]) for k in PERTURBATION_KEYS},
                     "fault_class": FAULT_CLASSES[classes[i]] if is_faulted[i] else "nominal",
                     "fault": f.to_dict() if is_faulted[i] else None,
                     "sensor_seed": int(sensor_seeds[i]), "modes": list(mc.modes),
                     "pre_launch_s": mc.pre_launch_s, "post_landing_s": mc.post_landing_s,
                     "watchdog_timeout_s": mc.watchdog_timeout_s, "preset_id": mc.preset_id})
    return jobs


def replay_overrides(cfg: FlightConfig) -> dict:
    """--set overrides that reproduce a dispersed config with scripts/run_sil.py."""
    return {"wind.speed_mps": cfg.wind.speed_mps, "wind.toward_deg": cfg.wind.toward_deg,
            "motor.impulse_scale": cfg.motor.impulse_scale, "rocket.cd": cfg.rocket.cd,
            "rocket.dry_mass_kg": cfg.rocket.dry_mass_kg, "launch.tilt_deg": cfg.launch.tilt_deg,
            "launch.azimuth_deg": cfg.launch.azimuth_deg}


def log_digest(res) -> str:
    """Hash of the FC's observable output; equal digests <=> identical output (REQ-006)."""
    h = hashlib.sha256()
    for k in ("t", "baro", "accel", "est_alt", "est_vel", "fc_deploy"):
        h.update(np.ascontiguousarray(res.log[k]).tobytes())
    h.update("|".join(map(str, res.log["fc_state"])).encode())
    return h.hexdigest()[:16]


def sil_config_for(job: dict, mode: str) -> SilConfig:
    faults = (Fault.from_dict(job["fault"]),) if job["fault"] else ()
    return SilConfig(seed=job["sensor_seed"], fc_mode=mode, faults=faults, pre_launch_s=job["pre_launch_s"],
                     post_landing_s=job["post_landing_s"], watchdog_timeout_s=job["watchdog_timeout_s"])


def flight_config_for(job: dict) -> FlightConfig:
    return perturbed_config(FlightConfig.from_dict(job["base"]), job["pert"])


def _fly_sil(job: dict, mode: str):
    sil = sil_config_for(job, mode)
    if not job.get("preset_id"):
        return run_sil(flight_config_for(job), sil)
    from .fleet import load_fleet
    from .staging import BoosterSpec
    preset = {p.id: p for p in load_fleet()}[job["preset_id"]]
    booster = None
    if preset.booster is not None:     # same draws perturb the booster (same day, same rail)
        b = preset.booster
        booster = BoosterSpec(perturbed_config(b.flight, job["pert"]), b.sustainer_ignition_delay_s, b.stack_cd)
    return run_sil_preset(preset, sil, flight=flight_config_for(job), booster=booster)


def run_job(job: dict) -> dict:
    """Worker: fly one job in every mode. Top-level so it pickles."""
    cfg = flight_config_for(job)
    out = {"run": job["run"], "fault_class": job["fault_class"], "fault": job["fault"],
           "sensor_seed": job["sensor_seed"], "overrides": replay_overrides(cfg),
           "pre_launch_s": job["pre_launch_s"], "post_landing_s": job["post_landing_s"], "modes": {}}
    hang_injected = bool(job["fault"]) and job["fault"]["kind"] == "hang"
    for mode in job["modes"]:
        res = _fly_sil(job, mode)
        retried = None
        if res.fc_failed and not hang_injected:
            # The FC is deterministic: a genuine FC hang/crash reproduces
            # exactly on a re-run, a wall-clock artefact (host overload) does
            # not. Re-run once; keep the original reason on record and count
            # these in the report so nothing is hidden.
            retried = res.failure_reason
            res = _fly_sil(job, mode)
        ta = true_apogee_t(res)
        out["modes"][mode] = {
            "fc_deploy_t": res.fc_deploy_t,
            "dt_s": None if res.fc_deploy_t is None else res.fc_deploy_t - ta,
            "true_apogee_t": ta, "true_apogee_m": res.flight.apogee_m,
            "reason": res.fc_deploy_reason, "mechanism": res.deploy_mechanism,
            "deploy_speed_mps": res.flight.deployment.get("speed_mps"),
            "landed": res.flight.landed, "fc_failed": res.fc_failed, "failure_reason": res.failure_reason,
            "watchdog_retry": retried, "protocol_errors": res.protocol_errors,
            "health": [h.split("HEALTH ", 1)[1] for h in res.fc_health_events],
            "verdicts": {rid: [v.status, v.detail] for rid, v in evaluate(res).items()},
            "digest": log_digest(res)}
    return out


def _pool_map(fn, items, workers: int, progress=None):
    results = []
    if workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            for i, r in enumerate(ex.map(fn, items, chunksize=max(1, len(items) // (workers * 8)))):
                results.append(r)
                if progress:
                    progress(i + 1, len(items))
    else:
        for i, it in enumerate(items):
            results.append(fn(it))
            if progress:
                progress(i + 1, len(items))
    return results


def run_campaign(base: FlightConfig, mc: SilMcConfig, workers: int = 1, progress=None) -> list[dict]:
    return _pool_map(run_job, make_jobs(base, mc), workers, progress)


def _pad_job(args):
    seed, duration, preset_id = (*args, None)[:3]
    sil = SilConfig(seed=seed)
    if preset_id:
        from .fleet import load_fleet, sil_settings
        s = sil_settings({p.id: p for p in load_fleet()}[preset_id])
        sil = SilConfig(seed=seed, sensors=s["sensors"], fc_args=tuple(s["fc_args"]))
    out = run_pad_sit(duration, sil)
    return {"seed": seed, "false_launch": out["false_launch"], "deploy": out["deploy"],
            "health_events": out["health_events"], "protocol_errors": out["protocol_errors"]}


def run_pad_sits(n: int, seed: int, duration_s: float = 60.0, workers: int = 1, progress=None,
                 preset_id: str | None = None) -> list[dict]:
    seeds = np.random.default_rng([seed, 2]).integers(0, 2**31 - 1, n)
    return _pool_map(_pad_job, [(int(s), duration_s, preset_id) for s in seeds], workers, progress)


def replay_determinism(base: FlightConfig, mc: SilMcConfig, records: list[dict], runs: list[int],
                       mode: str = "kalman") -> list[dict]:
    """REQ-006: re-fly selected runs serially; their output digests must match."""
    jobs = {j["run"]: j for j in make_jobs(base, mc)}
    out = []
    for i in runs:
        job = dict(jobs[i], modes=[mode])
        again = run_job(job)["modes"][mode]["digest"]
        out.append({"run": i, "original": records[i]["modes"][mode]["digest"], "replay": again,
                    "identical": again == records[i]["modes"][mode]["digest"]})
    return out


# ---------------------------------------------------------------- summary --

def requirement_rates(records: list[dict], mode: str = "kalman") -> dict:
    """Per requirement: applicable runs, passes, failures (per-run checks only)."""
    rates = {}
    for r in records:
        for rid, (status, _detail) in r["modes"][mode]["verdicts"].items():
            d = rates.setdefault(rid, {"applicable": 0, "pass": 0, "fail": 0})
            if status != "n/a":
                d["applicable"] += 1
                d[status] += 1
    return rates


def timing_by_class(records: list[dict], mode: str) -> dict:
    out = {}
    for cls in ("nominal",) + FAULT_CLASSES:
        rows = [r["modes"][mode] for r in records if r["fault_class"] == cls]
        if not rows:
            continue
        dts = np.array([x["dt_s"] for x in rows if x["dt_s"] is not None])
        req_key = "REQ-001" if cls == "nominal" else ("REQ-005" if cls == "hang" else "REQ-004")
        app = [x["verdicts"][req_key][0] for x in rows if x["verdicts"][req_key][0] != "n/a"]
        out[cls] = {"n": len(rows), "fc_deployed": int(dts.size),
                    "dt_mean_s": float(dts.mean()) if dts.size else math.nan,
                    "dt_abs_p95_s": float(np.percentile(np.abs(dts), 95)) if dts.size else math.nan,
                    "dt_max_abs_s": float(np.max(np.abs(dts))) if dts.size else math.nan,
                    "timer_deploys": sum(1 for x in rows if x["reason"] == "backup_timer"),
                    "req": req_key, "req_pass": app.count("pass"), "req_applicable": len(app)}
    return out


def default_workers() -> int:
    return max(1, min(8, (os.cpu_count() or 2) - 1))


def preset_campaign_config(preset_id: str, n_runs: int, seed: int, fault_fraction: float = 0.6):
    """(base flight config, SilMcConfig) for a fleet preset: its SIL flight
    config, Kalman FC only (the detector comparison is the C6 campaign), and
    fault times scaled by its apogee time."""
    from .config import apply_overrides
    from .fleet import fly_preset, load_fleet, sil_flight_config_for
    preset = {p.id: p for p in load_fleet()}[preset_id]
    base = sil_flight_config_for(preset)
    t_ap = fly_preset(preset, flight=apply_overrides(base, {"recovery.enabled": False})).flight.events["apogee"].t
    mc = SilMcConfig(n_runs=n_runs, seed=seed, fault_fraction=fault_fraction, modes=("kalman",),
                     preset_id=preset_id, fault_time_scale=t_ap / 7.42)
    return base, mc
