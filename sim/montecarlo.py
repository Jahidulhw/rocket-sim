"""Monte Carlo dispersion analysis.

Each run perturbs the nominal configuration with independent random draws.
All draws for all runs are generated up front from one seeded PCG64 stream
(in a fixed order), so results are identical for the same seed regardless of
how many worker processes execute the runs.

Distributions (every one is a normal distribution truncated at +-3 sigma by
clipping, so no draw is physically absurd, e.g. a negative Cd):

    quantity            perturbation                        default sigma
    ------------------  ----------------------------------  -------------
    wind speed          nominal + N(0, s), then >= 0        1.0 m/s
    wind direction      nominal + N(0, s)                   20 deg
    motor total impulse thrust curve x (1 + N(0, s))        3 %
    body Cd             nominal x (1 + N(0, s))             5 %
    dry mass            nominal x (1 + N(0, s))             2 %
    rail pointing       two independent small rotations,    1 deg each
                        toward east and toward north

The defaults are engineering judgement for a hobby launch, not measured data:
~3 % is a typical spread in certified hobby-motor impulse, and a hand-set
launch rod is easily 1 deg off its intended angle.
"""

from __future__ import annotations

import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, field

import numpy as np

from .config import FlightConfig
from .export import resample_times
from .flight import simulate

PERTURBATION_KEYS = ["wind_speed_offset_mps", "wind_dir_offset_deg", "impulse_scale", "cd_scale",
                     "dry_mass_scale", "rail_tilt_east_deg", "rail_tilt_north_deg"]


@dataclass
class Dispersion:
    wind_speed_sigma_mps: float = 1.0
    wind_dir_sigma_deg: float = 20.0
    impulse_sigma_frac: float = 0.03
    cd_sigma_frac: float = 0.05
    dry_mass_sigma_frac: float = 0.02
    launch_angle_sigma_deg: float = 1.0
    truncate_sigma: float = 3.0

    @classmethod
    def zero(cls) -> "Dispersion":
        return cls(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass
class MonteCarloConfig:
    n_runs: int = 500
    seed: int = 20261003
    dt_s: float = 0.05          # 10x the single-flight dt; accuracy verified in tests
    dispersion: Dispersion = field(default_factory=Dispersion)
    n_sample_paths: int = 25    # trajectories kept for the viewer
    path_rate_hz: float = 4.0


@dataclass
class MonteCarloResult:
    base_config: FlightConfig
    mc: MonteCarloConfig
    perturbations: dict          # key -> (N,) array
    landing_xy: np.ndarray       # (N, 2)
    apogee_m: np.ndarray         # (N,)
    flight_time_s: np.ndarray    # (N,)
    deploy_timing: list          # (N,) "before" / "near" / "after" / "none"
    sample_paths: list           # [{"run": k, "t": [...], "x": [...], "y": [...], "z": [...]}]
    nominal: object              # FlightResult of the undispersed config at mc.dt_s
    stats: dict


def _normal(rng: np.random.Generator, sigma: float, n: int, trunc: float) -> np.ndarray:
    z = np.clip(rng.standard_normal(n), -trunc, trunc)
    return sigma * z


def sample_perturbations(rng: np.random.Generator, d: Dispersion, n: int) -> dict:
    """Draw all perturbations for n runs (fixed draw order => reproducible)."""
    tr = d.truncate_sigma
    return {
        "wind_speed_offset_mps": _normal(rng, d.wind_speed_sigma_mps, n, tr),
        "wind_dir_offset_deg": _normal(rng, d.wind_dir_sigma_deg, n, tr),
        "impulse_scale": 1.0 + _normal(rng, d.impulse_sigma_frac, n, tr),
        "cd_scale": 1.0 + _normal(rng, d.cd_sigma_frac, n, tr),
        "dry_mass_scale": 1.0 + _normal(rng, d.dry_mass_sigma_frac, n, tr),
        "rail_tilt_east_deg": _normal(rng, d.launch_angle_sigma_deg, n, tr),
        "rail_tilt_north_deg": _normal(rng, d.launch_angle_sigma_deg, n, tr),
    }


def _tilted_rail(tilt_deg: float, azimuth_deg: float, east_deg: float, north_deg: float):
    """Rotate the rail toward east (about the north axis), then toward north
    (about the east axis); return the new (tilt, azimuth)."""
    t, a = math.radians(tilt_deg), math.radians(azimuth_deg)
    x, y, z = math.sin(t) * math.sin(a), math.sin(t) * math.cos(a), math.cos(t)
    e, n = math.radians(east_deg), math.radians(north_deg)
    x, z = x * math.cos(e) + z * math.sin(e), -x * math.sin(e) + z * math.cos(e)
    y, z = y * math.cos(n) + z * math.sin(n), -y * math.sin(n) + z * math.cos(n)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, z))))
    az = math.degrees(math.atan2(x, y)) % 360.0 if tilt > 0.0 else azimuth_deg
    return tilt, az


def perturbed_config(base: FlightConfig, p: dict) -> FlightConfig:
    """Apply one run's perturbations to a copy of the nominal config."""
    cfg = base.copy()
    cfg.wind.speed_mps = max(0.0, base.wind.speed_mps + float(p["wind_speed_offset_mps"]))
    cfg.wind.toward_deg = base.wind.toward_deg + float(p["wind_dir_offset_deg"])
    cfg.motor.impulse_scale = base.motor.impulse_scale * float(p["impulse_scale"])
    cfg.rocket.cd = base.rocket.cd * float(p["cd_scale"])
    cfg.rocket.dry_mass_kg = base.rocket.dry_mass_kg * float(p["dry_mass_scale"])
    de, dn = float(p["rail_tilt_east_deg"]), float(p["rail_tilt_north_deg"])
    if de != 0.0 or dn != 0.0:  # zero error leaves the rail bit-for-bit unchanged
        cfg.launch.tilt_deg, cfg.launch.azimuth_deg = _tilted_rail(
            base.launch.tilt_deg, base.launch.azimuth_deg, de, dn)
    cfg.validate()
    return cfg


def _run_one(args):
    """Worker: simulate one dispersed flight. Top-level so it can be pickled."""
    base_dict, pert, dt, keep_path, path_rate = args
    cfg = perturbed_config(FlightConfig.from_dict(base_dict), pert)
    cfg.sim.dt_s = dt
    res = simulate(cfg)
    out = {"landing_xy": res.landing_point[:2].copy(), "apogee_m": res.apogee_m,
           "flight_time_s": res.flight_time_s, "landed": res.landed,
           "deploy_timing": res.deployment.get("timing", "none")}
    if keep_path:
        t = resample_times(res.flight_time_s, path_rate)  # uniform grid + exact landing
        out["path"] = {"t": t.tolist(),
                       **{c: np.interp(t, res.t, res.position[:, k]).tolist() for k, c in enumerate("xyz")}}
    return out


def run_monte_carlo(base: FlightConfig, mc: MonteCarloConfig | None = None, workers: int = 1,
                    progress=None) -> MonteCarloResult:
    mc = mc or MonteCarloConfig()
    rng = np.random.default_rng(mc.seed)
    perts = sample_perturbations(rng, mc.dispersion, mc.n_runs)
    base_dict = base.to_dict()
    jobs = [(base_dict, {k: perts[k][i] for k in PERTURBATION_KEYS}, mc.dt_s,
             i < mc.n_sample_paths, mc.path_rate_hz) for i in range(mc.n_runs)]

    results = []
    if workers > 1:
        chunk = max(1, mc.n_runs // (workers * 4))
        with ProcessPoolExecutor(max_workers=workers) as ex:
            for i, r in enumerate(ex.map(_run_one, jobs, chunksize=chunk)):
                results.append(r)
                if progress:
                    progress(i + 1, mc.n_runs)
    else:
        for i, job in enumerate(jobs):
            results.append(_run_one(job))
            if progress:
                progress(i + 1, mc.n_runs)

    if not all(r["landed"] for r in results):
        raise RuntimeError("some Monte Carlo runs did not land within sim.max_time_s")
    landing = np.array([r["landing_xy"] for r in results])
    apogee = np.array([r["apogee_m"] for r in results])
    nominal_cfg = base.copy()
    nominal_cfg.sim.dt_s = mc.dt_s
    return MonteCarloResult(
        base_config=base, mc=mc, perturbations=perts, landing_xy=landing, apogee_m=apogee,
        flight_time_s=np.array([r["flight_time_s"] for r in results]),
        deploy_timing=[r["deploy_timing"] for r in results],
        sample_paths=[{"run": i, **r["path"]} for i, r in enumerate(results) if "path" in r],
        nominal=simulate(nominal_cfg), stats=summarize(landing, apogee))


# ------------------------------------------------------------- statistics --

def ellipse_from_cov(center, cov, k: float = 2.0) -> dict:
    """k-sigma ellipse of a 2D covariance.

    Semi-axes are k * sqrt(eigenvalues); angle is the major axis direction in
    degrees counter-clockwise from +x (east), normalised to (-90, 90].
    """
    cov = np.asarray(cov, dtype=float)
    vals, vecs = np.linalg.eigh(cov)                     # ascending eigenvalues
    major = vecs[:, 1]
    angle = math.degrees(math.atan2(major[1], major[0]))
    if angle <= -90.0:
        angle += 180.0
    elif angle > 90.0:
        angle -= 180.0
    return {"center_m": [float(c) for c in center], "k_sigma": float(k),
            "semi_major_m": float(k * math.sqrt(max(vals[1], 0.0))),
            "semi_minor_m": float(k * math.sqrt(max(vals[0], 0.0))),
            "angle_deg": float(angle)}


def summarize(landing_xy, apogee_m, pad=(0.0, 0.0), k_sigma: float = 2.0) -> dict:
    xy = np.asarray(landing_xy, dtype=float)
    apo = np.asarray(apogee_m, dtype=float)
    n = len(xy)
    ddof = 1 if n > 1 else 0          # sample (unbiased) statistics
    mean = xy.mean(axis=0)
    cov = np.cov(xy.T, ddof=ddof) if n > 1 else np.zeros((2, 2))
    drift = np.hypot(xy[:, 0] - pad[0], xy[:, 1] - pad[1])

    ell = ellipse_from_cov(mean, cov, k_sigma)
    d = xy - mean
    m2 = np.einsum("ij,jk,ik->i", d, np.linalg.pinv(cov), d)   # squared Mahalanobis distance
    ell["fraction_inside"] = float(np.mean(m2 <= k_sigma**2 + 1e-12))
    ell["expected_fraction_gaussian"] = 1.0 - math.exp(-k_sigma**2 / 2.0)

    return {
        "n_runs": n,
        "landing": {"mean_m": mean.tolist(), "std_m": xy.std(axis=0, ddof=ddof).tolist(),
                    "cov_m2": cov.tolist()},
        "drift": {"mean_m": float(drift.mean()), "std_m": float(drift.std(ddof=ddof)),
                  "p95_m": float(np.percentile(drift, 95)), "max_m": float(drift.max())},
        "apogee": {"mean_m": float(apo.mean()), "std_m": float(apo.std(ddof=ddof)),
                   "min_m": float(apo.min()), "max_m": float(apo.max()),
                   "p5_m": float(np.percentile(apo, 5)), "p95_m": float(np.percentile(apo, 95))},
        "ellipse": ell,
    }


def default_workers() -> int:
    return max(1, min(8, (os.cpu_count() or 2) - 1))


def mc_config_dict(mc: MonteCarloConfig) -> dict:
    return asdict(mc)
