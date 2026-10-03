"""Milestone 5: Monte Carlo dispersion."""

import math

import numpy as np
import pytest

from sim.config import REPO_ROOT, FlightConfig, apply_overrides
from sim.flight import simulate
from sim.montecarlo import (Dispersion, MonteCarloConfig, ellipse_from_cov, perturbed_config,
                            run_monte_carlo, sample_perturbations, summarize)

DEFAULT_CONFIG = REPO_ROOT / "configs" / "default.json"
N_SMALL = 12


@pytest.fixture(scope="module")
def base():
    return FlightConfig.load(DEFAULT_CONFIG)


# ------------------------------------------------------- reproducibility ---

def test_same_seed_gives_identical_results(base):
    mc = MonteCarloConfig(n_runs=N_SMALL, seed=7)
    a, b = run_monte_carlo(base, mc), run_monte_carlo(base, mc)
    assert np.array_equal(a.landing_xy, b.landing_xy)
    assert np.array_equal(a.apogee_m, b.apogee_m)


def test_different_seed_gives_different_results(base):
    a = run_monte_carlo(base, MonteCarloConfig(n_runs=N_SMALL, seed=7))
    b = run_monte_carlo(base, MonteCarloConfig(n_runs=N_SMALL, seed=8))
    assert not np.array_equal(a.landing_xy, b.landing_xy)
    assert not np.array_equal(a.apogee_m, b.apogee_m)


def test_parallel_and_serial_runs_identical(base):
    mc = MonteCarloConfig(n_runs=6, seed=3)
    serial = run_monte_carlo(base, mc, workers=1)
    parallel = run_monte_carlo(base, mc, workers=2)
    assert np.array_equal(serial.landing_xy, parallel.landing_xy)


def test_zero_dispersion_every_run_equals_nominal(base):
    mc = MonteCarloConfig(n_runs=5, seed=1, dispersion=Dispersion.zero())
    res = run_monte_carlo(base, mc)
    nominal = simulate(apply_overrides(base, {"sim.dt_s": mc.dt_s}))
    for k in range(mc.n_runs):
        assert np.array_equal(res.landing_xy[k], nominal.landing_point[:2])
        assert res.apogee_m[k] == nominal.apogee_m
    assert res.stats["landing"]["std_m"] == pytest.approx([0.0, 0.0], abs=1e-9)


# ------------------------------------------------------- distributions -----

def test_perturbation_samples_have_documented_distributions():
    d = Dispersion()
    rng = np.random.default_rng(0)
    p = sample_perturbations(rng, d, 20_000)
    assert np.mean(p["impulse_scale"]) == pytest.approx(1.0, abs=0.002)
    assert np.std(p["impulse_scale"]) == pytest.approx(d.impulse_sigma_frac, rel=0.05)
    assert np.std(p["cd_scale"]) == pytest.approx(d.cd_sigma_frac, rel=0.05)
    assert np.std(p["dry_mass_scale"]) == pytest.approx(d.dry_mass_sigma_frac, rel=0.05)
    assert np.std(p["wind_dir_offset_deg"]) == pytest.approx(d.wind_dir_sigma_deg, rel=0.05)
    # Truncated at +-3 sigma: no negative Cd, no absurd motors.
    assert np.max(np.abs(p["impulse_scale"] - 1)) <= 3 * d.impulse_sigma_frac + 1e-12
    assert np.max(np.abs(p["rail_tilt_east_deg"])) <= 3 * d.launch_angle_sigma_deg + 1e-12


def test_perturbed_config_applies_each_dispersion(base):
    pert = {"wind_speed_offset_mps": 1.5, "wind_dir_offset_deg": 10.0, "impulse_scale": 1.02,
            "cd_scale": 0.9, "dry_mass_scale": 1.05, "rail_tilt_east_deg": 0.0, "rail_tilt_north_deg": 2.0}
    cfg = perturbed_config(base, pert)
    assert cfg.wind.speed_mps == pytest.approx(base.wind.speed_mps + 1.5)
    assert cfg.wind.toward_deg == pytest.approx(base.wind.toward_deg + 10.0)
    assert cfg.motor.impulse_scale == pytest.approx(1.02)
    assert cfg.rocket.cd == pytest.approx(base.rocket.cd * 0.9)
    assert cfg.rocket.dry_mass_kg == pytest.approx(base.rocket.dry_mass_kg * 1.05)
    # Vertical rail tilted 2 deg toward north.
    assert cfg.launch.tilt_deg == pytest.approx(2.0)
    assert cfg.launch.azimuth_deg == pytest.approx(0.0, abs=1e-9)
    # Wind speed never goes negative.
    assert perturbed_config(base, {**pert, "wind_speed_offset_mps": -50.0}).wind.speed_mps == 0.0


def test_impulse_scale_scales_total_impulse(base):
    from sim.motor import build_motor
    cfg = apply_overrides(base, {"motor.impulse_scale": 1.03})
    assert build_motor(cfg.motor).total_impulse == pytest.approx(1.03 * build_motor(base.motor).total_impulse)


# ---------------------------------------------------------- statistics -----

def test_statistics_on_known_synthetic_input():
    # Four points at (+-1, 0) and (0, +-2) around (10, 5).
    xy = np.array([[11.0, 5.0], [9.0, 5.0], [10.0, 7.0], [10.0, 3.0]])
    apo = np.array([100.0, 110.0, 120.0, 130.0])
    s = summarize(xy, apo)
    assert s["landing"]["mean_m"] == pytest.approx([10.0, 5.0])
    # Sample (ddof=1) variances: x: 2/3, y: 8/3
    assert s["landing"]["std_m"] == pytest.approx([math.sqrt(2 / 3), math.sqrt(8 / 3)])
    assert s["apogee"]["mean_m"] == pytest.approx(115.0)
    assert s["apogee"]["std_m"] == pytest.approx(np.std(apo, ddof=1))
    assert s["apogee"]["min_m"] == 100.0 and s["apogee"]["max_m"] == 130.0
    e = s["ellipse"]
    assert e["k_sigma"] == 2.0
    assert e["semi_major_m"] == pytest.approx(2 * math.sqrt(8 / 3))
    assert e["semi_minor_m"] == pytest.approx(2 * math.sqrt(2 / 3))
    assert abs(e["angle_deg"]) == pytest.approx(90.0)        # major axis along y (north)
    drift = np.hypot(xy[:, 0], xy[:, 1])
    assert s["drift"]["mean_m"] == pytest.approx(drift.mean())


def test_percentile_of_known_input():
    xy = np.column_stack([np.arange(101.0), np.zeros(101)])   # drift distances 0..100
    s = summarize(xy, np.ones(101))
    assert s["drift"]["p95_m"] == pytest.approx(95.0)
    assert s["drift"]["max_m"] == 100.0


def test_ellipse_orientation_for_rotated_covariance():
    th = math.radians(30.0)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    cov = R @ np.diag([9.0, 1.0]) @ R.T
    e = ellipse_from_cov(np.zeros(2), cov, k=2.0)
    assert e["semi_major_m"] == pytest.approx(6.0)
    assert e["semi_minor_m"] == pytest.approx(2.0)
    assert e["angle_deg"] == pytest.approx(30.0)


def test_two_sigma_ellipse_coverage_on_gaussian_samples():
    """For a 2D Gaussian, the 2-sigma ellipse holds 1 - exp(-2) = 86.5% of points."""
    rng = np.random.default_rng(42)
    xy = rng.multivariate_normal([50.0, -20.0], [[400.0, 150.0], [150.0, 100.0]], size=20_000)
    s = summarize(xy, np.zeros(len(xy)))
    assert s["ellipse"]["fraction_inside"] == pytest.approx(1 - math.exp(-2), abs=0.01)


# ------------------------------------------------------ accuracy tradeoff --

@pytest.mark.parametrize("seed", [11, 12, 13])
def test_monte_carlo_dt_matches_fine_dt(base, seed):
    """MC runs use a 10x larger dt. Verify it on dispersed configurations."""
    mc = MonteCarloConfig()
    pert = {k: v[0] for k, v in sample_perturbations(np.random.default_rng(seed), mc.dispersion, 1).items()}
    cfg = perturbed_config(base, pert)
    coarse = simulate(apply_overrides(cfg, {"sim.dt_s": mc.dt_s}))
    fine = simulate(apply_overrides(cfg, {"sim.dt_s": 0.002}))
    assert coarse.apogee_m == pytest.approx(fine.apogee_m, rel=1e-4)
    assert np.linalg.norm(coarse.landing_point - fine.landing_point) < 0.05   # metres


def test_monte_carlo_dt_safe_even_for_high_speed_deployment(base):
    """Chute opened at burnout (~100 m/s) is stiff; the step limiter must keep it accurate."""
    cfg = apply_overrides(base, {"recovery.deploy_delay_s": 0.0})
    coarse = simulate(apply_overrides(cfg, {"sim.dt_s": MonteCarloConfig().dt_s}))
    fine = simulate(apply_overrides(cfg, {"sim.dt_s": 0.001}))
    assert coarse.apogee_m == pytest.approx(fine.apogee_m, rel=1e-4)
    assert np.linalg.norm(coarse.landing_point - fine.landing_point) < 0.05


# --------------------------------------------------------------- export ----

def test_montecarlo_export_structure(base, tmp_path):
    import json

    from sim.export import montecarlo_to_dict, write_json
    mc = MonteCarloConfig(n_runs=8, seed=5, n_sample_paths=3)
    res = run_monte_carlo(base, mc)
    path = write_json(montecarlo_to_dict(res, label="t"), tmp_path / "mc.json")
    d = json.loads(path.read_text(encoding="utf-8"))
    assert d["kind"] == "montecarlo"
    assert len(d["runs"]["landing_x"]) == len(d["runs"]["apogee_m"]) == 8
    assert d["runs"]["landing_x"] == pytest.approx(list(res.landing_xy[:, 0]), abs=0.01)
    assert len(d["sample_paths"]) == 3
    for p in d["sample_paths"]:
        assert len(p["t"]) == len(p["x"]) == len(p["y"]) == len(p["z"])
        assert np.all(np.diff(p["t"]) > 0)
    assert d["stats"]["n_runs"] == 8
    assert d["meta"]["montecarlo"]["seed"] == 5
    assert d["nominal"]["apogee_m"] == pytest.approx(res.nominal.apogee_m, abs=1e-3)
