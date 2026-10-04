"""SIL milestone 5: Monte Carlo campaign mechanics."""

from __future__ import annotations

import numpy as np
import pytest

from sim.sil_montecarlo import (FAULT_CLASSES, SilMcConfig, make_jobs, replay_determinism, requirement_rates,
                                run_campaign, sample_fault, timing_by_class)


def _small(**kw):
    base = dict(n_runs=6, seed=7, fault_fraction=0.5, pre_launch_s=2.0, post_landing_s=0.0,
                watchdog_timeout_s=0.5, modes=("kalman",))
    base.update(kw)
    return SilMcConfig(**base)


def test_jobs_are_reproducible_and_seed_dependent(sil_base_cfg):
    a = make_jobs(sil_base_cfg, SilMcConfig(n_runs=50, seed=1))
    b = make_jobs(sil_base_cfg, SilMcConfig(n_runs=50, seed=1))
    c = make_jobs(sil_base_cfg, SilMcConfig(n_runs=50, seed=2))
    assert a == b and a != c


def test_fault_mix_and_classes(sil_base_cfg):
    jobs = make_jobs(sil_base_cfg, SilMcConfig(n_runs=2000, seed=3, fault_fraction=0.6))
    faulted = [j for j in jobs if j["fault"] is not None]
    assert len(faulted) / len(jobs) == pytest.approx(0.6, abs=0.03)
    seen = {j["fault_class"] for j in faulted}
    assert seen == set(FAULT_CLASSES)
    assert all(j["fault_class"] == "nominal" for j in jobs if j["fault"] is None)


def test_sampled_faults_are_valid():
    rng = np.random.default_rng(0)
    for cls in FAULT_CLASSES:
        for _ in range(50):
            sample_fault(rng, cls)   # Fault.__post_init__ validates


@pytest.mark.req("REQ-006")
def test_parallel_results_identical_to_serial(fc_exe, sil_base_cfg):
    mc = _small()
    serial = run_campaign(sil_base_cfg, mc, workers=1)
    parallel = run_campaign(sil_base_cfg, mc, workers=3)
    assert serial == parallel     # every field, including the FC output digests


@pytest.mark.req("REQ-006")
def test_replayed_runs_match(fc_exe, sil_base_cfg):
    mc = _small(n_runs=3)
    records = run_campaign(sil_base_cfg, mc, workers=1)
    reps = replay_determinism(sil_base_cfg, mc, records, [0, 2])
    assert all(r["identical"] for r in reps)


def test_summaries_count_correctly():
    rec = lambda cls, status, dt, reason: {  # noqa: E731
        "fault_class": cls,
        "modes": {"kalman": {"dt_s": dt, "reason": reason,
                             "verdicts": {"REQ-001": [status if cls == "nominal" else "n/a", ""],
                                          "REQ-004": [status if cls != "nominal" else "n/a", ""],
                                          "REQ-005": ["n/a", ""]}}}}
    records = [rec("nominal", "pass", 0.05, "apogee"), rec("nominal", "fail", 0.7, "apogee"),
               rec("stuck_baro", "pass", 0.1, "apogee"), rec("drift_accel", "pass", 1.1, "backup_timer")]
    rates = requirement_rates(records)
    assert rates["REQ-001"] == {"applicable": 2, "pass": 1, "fail": 1}
    assert rates["REQ-004"] == {"applicable": 2, "pass": 2, "fail": 0}
    assert rates["REQ-005"]["applicable"] == 0
    t = timing_by_class(records, "kalman")
    assert t["nominal"]["n"] == 2 and t["nominal"]["dt_mean_s"] == pytest.approx(0.375)
    assert t["drift_accel"]["timer_deploys"] == 1
