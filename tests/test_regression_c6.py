"""Regression pin: the original default C6 rocket must fly exactly as before Phase A.

Baseline recorded by scripts/make_regression_baseline.py on fleet-phase-a at
main f28c3db, BEFORE any fleet change (tests/data/c6_baseline.json).

Tolerances:
  * physics: 1e-9 relative. The same machine reproduces it bit-for-bit; the
    slack only absorbs last-bit libm differences between platforms (CI Linux
    vs Windows), never a model change.
  * flight-computer event times: at most one 10 ms tick. A last-bit physics
    difference can move a sensor value across a %.3f rounding boundary on
    another platform; anything beyond one tick is a real change. When the
    deploy tick is identical, the SIL landing point must match to 1e-6.
  * requirement verdicts, deploy reason and mechanism: exact.
"""

from __future__ import annotations

import json

import pytest

from sim.config import REPO_ROOT, FlightConfig
from sim.faults import Fault
from sim.flight import simulate
from sim.montecarlo import MonteCarloConfig, run_monte_carlo
from sim.requirements import evaluate
from sim.sil import SilConfig, run_sil, sil_flight_config

BASE = json.loads((REPO_ROOT / "tests" / "data" / "c6_baseline.json").read_text(encoding="utf-8"))
REL = 1e-9
TICK = 0.01 + 1e-9


def _cfg():
    return FlightConfig.load(REPO_ROOT / "configs" / "default.json")


def _check_flight(res, ref):
    assert res.apogee_m == pytest.approx(ref["apogee_m"], rel=REL)
    assert res.events["apogee"].t == pytest.approx(ref["apogee_t"], rel=REL)
    assert res.flight_time_s == pytest.approx(ref["flight_time_s"], rel=REL)
    assert res.max_speed_mps == pytest.approx(ref["max_speed_mps"], rel=REL)
    assert res.landing_point[0] == pytest.approx(ref["landing_xy"][0], rel=REL)
    assert res.landing_point[1] == pytest.approx(ref["landing_xy"][1], abs=1e-6)
    assert res.events["burnout"].t == pytest.approx(ref["burnout_t"], rel=REL)
    d = res.deployment
    assert d["t"] == pytest.approx(ref["deploy_t"], rel=REL)
    assert d["speed_mps"] == pytest.approx(ref["deploy_speed_mps"], rel=REL)
    assert d["timing"] == ref["timing"]
    import numpy as np
    for t, z in ref["z_at"].items():
        assert float(np.interp(float(t), res.t, res.position[:, 2])) == pytest.approx(z, rel=REL, abs=1e-9)


def test_default_c6_open_loop_flight_unchanged():
    _check_flight(simulate(_cfg()), BASE["open_loop"])


def test_sil_c6_7_open_loop_flight_unchanged():
    _check_flight(simulate(sil_flight_config(_cfg())), BASE["sil_open_loop"])


def test_default_c6_monte_carlo_unchanged():
    mc = run_monte_carlo(_cfg(), MonteCarloConfig(n_runs=20, seed=7), workers=1)
    ref = BASE["montecarlo_20"]
    assert mc.stats["apogee"]["mean_m"] == pytest.approx(ref["apogee_mean_m"], rel=REL)
    assert mc.stats["landing"]["mean_m"] == pytest.approx(ref["landing_mean_m"], rel=REL)
    assert mc.stats["drift"]["p95_m"] == pytest.approx(ref["drift_p95_m"], rel=REL)


SIL_CASES = {
    "kalman_seed1": dict(seed=1, fc_mode="kalman"),
    "baseline_seed1": dict(seed=1, fc_mode="baseline"),
    "kalman_stuck_baro_seed2": dict(seed=2, fc_mode="kalman", faults=(Fault("stuck", 3.0, sensor="baro"),)),
}


@pytest.mark.req("REQ-001", "REQ-003", "REQ-004")
@pytest.mark.parametrize("name", list(SIL_CASES))
def test_default_c6_sil_results_unchanged(fc_exe, name):
    ref = BASE["sil"][name]
    res = run_sil(sil_flight_config(_cfg()), SilConfig(fc_exe=str(fc_exe), **SIL_CASES[name]))
    verdicts = {rid: v.status for rid, v in evaluate(res).items()}
    # The requirements that existed when the baseline was recorded: unchanged.
    assert {rid: verdicts[rid] for rid in ref["verdicts"]} == ref["verdicts"]
    # Requirements added later (two-burn, dual deploy) do not apply to this rocket.
    assert all(verdicts[rid] == "n/a" for rid in set(verdicts) - set(ref["verdicts"]))
    assert res.fc_deploy_reason == ref["reason"] and res.deploy_mechanism == ref["mechanism"]
    assert [h.split("HEALTH ", 1)[1] for h in res.fc_health_events] == ref["health"]
    assert res.fc_deploy_t == pytest.approx(ref["fc_deploy_t"], abs=TICK)
    st = res.fc_state_times()
    assert set(st) == set(ref["state_times"])
    for s, t in ref["state_times"].items():
        assert st[s] == pytest.approx(t, abs=TICK), s
    assert res.flight.apogee_m == pytest.approx(ref["apogee_m"], rel=REL)
    if res.fc_deploy_t == ref["fc_deploy_t"]:
        assert res.flight.landing_point[0] == pytest.approx(ref["landing_xy"][0], rel=1e-6)
        assert res.flight.landing_point[1] == pytest.approx(ref["landing_xy"][1], rel=1e-6, abs=1e-6)
