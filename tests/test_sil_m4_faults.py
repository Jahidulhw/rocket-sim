"""SIL milestone 4: fault injection, FC fault tolerance, requirement checks.

Every test that verifies a requirement carries @pytest.mark.req(...)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from sim.faults import Fault, FaultInjector
from sim.requirements import evaluate, fault_class, logs_identical
from sim.sil import SilConfig, run_pad_sit, run_sil

# ------------------------------------------------------- fault injector --


def _inj(*faults, seed=0):
    return FaultInjector(faults, np.random.default_rng(seed))


def test_fault_spec_parsing():
    f = Fault.parse("spike:2:6:accel:80:0.1")
    assert (f.kind, f.start_s, f.duration_s, f.sensor, f.magnitude, f.probability) == \
        ("spike", 2.0, 6.0, "accel", 80.0, 0.1)
    assert Fault.parse("stuck:3:inf:baro").duration_s == math.inf
    assert Fault.parse("hang:4").kind == "hang"
    for bad in ["nope:1", "spike:1:2:baro", "stuck:1:2:gyro", "dropout:1:0", "x"]:
        with pytest.raises(ValueError):
            Fault.parse(bad)


def test_fault_round_trips_through_dict():
    for f in [Fault("stuck", 3.0, sensor="accel"), Fault("spike", 1.0, 5.0, "baro", 40.0, 0.2)]:
        assert Fault.from_dict(f.to_dict()) == f


def test_faults_only_act_inside_their_window():
    inj = _inj(Fault("drift", 2.0, 1.0, "baro", 10.0))
    assert inj.apply(1.99, 5.0, 9.8) == (True, 5.0, 9.8, [])
    send, b, a, active = inj.apply(2.5, 5.0, 9.8)
    assert b == pytest.approx(10.0) and a == 9.8 and active == ["drift_baro"]
    assert inj.apply(3.0, 5.0, 9.8) == (True, 5.0, 9.8, [])


def test_stuck_holds_first_value_in_window():
    inj = _inj(Fault("stuck", 1.0, sensor="accel"))
    vals = [inj.apply(1.0 + 0.01 * k, 0.0, 50.0 - k)[2] for k in range(20)]
    assert vals == [50.0] * 20


def test_dropout_probability_and_blackout():
    inj = _inj(Fault("dropout", 0.0, 1000.0, probability=0.3), seed=4)   # window covers all 200 s sampled
    sent = [inj.apply(0.01 * k, 0.0, 0.0)[0] for k in range(20000)]
    assert 1.0 - np.mean(sent) == pytest.approx(0.3, abs=0.015)
    blackout = _inj(Fault("dropout", 1.0, 0.5))
    assert [blackout.apply(t, 0, 0)[0] for t in (0.99, 1.0, 1.49, 1.5)] == [True, False, False, True]


def test_spike_magnitude_sign_and_rate():
    inj = _inj(Fault("spike", 0.0, 1000.0, "baro", 50.0, 0.1), seed=2)
    deltas = np.array([inj.apply(0.01 * k, 100.0, 0.0)[1] - 100.0 for k in range(20000)])
    hit = deltas != 0.0
    assert hit.mean() == pytest.approx(0.1, abs=0.01)
    assert set(np.abs(deltas[hit])) == {50.0}
    assert (deltas[hit] > 0).mean() == pytest.approx(0.5, abs=0.05)


def test_hang_is_reported_from_start_time():
    inj = _inj(Fault("hang", 4.0))
    assert inj.hang_at_s == 4.0
    assert inj.apply(3.99, 0, 0)[3] == [] and inj.apply(4.0, 0, 0)[3] == ["hang"]


def test_faults_do_not_shift_sensor_noise(fc_exe, sil_base_cfg):
    """Outside its window, a faulted run's sensor stream equals the nominal one."""
    base = SilConfig(seed=9, fc_exe=str(fc_exe), pre_launch_s=1.0, post_landing_s=0.0)
    nom = run_sil(sil_base_cfg, base)
    flt = run_sil(sil_base_cfg, SilConfig(**{**base.__dict__, "faults": (Fault("spike", 3.0, 1.0, "baro", 40.0, 0.5),)}))
    before = nom.log["t"] < 3.0
    n = int(before.sum())
    assert np.array_equal(nom.log["baro"][:n], flt.log["baro"][:n])
    assert np.array_equal(nom.log["accel"][:n], flt.log["accel"][:n])


# ---------------------------------------------- fault classes, closed loop --

FAULT_CASES = {
    "dropout_blackout_over_apogee": Fault("dropout", 6.5, 1.5),
    "dropout_30pct": Fault("dropout", 0.0, 15.0, probability=0.3),
    "stuck_baro_coast": Fault("stuck", 3.0, sensor="baro"),
    "stuck_baro_boost": Fault("stuck", 0.5, sensor="baro"),
    "stuck_accel_boost": Fault("stuck", 0.5, sensor="accel"),
    "stuck_accel_coast": Fault("stuck", 5.0, sensor="accel"),
    "spike_baro": Fault("spike", 0.0, 20.0, "baro", 50.0, 0.1),
    "spike_accel": Fault("spike", 0.0, 20.0, "accel", 80.0, 0.1),
    "drift_baro_up": Fault("drift", 0.0, sensor="baro", magnitude=2.0),
    "drift_baro_down": Fault("drift", 0.0, sensor="baro", magnitude=-2.0),
    "drift_accel_up": Fault("drift", 0.0, sensor="accel", magnitude=0.5),
    "drift_accel_down": Fault("drift", 0.0, sensor="accel", magnitude=-0.5),
}


@pytest.fixture(scope="module")
def fault_runs(fc_exe, sil_base_cfg):
    return {name: run_sil(sil_base_cfg, SilConfig(seed=21, fc_exe=str(fc_exe), faults=(f,),
                                                  pre_launch_s=3.0, post_landing_s=0.0))
            for name, f in FAULT_CASES.items()}


@pytest.mark.req("REQ-003", "REQ-004")
@pytest.mark.parametrize("name", list(FAULT_CASES))
def test_single_fault_meets_requirements(fault_runs, name):
    v = evaluate(fault_runs[name])
    assert v["REQ-003"].passed, v["REQ-003"].detail
    assert v["REQ-004"].passed, v["REQ-004"].detail
    assert fault_runs[name].flight.landed


@pytest.mark.req("REQ-004")
def test_stuck_baro_is_isolated_and_accel_only_filter_finds_apogee(fault_runs):
    r = fault_runs["stuck_baro_coast"]
    assert any("baro FAILED" in h for h in r.fc_health_events)
    assert r.fc_deploy_reason == "apogee"
    assert abs(r.fc_deploy_t - r.flight.deployment["true_apogee_t"]) < 0.3


@pytest.mark.req("REQ-004")
def test_stuck_accel_is_isolated_and_baseline_takes_over(fault_runs):
    r = fault_runs["stuck_accel_boost"]
    assert any("accel FAILED" in h for h in r.fc_health_events)
    assert r.fc_deploy_reason == "apogee"
    assert 0.0 < r.fc_deploy_t - r.flight.deployment["true_apogee_t"] < 1.2


@pytest.mark.req("REQ-004")
@pytest.mark.parametrize("name", ["spike_baro", "spike_accel"])
def test_spikes_are_gated_out(fault_runs, name):
    r = fault_runs[name]
    assert r.fc_health_events == []          # outliers, not failures
    assert abs(r.fc_deploy_t - r.flight.deployment["true_apogee_t"]) < 0.15


@pytest.mark.req("REQ-004")
def test_unisolated_disagreement_falls_back_to_backup_timer(fault_runs):
    r = fault_runs["drift_accel_up"]
    assert any("INCONSISTENT" in h for h in r.fc_health_events)
    assert r.fc_deploy_reason == "backup_timer"


def test_baseline_detector_is_fooled_by_baro_spikes(fc_exe, sil_base_cfg):
    """Why the default FC gates its inputs: the raw-barometer baseline deploys
    seconds early under the same spike fault (it would fail REQ-004)."""
    r = run_sil(sil_base_cfg, SilConfig(seed=21, fc_exe=str(fc_exe), fc_mode="baseline",
                                        faults=(FAULT_CASES["spike_baro"],), pre_launch_s=3.0, post_landing_s=0.0))
    assert r.fc_deploy_t - r.flight.deployment["true_apogee_t"] < -1.5   # early, outside the REQ-004 window
    assert not evaluate(r)["REQ-004"].passed


# ------------------------------------------------------------ FC hang --


@pytest.mark.req("REQ-005", "REQ-003")
@pytest.mark.parametrize("t_hang", [-2.0, 1.0, 4.0, 9.5])
def test_fc_hang_detected_and_chute_still_deploys(fc_exe, sil_base_cfg, t_hang):
    r = run_sil(sil_base_cfg, SilConfig(seed=22, fc_exe=str(fc_exe), faults=(Fault("hang", t_hang),),
                                        watchdog_timeout_s=0.5, pre_launch_s=3.0, post_landing_s=0.0))
    v = evaluate(r)
    assert v["REQ-005"].passed, v["REQ-005"].detail
    assert v["REQ-003"].passed
    if t_hang < 7.0:
        assert r.deploy_mechanism == "motor"      # FC died before apogee: backup charge
    else:
        assert r.deploy_mechanism == "fc"         # FC had already deployed


# --------------------------------------------- pad, determinism, checks --


@pytest.mark.req("REQ-002")
def test_no_false_launch_on_pad_through_real_fc(fc_exe):
    # The 1000-sit campaign runs in scripts/run_sil_montecarlo.py and in the
    # C++ unit test NoFalseLaunchIn1000SixtySecondPadSits_REQ002; this checks
    # the real process with the real Python sensor model.
    for seed in range(5):
        out = run_pad_sit(60.0, SilConfig(seed=1000 + seed, fc_exe=str(fc_exe)))
        assert not out["false_launch"] and not out["deploy"], seed
        assert out["health_events"] == [] and out["protocol_errors"] == 0


@pytest.mark.req("REQ-006")
def test_faulted_runs_are_deterministic(fc_exe, sil_base_cfg):
    sil = SilConfig(seed=23, fc_exe=str(fc_exe), pre_launch_s=1.0, post_landing_s=0.0,
                    faults=(Fault("spike", 0.0, 20.0, "accel", 80.0, 0.2),))
    assert logs_identical(run_sil(sil_base_cfg, sil), run_sil(sil_base_cfg, sil))


@pytest.mark.req("REQ-001", "REQ-003", "REQ-009", "REQ-010")
def test_nominal_run_passes_all_applicable_requirements(fc_exe, sil_base_cfg):
    r = run_sil(sil_base_cfg, SilConfig(seed=24, fc_exe=str(fc_exe)))
    v = evaluate(r)
    assert fault_class(r) == "nominal"
    for rid in ("REQ-001", "REQ-003", "REQ-009", "REQ-010"):
        assert v[rid].passed, (rid, v[rid].detail)
    assert v["REQ-004"].status == "n/a" and v["REQ-005"].status == "n/a"


def test_requirement_checks_detect_violations(fc_exe, sil_base_cfg):
    """The checks themselves must be able to fail: a deliberately broken FC
    configuration (backup timer far too short) violates REQ-009, and a
    mistuned filter that deploys early is caught by REQ-001."""
    r = run_sil(sil_base_cfg, SilConfig(seed=25, fc_exe=str(fc_exe), fc_args=("--param", "backup_timer_s=4.0"),
                                        pre_launch_s=3.0, post_landing_s=0.0))
    v = evaluate(r)
    assert v["REQ-009"].status == "fail" and v["REQ-001"].status == "fail"
    assert v["REQ-003"].passed   # still never in boost
