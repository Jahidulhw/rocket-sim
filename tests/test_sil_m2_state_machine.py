"""SIL milestone 2: the C++ state machine flies closed-loop SIL flights
(baseline raw-barometer apogee detection)."""

from __future__ import annotations

import numpy as np
import pytest

from sim.sil import SilConfig, run_sil

FC_ORDER = ["PAD", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED"]


@pytest.fixture(scope="module")
def nominal_run(fc_exe, sil_base_cfg):
    return run_sil(sil_base_cfg, SilConfig(seed=1, fc_exe=str(fc_exe), fc_mode="baseline"))


def test_full_sil_flight_visits_every_state_in_order(nominal_run):
    assert nominal_run.fc_state_sequence() == FC_ORDER
    assert not nominal_run.fc_failed and nominal_run.protocol_errors == 0
    assert nominal_run.fc_returncode == 0


def test_fc_deploys_at_apogee_and_rocket_lands(nominal_run):
    r = nominal_run
    assert r.flight.landed
    assert r.deploy_mechanism == "fc"            # FC beat the C6-7 backup charge
    assert r.fc_deploy_reason == "apogee"
    assert r.flight.events["deploy"].t == pytest.approx(r.fc_deploy_t, abs=1e-12)
    dt = r.flight.deployment["dt_from_apogee_s"]
    assert 0.0 < dt < 1.2                        # baseline: after apogee, not by much


@pytest.mark.req("REQ-010")
def test_fc_events_track_truth(nominal_run):
    r, ev = nominal_run, nominal_run.flight.events
    st = r.fc_state_times()
    assert 0.0 < st["BOOST"] - ev["liftoff"].t < 0.1
    assert 0.0 < st["COAST"] - ev["burnout"].t < 0.1
    assert 4.0 < st["LANDED"] - ev["landing"].t < 10.0


@pytest.mark.req("REQ-003")
def test_no_deploy_command_in_pad_or_boost(nominal_run):
    log = nominal_run.log
    locked = np.isin(log["fc_state"], ["PAD", "BOOST"])
    assert locked.any()
    assert not log["fc_deploy"][locked].any()


def test_deploy_command_is_latched(nominal_run):
    log = nominal_run.log
    i0 = np.flatnonzero(log["fc_deploy"])[0]
    assert log["fc_deploy"][i0:].all()


def test_reported_altitude_is_ground_relative(nominal_run):
    log = nominal_run.log
    pad = log["t"] < 0.0
    # Baro bias (0.5 m) removed by the ground reference: single-sample error
    # is just noise + quantization.
    err = log["est_alt"][pad][200:] - log["z_true"][pad][200:]
    assert abs(err.mean()) < 0.1
    assert err.std() < 0.6


def test_baseline_deploy_timing_under_nominal_noise(fc_exe, sil_base_cfg, record_property):
    """Records how late the BASELINE detector deploys relative to true apogee."""
    dts = []
    for seed in range(10):
        r = run_sil(sil_base_cfg, SilConfig(seed=100 + seed, fc_exe=str(fc_exe), fc_mode="baseline",
                                            pre_launch_s=3.0, post_landing_s=0.0))
        assert r.deploy_mechanism == "fc" and r.fc_deploy_reason == "apogee", seed
        dts.append(r.flight.deployment["dt_from_apogee_s"])
    dts = np.array(dts)
    record_property("baseline_dt_mean_s", float(dts.mean()))
    record_property("baseline_dt_max_s", float(dts.max()))
    print(f"\nbaseline deploy - true apogee over 10 seeds: mean {dts.mean():+.3f} s, "
          f"std {dts.std(ddof=1):.3f} s, min {dts.min():+.3f} s, max {dts.max():+.3f} s")
    assert np.all(dts > 0.0)      # raw-baro detection is always late...
    assert np.all(dts < 1.2)      # ...but well before the motor backup (~+1.4 s)


@pytest.mark.req("REQ-009")
def test_backup_timer_does_not_preempt_nominal_detection(fc_exe, sil_base_cfg):
    """The 8.5 s timer is a backup: on a nominal flight the detector must win.
    (The timer path itself is covered by C++ unit tests, and end-to-end under
    a stuck barometer by the milestone 4 fault tests.)"""
    r = run_sil(sil_base_cfg, SilConfig(seed=3, fc_exe=str(fc_exe), pre_launch_s=2.0, post_landing_s=0.0))
    st = r.fc_state_times()
    assert r.fc_deploy_reason == "apogee"
    assert st["APOGEE"] < st["BOOST"] + 8.5 - 0.5
