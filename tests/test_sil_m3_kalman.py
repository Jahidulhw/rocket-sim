"""SIL milestone 3: Kalman-filter apogee detection, closed loop, and the
baseline-vs-Kalman comparison on identical seeded flights."""

from __future__ import annotations

import numpy as np
import pytest

from sim.sil import SilConfig, run_sil


@pytest.fixture(scope="module")
def kalman_run(fc_exe, sil_base_cfg):
    return run_sil(sil_base_cfg, SilConfig(seed=1, fc_exe=str(fc_exe), fc_mode="kalman"))


def test_kalman_is_the_default_mode():
    assert SilConfig().fc_mode == "kalman"


def test_kalman_sil_flight_deploys_at_apogee(kalman_run):
    r = kalman_run
    assert r.fc_state_sequence() == ["PAD", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED"]
    assert r.deploy_mechanism == "fc" and r.fc_deploy_reason == "apogee"
    assert abs(r.flight.deployment["dt_from_apogee_s"]) < 0.15
    assert r.flight.landed and not r.fc_failed and r.protocol_errors == 0


def test_kalman_estimates_track_truth_in_coast(kalman_run):
    log, ev = kalman_run.log, kalman_run.flight.events
    coast = (log["t"] > ev["burnout"].t + 0.3) & (log["t"] < kalman_run.flight.deployment["true_apogee_t"])
    v_err = log["est_vel"][coast] - log["vz_true"][coast]
    h_err = log["est_alt"][coast] - log["z_true"][coast]
    assert np.sqrt(np.mean(v_err**2)) < 0.2       # m/s
    assert np.sqrt(np.mean(h_err**2)) < 0.3       # m (baro bias removed by the ground reference)


def test_kalman_velocity_is_near_zero_on_pad(kalman_run):
    # After the first 2 s of the 10 s pad sit (filter start-up while the
    # accelerometer reference is still being learned), the estimate must
    # sit near zero: it is what launch-time velocity starts from.
    log = kalman_run.log
    settled_pad = (log["t"] >= -8.0) & (log["t"] < 0.0)
    assert np.max(np.abs(log["est_vel"][settled_pad])) < 0.3


def test_baseline_vs_kalman_same_seeds(fc_exe, sil_base_cfg, record_property):
    """Both detectors fly the same seeded flights (identical sensor noise up to
    deployment). The Kalman detector must be closer to true apogee on every seed."""
    dts = {"baseline": [], "kalman": []}
    for seed in range(200, 208):
        for mode in dts:
            r = run_sil(sil_base_cfg, SilConfig(seed=seed, fc_exe=str(fc_exe), fc_mode=mode,
                                                pre_launch_s=3.0, post_landing_s=0.0))
            assert r.deploy_mechanism == "fc" and r.fc_deploy_reason == "apogee", (seed, mode)
            dts[mode].append(r.flight.deployment["dt_from_apogee_s"])
    b, k = np.array(dts["baseline"]), np.array(dts["kalman"])
    record_property("baseline_mean_dt_s", float(b.mean()))
    record_property("kalman_mean_dt_s", float(k.mean()))
    print(f"\nbaseline {b.mean():+.3f} +- {b.std(ddof=1):.3f} s   kalman {k.mean():+.3f} +- {k.std(ddof=1):.3f} s")
    assert np.all(np.abs(k) < np.abs(b))
    assert np.all(np.abs(k) < 0.15)


def test_mistuned_process_noise_biases_velocity_and_deploys_early(fc_exe, sil_base_cfg):
    """q far too small makes the constant-acceleration model too rigid to
    follow the changing drag: velocity is biased low near apogee and the
    zero crossing (deploy) comes EARLY. Documented mistuning failure mode."""
    r = run_sil(sil_base_cfg, SilConfig(seed=1, fc_exe=str(fc_exe), fc_mode="kalman",
                                        fc_args=("--param", "kf.jerk_psd=0.01"),
                                        pre_launch_s=3.0, post_landing_s=0.0))
    assert r.flight.deployment["dt_from_apogee_s"] < 0.0


def test_fc_rejects_unknown_param(fc_exe):
    import subprocess
    p = subprocess.run([str(fc_exe), "--param", "guidance_gain=1"], capture_output=True, timeout=10)
    assert p.returncode == 2 and b"invalid --param" in p.stderr
