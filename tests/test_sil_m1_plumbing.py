"""SIL milestone 1: sensor models, line protocol, FC process plumbing, watchdog,
and the closed-loop hook in simulate()."""

from __future__ import annotations

import subprocess
import sys
import time

import numpy as np
import pytest

from sim.flight import simulate
from sim.physics import G0
from sim.protocol import (FC_STATES, ProtocolError, format_sensor_frame, parse_reply,
                          time_matches)
from sim.sensors import SensorConfig, SensorSuite
from sim.sil import FcFailure, FcProcess, SilConfig, SilLink, run_sil

N_STAT = 200_000


# ---------------------------------------------------------------- sensors --

def _samples(cfg: SensorConfig, z: float, az: float, n: int = N_STAT, seed: int = 1):
    s = SensorSuite(cfg, np.random.default_rng(seed))
    out = np.array([s.sample(z, az) for _ in range(n)])
    return out[:, 0], out[:, 1]


def test_baro_noise_statistics_match_config():
    cfg = SensorConfig()
    baro, _ = _samples(cfg, z=150.0, az=0.0)
    # Rounding to q adds uniform error of variance q^2/12, independent of the noise.
    expected_std = np.sqrt(cfg.baro_noise_sigma_m**2 + cfg.baro_quant_m**2 / 12.0)
    assert baro.mean() == pytest.approx(150.0 + cfg.baro_bias_m, abs=5 * expected_std / np.sqrt(N_STAT))
    assert baro.std(ddof=1) == pytest.approx(expected_std, rel=0.01)


def test_baro_is_quantized():
    cfg = SensorConfig()
    baro, _ = _samples(cfg, z=37.123, az=0.0, n=2000)
    steps = baro / cfg.baro_quant_m
    assert np.max(np.abs(steps - np.round(steps))) < 1e-9


def test_accel_noise_statistics_match_config_on_pad():
    cfg = SensorConfig()
    _, acc = _samples(cfg, z=0.0, az=0.0)
    sigma = cfg.accel_noise_sigma_mps2
    assert acc.mean() == pytest.approx(G0 + cfg.accel_bias_mps2, abs=5 * sigma / np.sqrt(N_STAT))
    assert acc.std(ddof=1) == pytest.approx(sigma, rel=0.01)


@pytest.mark.parametrize("az, expected", [
    (0.0, G0),             # at rest on the pad: reads the gravity reaction, +g
    (50.0, 50.0 + G0),     # boost
    (-G0, 0.0),            # free fall: reads zero
    (-12.0, G0 - 12.0),    # coasting up with drag: negative specific force
])
def test_accel_sign_convention(az, expected):
    s = SensorSuite(SensorConfig.ideal(), np.random.default_rng(0))
    assert s.sample(10.0, az)[1] == pytest.approx(expected, abs=1e-12)


def test_ideal_baro_reads_truth():
    s = SensorSuite(SensorConfig.ideal(), np.random.default_rng(0))
    assert s.sample(123.456, 0.0)[0] == pytest.approx(123.456, abs=1e-12)


def test_accel_saturates_at_full_scale():
    cfg = SensorConfig(accel_noise_sigma_mps2=0.0, accel_bias_mps2=0.0)
    s = SensorSuite(cfg, np.random.default_rng(0))
    assert s.sample(0.0, 400.0)[1] == pytest.approx(cfg.accel_range_mps2)
    assert s.sample(0.0, -400.0)[1] == pytest.approx(-cfg.accel_range_mps2)


def test_sensors_are_seed_deterministic():
    a = _samples(SensorConfig(), 10.0, 0.0, n=500, seed=7)
    b = _samples(SensorConfig(), 10.0, 0.0, n=500, seed=7)
    c = _samples(SensorConfig(), 10.0, 0.0, n=500, seed=8)
    assert np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])
    assert not np.array_equal(a[0], c[0])


def test_each_sample_consumes_exactly_two_draws():
    # Fault injection relies on this: the noise stream stays aligned whatever
    # happens to individual frames.
    rng = np.random.default_rng(3)
    s = SensorSuite(SensorConfig(), rng)
    for _ in range(10):
        s.sample(0.0, 0.0)
    ref = np.random.default_rng(3)
    ref.standard_normal(20)
    assert rng.standard_normal() == ref.standard_normal()


# ----------------------------------------------------------- protocol (py) --

def test_format_sensor_frame_exact():
    assert format_sensor_frame(1.25, 312.4, -9.75) == "S 1.250000 312.400 -9.7500"
    assert format_sensor_frame(-0.01, 0.5, 10.01) == "S -0.010000 0.500 10.0100"


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_format_sensor_frame_rejects_non_finite(bad):
    with pytest.raises(ValueError):
        format_sensor_frame(0.0, bad, 0.0)


def test_parse_reply_valid_and_crlf():
    r = parse_reply("R 7.420000 APOGEE 313.512 -0.041 1\r\n")
    assert (r.t, r.state, r.est_alt_m, r.est_vel_mps, r.deploy) == (7.42, "APOGEE", 313.512, -0.041, True)
    for st in FC_STATES:
        assert parse_reply(f"R 0.000000 {st} 0.000 0.000 0").state == st


@pytest.mark.parametrize("bad", [
    "",
    "R 1.0 PAD 0 0",              # too few fields
    "R 1.0 PAD 0 0 0 0",          # too many fields
    "R 1.0 FOO 0 0 0",            # unknown state
    "R 1.0 pad 0 0 0",            # case sensitive
    "R 1.0 PAD 0 0 4",            # deploy is a 2-bit mask: 0-3 only (1 drogue, 2 main)
    "R 1.0 PAD 0 0 9",
    "R 1.0 PAD 0 0 yes",
    "R nan PAD 0 0 0",
    "R 1.0 PAD inf 0 0",
    "R +1.0 PAD 0 0 0",
    "R 1.0  PAD 0 0 0",           # double space
    "S 1.0 0 0",                  # our own frame echoed back
    "E bad_number",               # FC rejected our input
    "R 1.0 PAD 0 0 1" + " " * 300,
])
@pytest.mark.req("REQ-007")
def test_parse_reply_rejects_malformed(bad):
    with pytest.raises(ProtocolError):
        parse_reply(bad)


def test_parse_reply_deploy_bitmask():
    r = parse_reply("R 1.000000 DESCENT 290.000 -20.000 3")
    assert r.deploy and r.deploy_main and r.command == 3
    r = parse_reply("R 1.000000 DESCENT 290.000 -20.000 2")
    assert not r.deploy and r.deploy_main and r.command == 2
    assert parse_reply("R 1.000000 COAST 100.000 5.000 0").command == 0


def test_time_matches_tolerance():
    assert time_matches(0.01, 0.010000)
    assert time_matches(1.0 / 3.0, 0.333333)      # %.6f rounding of the sent stamp
    assert not time_matches(0.01, 0.02)


# ------------------------------------------------------- FC process (C++) --

@pytest.mark.req("REQ-008")
def test_protocol_round_trip_with_fc_process(fc_exe):
    # Stub mode: echoes the barometer, so the round trip is checkable value by value.
    with FcProcess(fc_exe, ["--mode", "stub"], timeout_s=5.0) as fc:
        for k in range(200):
            t, baro = k * 0.01, 0.1 * k
            r = parse_reply(fc.exchange(format_sensor_frame(t, baro, G0)))
            assert time_matches(t, r.t)
            assert r.state == "PAD" and r.deploy is False
            assert r.est_alt_m == pytest.approx(baro, abs=5e-4)
        fc.close()
    assert fc.returncode == 0


@pytest.mark.req("REQ-008")
def test_fc_rejects_malformed_lines_and_keeps_running(fc_exe):
    with FcProcess(fc_exe, timeout_s=5.0) as fc:
        assert parse_reply(fc.exchange("S 0.000000 0.0 9.81")).state == "PAD"
        for bad in ["garbage", "S 1 2", "S 1 2 x", "S nan 0 0", "S 0.000000 0 9.81",  # last: time not increasing
                    "S 1 2 3 4", ""]:
            reply = fc.exchange(bad)
            assert reply.startswith("E "), (bad, reply)
            with pytest.raises(ProtocolError):
                parse_reply(reply)
        # One reply per line was preserved: the next good frame is answered in sync.
        r = parse_reply(fc.exchange("S 0.010000 1.0 9.81"))
        assert time_matches(0.01, r.t)
        fc.close()
    assert fc.returncode == 0


@pytest.mark.req("REQ-005")
def test_watchdog_fires_on_hung_fc(fc_exe):
    timeout = 0.5
    fc = FcProcess(fc_exe, ["--inject-hang-at", "0.05"], timeout_s=timeout)
    try:
        for k in range(5):
            parse_reply(fc.exchange(format_sensor_frame(k * 0.01, 0.0, G0)))
        t0 = time.perf_counter()
        with pytest.raises(FcFailure) as info:
            fc.exchange(format_sensor_frame(0.05, 0.0, G0))
        elapsed = time.perf_counter() - t0
    finally:
        fc.close(grace_s=0.5)  # the hung process ignores END and must be killed
    assert info.value.kind == "watchdog_timeout"
    assert timeout * 0.9 <= elapsed < timeout + 2.0
    assert fc.returncode not in (None, 0)
    assert any("injected hang" in s for s in fc.stderr_lines)


@pytest.mark.req("REQ-005")
def test_watchdog_detects_exited_fc(fc_exe):
    fc = FcProcess(fc_exe, timeout_s=5.0)
    try:
        with pytest.raises(FcFailure) as info:
            fc.exchange("END")          # FC exits; no reply will ever come
    finally:
        fc.close()
    assert info.value.kind == "exited"
    assert fc.returncode == 0


def test_fc_rejects_unknown_cli_argument(fc_exe):
    r = subprocess.run([str(fc_exe), "--steer"], capture_output=True, timeout=10)
    assert r.returncode == 2
    assert b"unknown argument" in r.stderr


# Fake FC (Python) that answers every frame with a different kind of bad reply.
_BAD_FC = r"""
import sys
bad = ["R {t} PAD 0.0 0.0 1 extra", "R 999.000000 PAD 0.0 0.0 1", "garbage 1", "E bad_number",
       "R {t} FOO 0.0 0.0 1", "R {t} DESCENT nan 0.0 1"]
for k, line in enumerate(sys.stdin):
    if line.strip() == "END":
        break
    t = line.split()[1]
    sys.stdout.write(bad[k % len(bad)].format(t=t) + "\n")
    sys.stdout.flush()
"""


@pytest.mark.req("REQ-007")
def test_sim_treats_malformed_fc_replies_as_no_command():
    """A garbage reply must never fire the chute, however many '1's it contains."""
    fc = FcProcess(sys.executable, ["-c", _BAD_FC], timeout_s=10.0)
    link = SilLink(fc, SensorSuite(SensorConfig(), np.random.default_rng(0)))
    try:
        commands = [link.exchange(k * 0.01, 0.0, 0.0, 0.0) for k in range(12)]
    finally:
        fc.close()
    assert commands == [False] * 12
    assert link.protocol_errors == 12
    assert not link.failed
    assert link.fc_deploy_t is None


# ------------------------------------------------ closed-loop simulate hook --

class _ScriptedController:
    """Pure-Python stand-in for the FC: records ticks, deploys at a set time."""

    def __init__(self, deploy_at: float | None, period: float = 0.01):
        self.period_s = period
        self.deploy_at = deploy_at
        self.t, self.az, self.z = [], [], []

    def tick(self, t, y, accel):
        self.t.append(t)
        self.z.append(y[2])
        self.az.append(accel[2])
        return self.deploy_at is not None and t >= self.deploy_at - 1e-9


def test_controller_deploy_is_applied_at_the_tick(sil_base_cfg):
    ctl = _ScriptedController(deploy_at=3.0)
    res = simulate(sil_base_cfg, controller=ctl)
    assert res.landed
    assert res.deployment["mechanism"] == "fc"
    assert res.events["deploy"].t == pytest.approx(3.0, abs=1e-12)
    # Deployed while climbing: timing is judged against the chute-free true apogee.
    assert res.deployment["timing"] == "before"


def test_controller_ticks_on_exact_uniform_grid(sil_base_cfg):
    ctl = _ScriptedController(deploy_at=None)
    res = simulate(sil_base_cfg, controller=ctl)
    t = np.array(ctl.t)
    k = np.arange(len(t))
    assert np.max(np.abs(t - k * 0.01)) < 1e-9             # no drift, no skipped tick
    assert t[-1] <= res.flight_time_s < t[-1] + 0.01 + 1e-9  # ticked right up to landing


def test_controller_sees_true_acceleration(sil_base_cfg):
    ctl = _ScriptedController(deploy_at=None)
    res = simulate(sil_base_cfg, controller=ctl)
    t, az = np.array(ctl.t), np.array(ctl.az)
    assert az[0] == 0.0                                      # held on the pad
    assert np.max(az) > 150.0                                # boost
    i_ap = np.argmin(np.abs(t - res.events["apogee"].t))
    assert az[i_ap] == pytest.approx(-G0, abs=0.2)           # ~free fall at apogee (vz ~ 0, little drag)
    coast = (t > res.events["burnout"].t + 0.05) & (t < res.events["apogee"].t - 0.5)
    assert np.all(az[coast] < -G0)                           # drag adds to gravity while climbing


def test_controller_that_never_deploys_matches_open_loop(sil_base_cfg):
    """Stepping onto tick instants must not change the physics."""
    open_loop = simulate(sil_base_cfg)
    closed = simulate(sil_base_cfg, controller=_ScriptedController(deploy_at=None))
    assert closed.deployment["mechanism"] == open_loop.deployment["mechanism"] == "motor"
    assert closed.apogee_m == pytest.approx(open_loop.apogee_m, rel=1e-7)
    assert closed.events["deploy"].t == pytest.approx(open_loop.events["deploy"].t, abs=1e-12)
    assert closed.landing_point == pytest.approx(open_loop.landing_point, abs=1e-4)


def test_c6_7_backup_fires_after_true_apogee(sil_base_cfg):
    res = simulate(sil_base_cfg)
    d = res.deployment
    assert 0.8 < d["dt_from_apogee_s"] < 2.0


# ------------------------------------------------------------- SIL runs --

def test_sil_stub_full_run(fc_exe, sil_base_cfg):
    sil = SilConfig(seed=11, fc_exe=str(fc_exe), fc_mode="stub", pre_launch_s=2.0, post_landing_s=3.0)
    res = run_sil(sil_base_cfg, sil)
    log = res.log
    assert res.flight.landed
    assert not res.fc_failed and res.protocol_errors == 0
    assert res.fc_returncode == 0
    assert res.deploy_mechanism == "motor"      # the stub never commands deploy
    assert res.fc_deploy_t is None
    n_pre, n_post = 200, 300
    assert log["t"][n_pre] == 0.0
    assert len(log["t"]) > n_pre + n_post + 1000
    assert np.allclose(np.diff(log["t"]), 0.01, atol=1e-9)
    assert np.all(log["sent"]) and np.all(log["reply_ok"])
    assert set(log["fc_state"]) == {"PAD"}
    assert np.allclose(log["est_alt"], log["baro"], atol=5e-4)   # stub echoes baro
    assert np.all(log["z_true"][:n_pre] == 0.0) and np.all(log["z_true"][-n_post:] == 0.0)


@pytest.mark.req("REQ-005")
def test_sil_watchdog_end_to_end_backup_charge_deploys(fc_exe, sil_base_cfg):
    sil = SilConfig(seed=2, fc_exe=str(fc_exe), fc_args=("--inject-hang-at", "3.0"),
                    watchdog_timeout_s=0.5, pre_launch_s=1.0, post_landing_s=1.0)
    res = run_sil(sil_base_cfg, sil)
    assert res.fc_failed
    assert res.failure_t == pytest.approx(3.0, abs=1e-9)
    assert "watchdog_timeout" in res.failure_reason
    assert res.deploy_mechanism == "motor" and res.flight.landed
    after = res.log["t"] > 3.0 + 1e-9
    assert after.any() and not res.log["sent"][after].any()     # no frames to a dead FC


@pytest.mark.req("REQ-006")
def test_sil_run_is_reproducible(fc_exe, sil_base_cfg):
    sil = SilConfig(seed=5, fc_exe=str(fc_exe), pre_launch_s=0.5, post_landing_s=0.5)
    a, b = run_sil(sil_base_cfg, sil), run_sil(sil_base_cfg, sil)
    for k in ("t", "baro", "accel", "est_alt", "fc_state"):
        assert np.array_equal(a.log[k], b.log[k]), k
