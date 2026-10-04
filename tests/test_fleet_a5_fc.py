"""Phase A, milestone A5: the flight computer across the fleet (per-rocket FC
configuration, two burns, dual deploy)."""

from __future__ import annotations

import subprocess

import pytest

from sim.faults import Fault
from sim.fleet import load_fleet, sil_settings
from sim.requirements import evaluate
from sim.sil import SilConfig, run_sil_preset

FLEET = {p.id: p for p in load_fleet()}
FAST = dict(pre_launch_s=5.0, post_landing_s=12.0)


@pytest.fixture(scope="module")
def nominal(fc_exe):
    return {pid: run_sil_preset(p, SilConfig(seed=1, fc_exe=str(fc_exe), **FAST)) for pid, p in FLEET.items()}


@pytest.mark.req("REQ-001", "REQ-003", "REQ-009", "REQ-010", "REQ-011", "REQ-012", "REQ-013")
@pytest.mark.parametrize("pid", list(FLEET))
def test_every_preset_nominal_sil_flight_meets_all_applicable_requirements(nominal, pid):
    r = nominal[pid]
    for rid, v in evaluate(r).items():
        assert v.status != "fail", (pid, rid, v.detail)
    assert r.fc_deploy_reason == "apogee" and not r.fc_health_events


@pytest.mark.parametrize("pid", list(FLEET))
def test_preset_fc_arguments_are_accepted_by_the_real_fc(fc_exe, pid):
    args = sil_settings(FLEET[pid])["fc_args"]
    p = subprocess.run([str(fc_exe), "--mode", "kalman", *args], input="S 0.000000 0.0 9.81\nEND\n",
                       capture_output=True, text=True, timeout=10)
    assert p.returncode == 0, p.stderr
    assert p.stdout.startswith("R 0.000000 PAD")


def test_filter_r_is_derived_from_the_preset_sensor_model():
    s = sil_settings(FLEET["swift-k940"])
    args = " ".join(s["fc_args"])
    assert f"kf.accel_sigma_mps2={s['sensors'].accel_noise_sigma_mps2:g}" in args
    assert f"accel_full_scale_mps2={s['sensors'].accel_range_mps2:g}" in args
    import copy
    from sim.fleet import preset_from_dict
    import json
    d = json.loads(FLEET["swift-k940"].path.read_text(encoding="utf-8"))
    d = copy.deepcopy(d)
    d["sil"]["fc"]["kf.accel_sigma_mps2"] = 0.1      # hand-set R is refused
    with pytest.raises(ValueError, match="derived from sil.sensors"):
        preset_from_dict(d)


@pytest.mark.req("REQ-011", "REQ-012")
def test_two_stage_staging_is_handled_explicitly(nominal):
    r = nominal["argo-2stage"]
    seq = r.fc_state_sequence()
    assert seq[:6] == ["PAD", "BOOST", "COAST", "BOOST", "COAST", "APOGEE"]
    assert any("STAGING burnout 1 of 2" in line for line in r.fc_stderr)
    sep, final = r.flight.events["booster_burnout"].t, r.flight.events["burnout"].t
    assert r.fc_deploy_t > final > sep
    assert r.booster is not None and r.booster.landed


@pytest.mark.req("REQ-013")
@pytest.mark.parametrize("pid", ["swift-k940", "argo-2stage"])
def test_dual_deploy_main_commanded_near_its_altitude(nominal, pid):
    r = nominal[pid]
    assert r.fc_main_t is not None and r.fc_main_t > r.fc_deploy_t
    assert r.flight.deployment["main_mechanism"] == "fc"
    assert abs(r.flight.deployment["main_altitude_m"] - r.main_altitude_m) <= 25.0


def test_burnout_tailoff_regression_on_a_real_motor(fc_exe):
    """End-to-end regression for the bug the fleet runs found: on the G80T's
    long tail-off, a single (coast) process noise made the gate reject genuine
    samples and a NOMINAL flight went 'inconsistent' -> backup timer. The
    phase-dependent process noise fixes it."""
    p = FLEET["kestrel-g80"]
    old = run_sil_preset(p, SilConfig(seed=1, fc_exe=str(fc_exe), fc_args=("--param", "powered_jerk_psd=10"),
                                      pre_launch_s=5.0, post_landing_s=0.0))
    assert any("INCONSISTENT" in h for h in old.fc_health_events)
    assert old.fc_deploy_reason == "backup_timer"
    new = run_sil_preset(p, SilConfig(seed=1, fc_exe=str(fc_exe), pre_launch_s=5.0, post_landing_s=0.0))
    assert new.fc_health_events == [] and new.fc_deploy_reason == "apogee"


@pytest.mark.req("REQ-011", "REQ-012", "REQ-003")
@pytest.mark.parametrize("fault", [
    Fault("spike", 2.5, 3.0, "baro", 60.0, 0.2),          # spikes across the inter-stage gap
    Fault("spike", 2.5, 3.0, "accel", 120.0, 0.2),
    Fault("dropout", 3.0, 1.5),                           # link blackout across separation + ignition
    Fault("stuck", 1.0, sensor="accel"),                  # accel dead in boost 1: ignition undetectable
    Fault("stuck", 3.5, sensor="baro"),                   # baro frozen in the gap
], ids=lambda f: f.label + f"@{f.start_s:g}")
def test_two_stage_never_deploys_before_final_burnout_under_faults(fc_exe, fault):
    r = run_sil_preset(FLEET["argo-2stage"], SilConfig(seed=7, fc_exe=str(fc_exe), faults=(fault,),
                                                       pre_launch_s=3.0, post_landing_s=0.0))
    v = evaluate(r)
    for rid in ("REQ-003", "REQ-004", "REQ-011", "REQ-012"):
        assert v[rid].status == "pass", (rid, v[rid].detail)


def test_trace_flag_prints_filter_internals(fc_exe):
    p = subprocess.run([str(fc_exe), "--trace"], input="S 0.000000 0.0 9.81\nS 0.010000 0.0 9.81\nEND\n",
                       capture_output=True, text=True, timeout=10)
    assert p.returncode == 0 and "TRACE PAD" in p.stderr and "accel_innov" in p.stderr


def test_supersonic_coast_regression_hot_motor(fc_exe):
    """End-to-end regression for the second bug the fleet campaign found: a
    hot-motor Argo flight decelerates through the transonic drag rise at up to
    ~95 m/s^3 of jerk; with the subsonic coast q (10) the gate rejected those
    genuine samples and a NOMINAL flight went 'inconsistent'. Replays that
    exact dispersed run (fleet campaign seed 20261004)."""
    from sim.config import apply_overrides
    from sim.fleet import sil_flight_config_for
    from sim.montecarlo import perturbed_config
    from sim.staging import BoosterSpec
    p = FLEET["argo-2stage"]
    over = {"wind.speed_mps": 1.7756579814092897, "wind.toward_deg": 116.07613571134604,
            "motor.impulse_scale": 1.0395239575325157, "rocket.cd": 0.44591953097071424,
            "rocket.dry_mass_kg": 0.9993147918894781, "launch.tilt_deg": 0.2295060916830595,
            "launch.azimuth_deg": 168.46841990842535}
    cfg = apply_overrides(sil_flight_config_for(p), over)
    pert = {"wind_speed_offset_mps": 1.7756579814092897 - 2.0, "wind_dir_offset_deg": 116.07613571134604 - 90.0,
            "impulse_scale": 1.0395239575325157, "cd_scale": 0.44591953097071424 / 0.42,
            "dry_mass_scale": 0.9993147918894781 / 1.0, "rail_tilt_east_deg": 0.0, "rail_tilt_north_deg": 0.0}
    booster = BoosterSpec(perturbed_config(p.booster.flight, pert), p.booster.sustainer_ignition_delay_s,
                          p.booster.stack_cd)
    kw = dict(seed=546581527, fc_exe=str(fc_exe), pre_launch_s=5.0, post_landing_s=0.0)
    old = run_sil_preset(p, SilConfig(**kw, fc_args=("--param", "kf.jerk_psd=10")), flight=cfg, booster=booster)
    new = run_sil_preset(p, SilConfig(**kw), flight=cfg, booster=booster)
    assert any("INCONSISTENT" in h for h in old.fc_health_events) and old.fc_deploy_reason == "backup_timer"
    assert new.fc_health_events == [] and new.fc_deploy_reason == "apogee"
    assert abs(new.fc_deploy_t - new.flight.deployment["true_apogee_t"]) < 0.5
