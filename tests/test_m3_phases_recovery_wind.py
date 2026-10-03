"""Milestone 3: flight phases, parachute recovery, wind."""

import math

import numpy as np
import pytest

from sim.atmosphere import density
from sim.config import REPO_ROOT, FlightConfig
from sim.flight import EVENT_ORDER, PHASES, simulate
from sim.physics import G0, wind_velocity

DEFAULT_CONFIG = REPO_ROOT / "configs" / "default.json"


def default_cfg(**overrides):
    cfg = FlightConfig.load(DEFAULT_CONFIG)
    cfg.wind.speed_mps = 0.0
    cfg.sim.dt_s = 0.005
    for path, value in overrides.items():
        section, key = path.split("__")
        setattr(getattr(cfg, section), key, value)
    cfg.validate()
    return cfg


def windy_cfg():
    return default_cfg(wind__speed_mps=6.0, wind__toward_deg=90.0, wind__shear_exponent=1 / 7)


def angled_cfg():
    return default_cfg(launch__tilt_deg=15.0, launch__azimuth_deg=45.0, wind__speed_mps=3.0,
                       wind__toward_deg=200.0)


# ------------------------------------------------------------- phases ------

def phase_blocks(phases):
    """Collapse consecutive duplicates: [PAD, PAD, RAIL, ...] -> [PAD, RAIL, ...]."""
    blocks = [phases[0]]
    for p in phases[1:]:
        if p != blocks[-1]:
            blocks.append(p)
    return blocks


@pytest.mark.parametrize("make_cfg", [default_cfg, windy_cfg, angled_cfg],
                         ids=["default", "windy", "angled"])
def test_nominal_flights_visit_every_phase_once_in_order(make_cfg):
    res = simulate(make_cfg())
    assert phase_blocks(res.phase) == PHASES


@pytest.mark.parametrize("delay", [None, 0.0, 3.0, 7.0, 30.0])
def test_phase_order_valid_for_any_deploy_delay(delay):
    res = simulate(default_cfg(recovery__deploy_delay_s=delay) if delay is not None
                   else default_cfg(motor__ejection_delay_s=None))
    blocks = phase_blocks(res.phase)
    idx = [PHASES.index(p) for p in blocks]
    assert idx == sorted(set(idx)), f"phases out of order or repeated: {blocks}"
    assert blocks[-1] == "LANDED"
    assert len(res.phase) == len(res.t)


def test_events_recorded_in_physical_order():
    res = simulate(default_cfg())
    names = [n for n in EVENT_ORDER if n in res.events]
    assert names == ["liftoff", "rail_exit", "burnout", "deploy", "apogee", "landing"]
    times = [res.events[n].t for n in names if n != "deploy"]
    assert times == sorted(times)
    for ev in res.events.values():
        assert ev.position.shape == (3,) and ev.velocity.shape == (3,)


# ------------------------------------------------------------- apogee ------

def test_apogee_event_is_where_vertical_velocity_crosses_zero():
    res = simulate(default_cfg(recovery__deploy_delay_s=8.0))  # chute opens after apogee
    ap = res.events["apogee"]
    assert abs(ap.velocity[2]) < 1e-9
    before = res.t < ap.t
    after = res.t > ap.t
    k = np.searchsorted(res.t, ap.t)
    assert res.velocity[k - 1, 2] > 0.0 and res.velocity[k, 2] <= 0.0
    assert np.all(res.velocity[before & (res.t > res.events["rail_exit"].t), 2] > 0.0)
    assert np.all(res.velocity[after, 2] <= 0.0)
    # The event is (to interpolation accuracy) the highest point of the flight.
    assert ap.position[2] == pytest.approx(np.max(res.position[:, 2]), abs=1e-3)


# ---------------------------------------------------------- recovery -------

def test_descent_speed_converges_to_chute_terminal_velocity():
    cfg = default_cfg()
    res = simulate(cfg)
    rec = cfg.recovery
    area = math.pi * rec.chute_diameter_m**2 / 4.0
    t_dep, t_land = res.events["deploy"].t, res.events["landing"].t
    late = res.t > t_dep + 0.8 * (t_land - t_dep)          # last 20% of the descent
    speed = np.linalg.norm(res.velocity[late], axis=1)
    rho = np.array([density(z) for z in res.position[late, 2]])
    v_term = np.sqrt(2.0 * res.mass[late] * G0 / (rho * rec.chute_cd * area))
    assert np.max(np.abs(speed / v_term - 1.0)) < 0.02
    print(f"\n[chute] terminal velocity at landing ~ {v_term[-1]:.2f} m/s, sim {speed[-1]:.2f} m/s")


def test_deploy_time_is_burnout_plus_delay_and_is_logged():
    res = simulate(default_cfg())
    dep = res.events["deploy"]
    assert dep.t == pytest.approx(res.events["burnout"].t + 5.0, abs=1e-12)
    d = res.deployment
    assert d["deployed"] is True
    assert d["timing"] in ("before", "near", "after")
    assert d["speed_mps"] == pytest.approx(np.linalg.norm(dep.velocity))
    # Timing is judged against TRUE apogee (no-chute peak), which is later and
    # higher than the observed apogee when the chute opens while still climbing.
    assert d["dt_from_apogee_s"] == pytest.approx(dep.t - d["true_apogee_t"])
    assert dep.velocity[2] > 0.0
    assert d["true_apogee_t"] > res.events["apogee"].t
    assert d["true_apogee_m"] > res.apogee_m


def test_true_apogee_equals_observed_apogee_when_chute_opens_after_peak():
    res = simulate(default_cfg(recovery__deploy_delay_s=9.0))
    d = res.deployment
    assert d["true_apogee_t"] == res.events["apogee"].t
    assert d["true_apogee_m"] == res.apogee_m


def test_shadow_true_apogee_matches_a_no_chute_flight():
    no_chute = simulate(default_cfg(recovery__enabled=False))
    early = simulate(default_cfg(recovery__deploy_delay_s=0.0))
    assert early.deployment["true_apogee_m"] == pytest.approx(no_chute.apogee_m, abs=0.01)
    assert early.deployment["true_apogee_t"] == pytest.approx(no_chute.events["apogee"].t, abs=0.01)


@pytest.mark.parametrize("delay,expected", [(0.0, "before"), (5.5, "near"), (9.0, "after")])
def test_deploy_timing_classification(delay, expected):
    res = simulate(default_cfg(recovery__deploy_delay_s=delay))
    assert res.deployment["timing"] == expected


def test_early_deploy_is_fast_and_lowers_apogee():
    early = simulate(default_cfg(recovery__deploy_delay_s=0.0))
    nominal = simulate(default_cfg())
    assert early.deployment["speed_mps"] > 5 * nominal.deployment["speed_mps"]
    assert early.apogee_m < nominal.apogee_m


def test_plugged_motor_never_deploys():
    res = simulate(default_cfg(motor__ejection_delay_s=None))
    assert "deploy" not in res.events
    assert res.deployment["deployed"] is False
    assert res.landed


# --------------------------------------------------------------- wind ------

def test_wind_profile_constant_below_reference_and_power_law_above():
    cfg = windy_cfg()
    w = cfg.wind
    assert np.allclose(wind_velocity(0.0, w), [6.0, 0.0, 0.0])
    assert np.allclose(wind_velocity(w.reference_height_m, w), [6.0, 0.0, 0.0])
    z = 4 * w.reference_height_m
    assert np.allclose(wind_velocity(z, w), [6.0 * 4 ** (1 / 7), 0.0, 0.0])
    # toward 0 deg = north = +y
    w.toward_deg = 0.0
    assert np.allclose(wind_velocity(0.0, w), [0.0, 6.0, 0.0])


def test_wind_toward_plus_x_shifts_landing_toward_plus_x():
    calm = simulate(default_cfg())
    windy = simulate(default_cfg(wind__speed_mps=4.0, wind__toward_deg=90.0))
    assert windy.landing_point[0] > calm.landing_point[0] + 10.0
    assert abs(windy.landing_point[1]) < 1e-6


def test_zero_wind_vertical_launch_lands_at_origin():
    res = simulate(default_cfg())
    assert np.hypot(*res.landing_point[:2]) < 1e-9


def test_wind_acts_only_through_drag():
    """In vacuum there is no drag, so wind must have no effect at all."""
    base = default_cfg(atmosphere__model="vacuum", motor__ejection_delay_s=None)
    windy = base.copy()
    windy.wind.speed_mps = 10.0
    a, b = simulate(base), simulate(windy)
    assert np.array_equal(a.position, b.position)
