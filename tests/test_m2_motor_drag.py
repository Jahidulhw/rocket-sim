"""Milestone 2: real thrust curve, mass loss, ISA atmosphere, drag."""

import numpy as np
import pytest

from sim.atmosphere import density
from sim.config import REPO_ROOT, AtmosphereConfig, FlightConfig, MotorConfig
from sim.flight import simulate
from sim.motor import ThrustCurveMotor, load_eng
from sim.physics import drag_force

C6_PATH = REPO_ROOT / "data" / "motors" / "Estes_C6.eng"
C6_PUBLISHED_IMPULSE_NS = 8.82   # thrustcurve.org / NAR certification summary
DEFAULT_CONFIG = REPO_ROOT / "configs" / "default.json"


@pytest.fixture(scope="module")
def c6():
    return load_eng(C6_PATH)


# ---------------------------------------------------------------- motor ----

def test_eng_header_parsed(c6):
    assert c6.name == "C6"
    assert c6.diameter_m == pytest.approx(0.018)
    assert c6.length_m == pytest.approx(0.070)
    assert c6.delays_s == [0.0, 3.0, 5.0, 7.0]
    assert c6.propellant_mass == pytest.approx(0.0108)
    assert c6.total_mass == pytest.approx(0.0231)
    assert c6.burn_time == pytest.approx(1.86)


def test_integrated_thrust_matches_published_total_impulse(c6):
    # Independent numerical integration of the interpolated curve (not the
    # loader's own bookkeeping) on a fine grid.
    t = np.linspace(0.0, c6.burn_time + 0.1, 200_001)
    f = np.array([c6.thrust(x) for x in t])
    impulse = np.trapezoid(f, t)
    assert impulse == pytest.approx(C6_PUBLISHED_IMPULSE_NS, rel=0.01)
    assert c6.total_impulse == pytest.approx(impulse, rel=1e-4)


def test_thrust_is_linear_between_points_and_zero_outside(c6):
    # Midway between (0.192, 14.090) and (0.209, 11.446)
    assert c6.thrust(0.2005) == pytest.approx(0.5 * (14.090 + 11.446))
    assert c6.thrust(-0.1) == 0.0
    assert c6.thrust(c6.burn_time) == 0.0
    assert c6.thrust(c6.burn_time + 1.0) == 0.0
    # The file starts at t = 0.031 s; the loader adds an implicit (0, 0) point.
    assert c6.thrust(0.0) == 0.0
    assert c6.thrust(0.0155) == pytest.approx(0.5 * 0.946)


def test_mass_flow_proportional_to_thrust(c6):
    for t in (0.1, 0.5, 1.5):
        assert c6.mass_flow(t) == pytest.approx(c6.propellant_mass * c6.thrust(t) / c6.total_impulse)


@pytest.mark.parametrize("text", [
    "",                                                  # empty
    "; only comments\n",                                 # no header
    "C6 18 70 0-3-5-7 .0108 .0231 E\n",                  # header but no data
    "C6 18 70 0-3-5-7 .0108 .0231 E\n0.2 5\n0.1 4\n",    # time going backwards
    "C6 18 70 0-3-5-7 .0108\n0.1 5\n0.2 0\n",            # header too short
])
def test_malformed_eng_rejected(tmp_path, text):
    p = tmp_path / "bad.eng"
    p.write_text(text)
    with pytest.raises(ValueError):
        load_eng(p)


def test_plugged_delay_parsed(tmp_path):
    p = tmp_path / "plugged.eng"
    p.write_text("X1 18 70 P .01 .02 Test\n0.1 5\n0.5 0\n")
    assert load_eng(p).delays_s == []


# ---------------------------------------------------------------- mass -----

def c6_flight_config(model="isa"):
    cfg = FlightConfig.load(DEFAULT_CONFIG)
    cfg.atmosphere.model = model
    return cfg


def test_mass_at_burnout_equals_initial_minus_propellant():
    res = simulate(c6_flight_config())
    m0 = res.mass[0]
    i_burn = np.searchsorted(res.t, res.events["burnout"].t)
    assert res.t[i_burn] == pytest.approx(res.motor.burn_time)
    assert res.mass[i_burn] == pytest.approx(m0 - res.motor.propellant_mass, abs=1e-7)
    assert np.all(np.diff(res.mass) <= 1e-15)                         # never gains mass
    assert np.all(res.mass[i_burn:] == res.mass[i_burn])               # constant after burnout


# ---------------------------------------------------------- atmosphere -----

def test_density_sea_level_and_monotonic():
    assert density(0.0) == 1.225
    h = np.linspace(0.0, 20_000.0, 2001)
    rho = np.array([density(x) for x in h])
    assert np.all(np.diff(rho) < 0.0)


@pytest.mark.parametrize("h,rho_table", [(1000.0, 1.1117), (5000.0, 0.73643), (11000.0, 0.36392)])
def test_density_matches_isa_table(h, rho_table):
    assert density(h) == pytest.approx(rho_table, rel=2e-3)


# ---------------------------------------------------------------- drag -----

def test_drag_force_formula_and_direction():
    v_rel = np.array([3.0, -4.0, 12.0])        # |v_rel| = 13
    rho, cd, area = 1.2, 0.8, 0.01
    d = drag_force(v_rel, rho, cd, area)
    assert np.allclose(d, -0.5 * rho * cd * area * 13.0 * v_rel)
    assert d @ v_rel < 0.0
    assert np.all(drag_force(np.zeros(3), rho, cd, area) == 0.0)


def test_drag_lowers_apogee():
    vac = simulate(c6_flight_config("vacuum")).apogee_m
    air = simulate(c6_flight_config("isa")).apogee_m
    assert air < vac


def test_default_rocket_apogee_is_plausible():
    res = simulate(c6_flight_config())
    print(f"\n[default rocket, Estes C6] apogee = {res.apogee_m:.1f} m "
          f"({res.apogee_m * 3.28084:.0f} ft), max speed = {res.max_speed_mps:.1f} m/s, "
          f"apogee time = {res.events['apogee'].t:.2f} s")
    # Small 24.8 mm, ~64 g rocket on a C6: hobby references put this class at
    # a few hundred metres. A wide band catches unit/sign errors, not tuning.
    assert 200.0 < res.apogee_m < 550.0
