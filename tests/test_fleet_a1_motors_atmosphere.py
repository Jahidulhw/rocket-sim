"""Phase A, milestone A1: bundled motor library and the layered standard atmosphere."""

from __future__ import annotations

import math
import re

import pytest

from sim import atmosphere as atm
from sim.config import REPO_ROOT
from sim.motor import load_eng

MOTOR_DIR = REPO_ROOT / "data" / "motors"
MOTOR_FILES = sorted(MOTOR_DIR.glob("*.eng"))
IMPULSE_CLASSES = "ABCDEFGHIJK"


def _published(path):
    hdr = path.read_text(encoding="utf-8")
    return {"impulse": float(re.search(r"total impulse ([\d.]+) Ns", hdr).group(1)),
            "source": re.search(r'data source: "(\w+)"', hdr).group(1), "header": hdr}


# ----------------------------------------------------------- motor library --

def test_library_covers_hobby_and_high_power_classes():
    classes = set()
    for p in MOTOR_FILES:
        m = load_eng(p)
        # NAR class letter from total impulse: A = 1.26-2.5 Ns, each letter doubles.
        n = math.ceil(math.log2(m.total_impulse / 2.5)) if m.total_impulse > 2.5 else 0
        classes.add(IMPULSE_CLASSES[n])
    assert set("ABCDEFG") <= classes          # hobby range
    assert {"H", "I", "J", "K"} <= classes    # high power for the fleet presets


@pytest.mark.parametrize("path", MOTOR_FILES, ids=lambda p: p.stem)
def test_curve_integrates_to_published_total_impulse(path):
    pub = _published(path)
    m = load_eng(path)
    assert m.total_impulse == pytest.approx(pub["impulse"], rel=0.01)


@pytest.mark.parametrize("path", MOTOR_FILES, ids=lambda p: p.stem)
def test_every_file_records_its_provenance(path):
    hdr = _published(path)["header"]
    assert "thrustcurve.org" in hdr
    assert re.search(r"motorId [0-9a-f]{24}", hdr)
    assert re.search(r"simfile [0-9a-f]{24}", hdr)
    assert _published(path)["source"] in ("cert", "mfr", "user")


@pytest.mark.parametrize("path", MOTOR_FILES, ids=lambda p: p.stem)
def test_motor_file_is_physically_sane(path):
    m = load_eng(path)
    assert 0.0 < m.propellant_mass < m.total_mass
    assert m.burn_time > 0.0 and m.diameter_m > 0.0
    assert m.thrust(m.burn_time) == 0.0 and m.thrust(0.5 * m.burn_time) > 0.0


def test_default_c6_file_untouched():
    # The regression pin depends on this exact file (fetch_motors.py never rewrites it).
    # Line endings are normalised: a Windows checkout with autocrlf may write CRLF.
    import hashlib
    data = (MOTOR_DIR / "Estes_C6.eng").read_bytes().replace(b"\r\n", b"\n")
    assert hashlib.sha256(data).hexdigest() == "546484b816a9861595aeb55203ea093d09ee09a64678f463001f60a08f5271dc"


# -------------------------------------------------------------- atmosphere --
# Reference: U.S. Standard Atmosphere 1976 (= ISA to 32 km and beyond),
# layer-base values at GEOPOTENTIAL altitude. The simulator assumes constant
# gravity, in which geopotential and geometric altitude coincide, so the model
# is evaluated directly at the table altitude.

# (H m, T K, P Pa, rho kg/m^3)
USSA76 = [
    (0.0, 288.15, 101325.0, 1.2250),
    (11000.0, 216.65, 22632.06, 0.36392),
    (20000.0, 216.65, 5474.889, 0.088035),
    (25000.0, 221.65, 2511.02, 0.039466),
    (32000.0, 228.65, 868.0187, 0.013225),
    (47000.0, 270.65, 110.9063, 0.0014275),
    (51000.0, 270.65, 66.93887, 0.00086160),
]


@pytest.mark.parametrize("h, T, P, rho", USSA76)
def test_atmosphere_matches_standard_table(h, T, P, rho):
    assert atm.temperature(h) == pytest.approx(T, abs=1e-6)
    assert atm.pressure(h) == pytest.approx(P, rel=1e-3)
    assert atm.density(h) == pytest.approx(rho, rel=1e-3)


@pytest.mark.parametrize("h, a", [(0.0, 340.294), (11000.0, 295.070), (20000.0, 295.070),
                                  (32000.0, 303.131), (47000.0, 329.799)])
def test_speed_of_sound_matches_standard_table(h, a):
    assert atm.speed_of_sound(h) == pytest.approx(a, rel=1e-4)


@pytest.mark.parametrize("hb", [11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0])
def test_continuous_across_layer_boundaries(hb):
    for f in (atm.temperature, atm.pressure, atm.density, atm.speed_of_sound):
        lo, hi = f(hb - 1e-6), f(hb + 1e-6)
        assert hi == pytest.approx(lo, rel=1e-8), f.__name__


def test_density_strictly_decreasing_to_80_km():
    rho = [atm.density(h) for h in range(0, 80001, 250)]
    assert all(b < a for a, b in zip(rho, rho[1:]))


def test_density_is_pressure_over_rt():
    for h in (0.0, 3000.0, 15000.0, 28000.0, 40000.0):
        assert atm.density(h) == pytest.approx(atm.pressure(h) / (atm.R_AIR * atm.temperature(h)), rel=2e-5)


def test_rho0_override_scales_density_only():
    for h in (0.0, 5000.0, 30000.0):
        assert atm.density(h, rho0=1.1) == pytest.approx(atm.density(h) * 1.1 / atm.RHO0, rel=1e-12)


def test_troposphere_and_lower_stratosphere_formulas_unchanged():
    """The pre-Phase-A formulas, verbatim: the default rocket must see bit-identical air."""
    T0, RHO0, L, R, G = 288.15, 1.225, 0.0065, 287.05287, 9.80665
    exp_ = G / (R * L) - 1.0
    T11 = T0 - L * 11000.0
    ratio11 = (T11 / T0) ** exp_
    def legacy(h):
        if h <= 11000.0:
            return RHO0 * ((T0 - L * h) / T0) ** exp_
        return RHO0 * ratio11 * math.exp(-G * (h - 11000.0) / (R * T11))
    for h in [0.0, 1.0, 37.5, 313.7, 1000.0, 5000.0, 10999.0, 11000.0, 15000.0, 19999.0]:
        assert atm.density(h) == legacy(h)          # bitwise


def test_altitude_above_model_top_is_rejected_loudly():
    with pytest.raises(ValueError):
        atm.density(90_000.0)
