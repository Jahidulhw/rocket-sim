"""Phase A, milestone A2: Mach-dependent drag coefficient."""

from __future__ import annotations

import numpy as np
import pytest

from sim.aero import MachDrag, drag_coefficient
from sim.atmosphere import speed_of_sound
from sim.config import REPO_ROOT, FlightConfig, apply_overrides
from sim.flight import simulate
from sim.physics import Dynamics

DEFAULT = REPO_ROOT / "configs" / "default.json"
P = MachDrag()     # default shape parameters


def test_subsonic_plateau_equals_constant_cd_exactly():
    for cd0 in (0.3, 0.45, 0.75):
        for m in np.linspace(0.0, 0.3, 61):
            assert drag_coefficient(cd0, float(m), P) == cd0          # bitwise
        for m in np.linspace(0.3, P.mach_critical, 21):
            assert drag_coefficient(cd0, float(m), P) == cd0


def test_shape_rise_peak_and_supersonic_decline():
    cd0 = 0.5
    assert drag_coefficient(cd0, P.mach_peak, P) == pytest.approx(cd0 * P.peak_factor)
    assert drag_coefficient(cd0, P.mach_supersonic, P) == pytest.approx(cd0 * P.supersonic_factor)
    rise = [drag_coefficient(cd0, m, P) for m in np.linspace(P.mach_critical, P.mach_peak, 50)]
    fall = [drag_coefficient(cd0, m, P) for m in np.linspace(P.mach_peak, P.mach_supersonic, 50)]
    assert all(b > a for a, b in zip(rise, rise[1:]))       # transonic rise
    assert all(b < a for a, b in zip(fall, fall[1:]))       # supersonic decline
    assert drag_coefficient(cd0, 3.0, P) == pytest.approx(cd0 * P.supersonic_factor)
    assert max(rise + fall) == pytest.approx(cd0 * P.peak_factor)


def test_cd_is_continuous_and_smooth_across_mach():
    m = np.linspace(0.0, 3.0, 30001)                     # dM = 1e-4
    cd = np.array([drag_coefficient(0.5, float(x), P) for x in m])
    jumps = np.abs(np.diff(cd))
    assert jumps.max() < 1e-3                            # C0: no step anywhere
    slope = np.diff(cd) / np.diff(m)
    # C1: the slope never jumps between neighbouring samples, including at
    # the segment joints (critical, peak, supersonic Mach).
    assert np.abs(np.diff(slope)).max() < 0.05
    for joint in (P.mach_critical, P.mach_peak, P.mach_supersonic):
        lo, hi = drag_coefficient(0.5, joint - 1e-9, P), drag_coefficient(0.5, joint + 1e-9, P)
        assert hi == pytest.approx(lo, abs=1e-8)


@pytest.mark.parametrize("bad", [dict(mach_critical=1.2), dict(mach_peak=2.5), dict(peak_factor=0.9),
                                 dict(supersonic_factor=2.5), dict(mach_critical=0.2)])
def test_invalid_shape_rejected(bad):
    with pytest.raises(ValueError):
        MachDrag(**bad).validate()


def test_mach_model_on_subsonic_rocket_is_bitwise_identical():
    """The C6 rocket peaks at Mach 0.299: switching it to the Mach model must
    not change a single bit of the trajectory."""
    base = FlightConfig.load(DEFAULT)
    a = simulate(base)
    b = simulate(apply_overrides(base, {"rocket.drag_model": "mach"}))
    assert np.array_equal(a.position, b.position) and np.array_equal(a.t, b.t)
    assert a.max_speed_mps / speed_of_sound(0.0) < 0.3


def test_default_config_still_uses_constant_cd():
    assert FlightConfig.load(DEFAULT).rocket.drag_model == "constant"


def _supersonic_cfg(model: str) -> FlightConfig:
    """Minimum-diameter 54 mm high-power rocket on a Cesaroni K940 (test-only)."""
    cfg = FlightConfig.load(DEFAULT)
    return apply_overrides(cfg, {
        "rocket.dry_mass_kg": 1.2, "rocket.body_diameter_m": 0.056, "rocket.cd": 0.45,
        "rocket.drag_model": model, "motor.eng_file": "data/motors/Cesaroni_K940.eng",
        "motor.ejection_delay_s": None, "recovery.enabled": False, "launch.rail_length_m": 2.4,
        "wind.speed_mps": 0.0, "sim.dt_s": 0.002})


def test_supersonic_flight_shows_transonic_drag_rise():
    const, mach = simulate(_supersonic_cfg("constant")), simulate(_supersonic_cfg("mach"))
    dyn = Dynamics(_supersonic_cfg("mach"), mach.motor)
    speeds = np.linalg.norm(mach.velocity, axis=1)
    m_num = speeds / np.array([speed_of_sound(z) for z in mach.position[:, 2]])
    assert m_num.max() > 1.05                               # actually supersonic
    # The physics uses the model's Cd at every state (checked at the Mach maximum,
    # which for this rocket lies past the drag peak, in the supersonic decline).
    i = int(np.argmax(m_num))
    expected = drag_coefficient(0.45, float(m_num[i]), MachDrag())
    assert dyn.drag_cd(mach.position[i, 2], mach.velocity[i]) == pytest.approx(expected, rel=1e-9)
    # Passing through the transonic peak on the way up, Cd is ~1.9 x the constant value.
    up = np.arange(len(m_num)) < i
    j = int(np.argmin(np.abs(m_num[up] - MachDrag().mach_peak)))
    assert dyn.drag_cd(mach.position[j, 2], mach.velocity[j]) > 1.85 * 0.45
    # More drag through the transonic region => lower apogee and lower peak speed.
    assert mach.apogee_m < 0.97 * const.apogee_m
    assert mach.max_speed_mps < const.max_speed_mps


def test_monte_carlo_cd_scale_scales_the_whole_curve():
    # Monte Carlo perturbs rocket.cd (= cd0); the shape factors are relative,
    # so the whole Cd(M) curve scales with it.
    for m in (0.2, 1.0, 1.6):
        assert drag_coefficient(0.45 * 1.1, m, P) == pytest.approx(1.1 * drag_coefficient(0.45, m, P))
