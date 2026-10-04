"""Phase A, milestone A3: preset fleet, passive two-stage flight, dual deploy."""

from __future__ import annotations

import json
import math
import re

import numpy as np
import pytest

from sim.atmosphere import density, speed_of_sound
from sim.config import REPO_ROOT, FlightConfig, apply_overrides
from sim.fleet import CATEGORIES, PRESET_DIR, fly_preset, load_fleet, preset_from_dict, preset_motors
from sim.flight import simulate
from sim.motor import DelayedMotor, build_motor

FLEET = {p.id: p for p in load_fleet()}
DEFAULT = REPO_ROOT / "configs" / "default.json"


@pytest.fixture(scope="module")
def flights():
    return {pid: fly_preset(p) for pid, p in FLEET.items()}


def max_mach(f) -> float:
    v = np.linalg.norm(f.velocity, axis=1)
    return float(np.max(v / np.array([speed_of_sound(z) for z in f.position[:, 2]])))


# ---------------------------------------------------------------- presets --

def test_fleet_covers_every_category():
    assert sorted({p.category for p in FLEET.values()}) == sorted(CATEGORIES)
    assert sum(p.two_stage for p in FLEET.values()) == 1


def test_classic_preset_is_the_default_config_verbatim():
    classic = FLEET["classic-c6"]
    assert classic.flight == FlightConfig.load(DEFAULT)
    a, b = fly_preset(classic).flight, simulate(FlightConfig.load(DEFAULT))
    assert np.array_equal(a.t, b.t) and np.array_equal(a.position, b.position)


def test_presets_stay_in_scope():
    """Hobby / high-power / generic research rockets only: no weapon terms anywhere."""
    banned = ("missile", "warhead", "munition", "weapon", "target", "guidance", "icbm", "rpg", "artillery")
    for path in PRESET_DIR.glob("*.json"):
        text = path.read_text(encoding="utf-8").lower()
        assert not [w for w in banned if w in text], path.name


@pytest.mark.parametrize("pid", list(FLEET))
def test_every_preset_flies_to_a_plausible_apogee(flights, pid, capsys):
    p, r = FLEET[pid], flights[pid]
    f = r.flight
    assert f.landed and f.deployment["deployed"]
    lo, hi = p.expected["apogee_m"]
    assert lo <= f.apogee_m <= hi, f"{pid}: apogee {f.apogee_m:.0f} m outside {lo}-{hi}"
    mlo, mhi = p.expected["max_mach"]
    assert mlo <= max_mach(f) <= mhi
    if r.booster is not None:
        assert r.booster.landed


def test_high_power_preset_goes_supersonic(flights):
    assert max_mach(flights["swift-k940"].flight) > 1.2


def test_fleet_table(flights, capsys):
    rows = []
    for pid, r in flights.items():
        f, p = r.flight, FLEET[pid]
        motors = preset_motors(p)
        rows.append(f"{p.label:42} {motors:24} {f.apogee_m:8.0f} {f.max_speed_mps:8.1f} {max_mach(f):6.2f} "
                    f"{f.landing_distance_m:8.0f}")
    with capsys.disabled():
        print(f"\n{'rocket':42} {'motor(s)':24} {'apogee':>8} {'vmax':>8} {'Mach':>6} {'landing':>8}")
        print("\n".join(rows))
    assert len(rows) == len(FLEET)


@pytest.mark.parametrize("mutate, msg", [
    (lambda d: d.update(colour="red"), "unknown keys"),
    (lambda d: d.update(category="interceptor"), "category"),
    (lambda d: d.pop("flight"), "missing required key 'flight'"),
    (lambda d: d["flight"]["rocket"].update(dry_mass_kg=-1), "dry_mass_kg"),
    (lambda d: d["booster"].pop("stack_cd"), "missing ['stack_cd']"),
    (lambda d: d["booster"].update(sustainer_ignition_delay_s=60), "sustainer_ignition_delay_s"),
    (lambda d: d["flight"]["recovery"].update(main_deploy_altitude_m=None), "dual deploy needs both"),
])
def test_invalid_preset_rejected_with_useful_message(mutate, msg):
    d = json.loads((PRESET_DIR / "argo-2stage.json").read_text(encoding="utf-8"))
    mutate(d)
    with pytest.raises(ValueError, match=re.escape(msg)):
        preset_from_dict(d)


# ------------------------------------------------------- passive staging --

@pytest.fixture(scope="module")
def staged(flights):
    return flights["argo-2stage"].staged


def test_separation_conserves_momentum_and_mass(staged):
    stack_end = staged.stack.final_state
    b_first = np.r_[staged.booster_segment.position[0], staged.booster_segment.velocity[0], staged.booster_segment.mass[0]]
    s_first = np.r_[staged.sustainer_segment.position[0], staged.sustainer_segment.velocity[0],
                    staged.sustainer_segment.mass[0]]
    m_stack, m_b, m_s = stack_end[6], b_first[6], s_first[6]
    assert m_b + m_s == pytest.approx(m_stack, rel=1e-12)              # mass bookkeeping
    p_before = m_stack * stack_end[3:6]
    p_after = m_b * b_first[3:6] + m_s * s_first[3:6]
    assert p_after == pytest.approx(p_before, rel=1e-12)               # momentum
    assert np.array_equal(b_first[0:3], stack_end[0:3]) and np.array_equal(s_first[0:3], stack_end[0:3])


def test_mass_bookkeeping_matches_components(staged):
    p = FLEET["argo-2stage"]
    bm, sm = build_motor(p.booster.flight.motor), build_motor(p.flight.motor)
    b_dry, s_dry = p.booster.flight.rocket.dry_mass_kg, p.flight.rocket.dry_mass_kg
    assert staged.stack.mass[0] == pytest.approx(b_dry + bm.total_mass + s_dry + sm.total_mass, rel=1e-12)
    casing = bm.total_mass - bm.propellant_mass
    assert staged.stack.mass[-1] == pytest.approx(b_dry + casing + s_dry + sm.total_mass, rel=1e-9)
    assert staged.booster_segment.mass[0] == pytest.approx(b_dry + casing, rel=1e-12)
    assert staged.booster_segment.mass[-1] == pytest.approx(b_dry + casing, rel=1e-12)   # nothing left to burn
    s_casing = sm.total_mass - sm.propellant_mass
    assert staged.sustainer_segment.mass[-1] == pytest.approx(s_dry + s_casing, rel=1e-9)


def test_separation_at_booster_burnout_and_delayed_ignition(staged):
    p = FLEET["argo-2stage"]
    bm = build_motor(p.booster.flight.motor)
    assert staged.separation_t == pytest.approx(bm.burn_time)
    assert staged.ignition_t == pytest.approx(staged.separation_t + p.booster.sustainer_ignition_delay_s)
    f = staged.flight
    assert f.events["ignition"].t == pytest.approx(staged.ignition_t, abs=1e-12)
    gap = (f.t > staged.separation_t) & (f.t < staged.ignition_t)
    assert gap.any()
    assert np.all(f.mass[gap] == f.mass[gap][0])            # unlit: no propellant used in the gap
    assert set(np.array(f.phase)[gap]) == {"COAST"}


def test_sustainer_phase_sequence_has_two_burns(staged):
    seq = []
    for ph in staged.flight.phase:
        if not seq or seq[-1] != ph:
            seq.append(ph)
    assert seq == ["PAD", "RAIL", "BOOST", "COAST", "BOOST", "COAST", "DESCENT", "LANDED"]


def test_booster_falls_behind_by_drag_separation(staged):
    t1 = staged.separation_t + 0.9                          # still before sustainer ignition
    s, b = staged.sustainer_segment, staged.booster_segment
    zs, zb = np.interp(t1, s.t, s.position[:, 2]), np.interp(t1, b.t, b.position[:, 2])
    vs, vb = np.interp(t1, s.t, s.velocity[:, 2]), np.interp(t1, b.t, b.velocity[:, 2])
    assert vs > vb + 5.0 and zs > zb


def test_both_bodies_land_separately_with_their_own_recovery(staged):
    s, b = staged.flight, staged.booster
    assert s.landed and b.landed
    assert abs(s.flight_time_s - b.flight_time_s) > 60.0
    assert np.linalg.norm(s.landing_point - b.landing_point) > 50.0
    assert b.deployment["mechanism"] == "motor"            # booster: its own ejection charge
    assert s.deployment["mechanism"] == "timer"            # open loop: independent backup timer
    assert b.deployment["t"] > staged.separation_t
    assert s.deployment["t"] > s.events["burnout"].t       # never during either boost or the gap


# -------------------------------------------------- dual deploy / timer --

def test_dual_deploy_main_opens_at_altitude_and_slows_descent(flights):
    f = flights["swift-k940"].flight
    rec = FLEET["swift-k940"].flight.recovery
    main = f.events["main_deploy"]
    assert main.position[2] == pytest.approx(rec.main_deploy_altitude_m, abs=1.0)
    assert main.velocity[2] < 0 and main.t > f.events["apogee"].t
    m = f.mass[-1]

    def terminal(cd, d, z):
        return math.sqrt(2 * m * 9.81 / (density(z) * cd * math.pi * d * d / 4))
    i_drogue = np.searchsorted(f.t, main.t - 5.0)
    assert -f.velocity[i_drogue, 2] == pytest.approx(terminal(rec.chute_cd, rec.chute_diameter_m,
                                                              f.position[i_drogue, 2]), rel=0.03)
    i_main = np.searchsorted(f.t, f.events["landing"].t - 5.0)
    assert -f.velocity[i_main, 2] == pytest.approx(terminal(rec.main_cd, rec.main_diameter_m,
                                                            f.position[i_main, 2]), rel=0.03)
    assert -f.velocity[i_main, 2] < 0.5 * -f.velocity[i_drogue, 2]


def test_backup_timer_device_fires_at_its_preset_time(flights):
    f = flights["swift-k940"].flight
    assert f.deployment["mechanism"] == "timer"
    assert f.deployment["t"] == pytest.approx(FLEET["swift-k940"].flight.recovery.backup_timer_s, abs=1e-9)
    assert f.deployment["timing"] in ("near", "after")      # set after the latest dispersed apogee


def test_backup_timer_validation():
    with pytest.raises(ValueError, match="backup_timer_s"):
        apply_overrides(FlightConfig.load(DEFAULT), {"recovery.backup_timer_s": -1.0})


class _Commander:
    """Scripted controller: drogue at t_d, main at t_m (bitmask 1 / 2)."""
    period_s = 0.01

    def __init__(self, t_d, t_m):
        self.t_d, self.t_m = t_d, t_m

    def tick(self, t, y, accel):
        return (1 if t >= self.t_d else 0) | (2 if t >= self.t_m else 0)


def test_controller_bitmask_commands_drogue_then_main():
    p = FLEET["swift-k940"]
    cfg = apply_overrides(p.flight, {"recovery.backup_timer_s": None})
    f = simulate(cfg, controller=_Commander(24.6, 100.0))
    assert f.deployment["mechanism"] == "fc" and f.deployment["t"] == pytest.approx(24.6, abs=1e-9)
    assert f.deployment["main_mechanism"] == "fc" and f.deployment["main_t"] == pytest.approx(100.0, abs=1e-9)
    # With a controller the physics never auto-fires the main: the FC owns that decision.
    g = simulate(cfg, controller=_Commander(24.6, 1e9))
    assert "main_deploy" not in g.events and g.landed


# ---------------------------------------------------------- DelayedMotor --

def test_delayed_motor_shifts_the_curve_exactly():
    m = build_motor(FLEET["argo-2stage"].flight.motor)
    d = DelayedMotor(m, 4.0)
    assert d.burn_time == pytest.approx(4.0 + m.burn_time)
    for t in (0.0, 3.99, 4.0, 4.3, 5.0, d.burn_time + 0.1):
        assert d.thrust(t) == m.thrust(t - 4.0)
        assert d.mass_flow(t) == m.mass_flow(t - 4.0)
    assert d.breakpoints()[0] == 4.0 and d.breakpoints()[-1] == pytest.approx(d.burn_time)
    assert d.total_impulse == m.total_impulse
