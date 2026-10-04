"""Phase A, milestone A4: geometry schema, Barrowman CP, CG, static margin."""

from __future__ import annotations

import copy
import re

import numpy as np
import pytest

from sim.design import UNDER_STABLE_CAL, assess, build_geometry, center_of_pressure
from sim.fleet import fly_preset, load_fleet, preset_from_dict, preset_stability
from sim.flight import simulate

IN = 0.0254   # the reference example is in inches

# --------------------------------------------- published reference example --
# R. Nakka, "Static Stability - Example C1" (nakka-rocketry.net, 2025): the
# Xi-41 rocket. Tangent-ogive nose 7.44 in, 3.0 in body to 58.74 in, conical
# boattail 3.0 -> 1.9 in over 2.188 in, four trapezoidal fins: root 5.315,
# tip 1.85, semispan 2.755, sweep (root LE to tip LE) 2.303, root LE 52.4 in
# from the nose tip. Published results: nose X 3.467 in, boattail CNa -1.198
# and X 59.75 in, fins CNa 8.03 and X 54.33 in, rocket CP 42.08 in.
# RASAero (independent tool, Barrowman algorithm) gives 42.09 in.
XI41_LENGTH = (7.44 + 51.3 + 2.188) * IN
XI41 = {
    "nose": {"shape": "ogive", "length_m": 7.44 * IN, "diameter_m": 3.0 * IN, "mass_kg": 0.1},
    "body": [{"type": "tube", "length_m": 51.3 * IN, "diameter_m": 3.0 * IN, "mass_kg": 1.0},
             {"type": "transition", "length_m": 2.188 * IN, "fore_diameter_m": 3.0 * IN,
              "aft_diameter_m": 1.9 * IN, "mass_kg": 0.05}],
    "fins": [{"count": 4, "root_chord_m": 5.315 * IN, "tip_chord_m": 1.85 * IN, "semispan_m": 2.755 * IN,
              "sweep_m": 2.303 * IN, "root_le_from_tail_m": XI41_LENGTH - 52.4 * IN, "mass_kg": 0.2}],
}


@pytest.fixture(scope="module")
def xi41():
    return build_geometry(XI41)


def _by_kind(geom, kind):
    return next(c for c in geom.components if c.kind == kind)


def test_barrowman_components_match_published_example(xi41):
    nose, tail, fins = _by_kind(xi41, "nose"), _by_kind(xi41, "transition"), _by_kind(xi41, "fins")
    assert nose.cna == 2.0 and nose.xcp / IN == pytest.approx(3.467, abs=0.002)
    assert tail.cna == pytest.approx(-1.198, abs=0.002)        # boattail: negative (destabilising)
    assert tail.xcp / IN == pytest.approx(59.75, abs=0.01)
    assert fins.cna == pytest.approx(8.03, abs=0.01)
    assert fins.xcp / IN == pytest.approx(54.33, abs=0.01)


def test_barrowman_cp_matches_published_example_and_rasaero(xi41):
    cp_in = center_of_pressure(xi41.components) / IN
    assert cp_in == pytest.approx(42.08, abs=0.05)     # Nakka's hand calculation (round-off)
    assert cp_in == pytest.approx(42.09, abs=0.05)     # RASAero, independent implementation


def test_rectangular_unswept_fin_cp_is_at_quarter_chord():
    """Thin-airfoil theory: the aerodynamic centre of an unswept wing is at 1/4 chord."""
    g = copy.deepcopy(XI41)
    g["fins"][0].update(tip_chord_m=0.1, root_chord_m=0.1, sweep_m=0.0, root_le_from_tail_m=0.3)
    fins = _by_kind(build_geometry(g), "fins")
    assert fins.xcp - fins.x_fore == pytest.approx(0.025, abs=1e-12)


def test_straight_transition_has_no_lift_and_fins_dominate():
    g = copy.deepcopy(XI41)
    g["body"][1].update(aft_diameter_m=3.0 * IN)
    assert _by_kind(build_geometry(g), "transition").cna == pytest.approx(0.0, abs=1e-12)
    assert _by_kind(build_geometry(XI41), "fins").cna > 2.0


# ------------------------------------------------------ CG and margin --

class _Motor:
    def __init__(self, length=0.3, dia=0.038, total=0.6, prop=0.35):
        self.length_m, self.diameter_m, self.total_mass, self.propellant_mass, self.name = length, dia, total, prop, "m"


def test_cg_and_margin(xi41):
    rep = assess([(xi41, _Motor())])
    # Hand check: mass-weighted average of the component CGs and the motor CG.
    m = sum(c.mass for c in xi41.components) + 0.6
    xm = xi41.length_m - 0.15
    expect = (sum(c.mass * c.cg for c in xi41.components) + 0.6 * xm) / m
    assert rep.cg_liftoff_m == pytest.approx(expect, rel=1e-12)
    assert rep.cg_burnout_m < rep.cg_liftoff_m           # propellant (aft) burns away: CG moves forward
    assert rep.margin_burnout_cal > rep.margin_liftoff_cal
    assert rep.margin_liftoff_cal == pytest.approx((rep.cp_m - rep.cg_liftoff_m) / (3.0 * IN), rel=1e-12)


def test_under_and_over_stable_warnings(xi41):
    tail_heavy = copy.deepcopy(XI41)
    tail_heavy["masses"] = [{"name": "ballast", "mass_kg": 5.0, "position_m": XI41_LENGTH - 0.05}]
    rep = assess([(build_geometry(tail_heavy), _Motor())])
    assert not rep.stable and any("UNDER-STABLE" in w for w in rep.warnings)
    nose_heavy = copy.deepcopy(XI41)
    nose_heavy["masses"] = [{"name": "nose weight", "mass_kg": 5.0, "position_m": 0.05}]
    rep = assess([(build_geometry(nose_heavy), _Motor())])
    assert rep.stable and any("OVER-STABLE" in w for w in rep.warnings)


# ----------------------------------------------------- invalid inputs --

@pytest.mark.parametrize("mutate, msg", [
    (lambda g: g["nose"].update(shape="pyramid"), "shape must be one of"),
    (lambda g: g["nose"].update(length_m=-0.1), "length_m must be a positive number"),
    (lambda g: g["nose"].update(length_m=0.01), "shorter than half its diameter"),
    (lambda g: g["body"][0].update(diameter_m=0.05), "does not match"),
    (lambda g: g["body"][1].update(type="sphere"), "type must be 'tube' or 'transition'"),
    (lambda g: g["fins"][0].update(count=5), "count must be 3 or 4"),
    (lambda g: g["fins"][0].update(root_le_from_tail_m=5.0), "must lie on the body"),
    (lambda g: g["fins"][0].update(semispan_m=0.0), "semispan_m must be a positive number"),
    (lambda g: g.update(masses=[{"name": "x", "mass_kg": 0.1, "position_m": 9.0}]), "outside"),
    (lambda g: g.update(wings={}), "unknown keys"),
    (lambda g: g.pop("body"), "'body' must list"),
])
def test_invalid_geometry_rejected_with_useful_message(mutate, msg):
    g = copy.deepcopy(XI41)
    mutate(g)
    with pytest.raises(ValueError, match=re.escape(msg)):
        build_geometry(g)


def test_motor_must_fit(xi41):
    with pytest.raises(ValueError, match="does not fit"):
        assess([(xi41, _Motor(dia=0.06))])
    with pytest.raises(ValueError, match="longer than"):
        assess([(xi41, _Motor(length=3.0))])


def test_preset_geometry_must_agree_with_flight_config():
    import json
    from sim.fleet import PRESET_DIR
    d = json.loads((PRESET_DIR / "kestrel-g80.json").read_text(encoding="utf-8"))
    d["flight"]["rocket"]["dry_mass_kg"] = 0.6
    with pytest.raises(ValueError, match="component masses"):
        preset_from_dict(d)
    d = json.loads((PRESET_DIR / "kestrel-g80.json").read_text(encoding="utf-8"))
    d["flight"]["rocket"]["body_diameter_m"] = 0.05
    with pytest.raises(ValueError, match="body_diameter_m"):
        preset_from_dict(d)


# ------------------------------------------------------------- fleet --

FLEET = load_fleet()


@pytest.mark.parametrize("preset", FLEET, ids=lambda p: p.id)
def test_every_preset_is_stable(preset, capsys):
    reports = preset_stability(preset)
    with capsys.disabled():
        for name, r in reports.items():
            print(f"\n  {preset.id:13} {name:9} CP {r.cp_m:.3f} m  CG {r.cg_liftoff_m:.3f}/{r.cg_burnout_m:.3f} m  "
                  f"margin {r.margin_liftoff_cal:.2f}/{r.margin_burnout_cal:.2f} cal  {r.warnings or ''}", end="")
    for name, r in reports.items():
        assert r.margin_liftoff_cal >= UNDER_STABLE_CAL, (preset.id, name)
        assert r.margin_burnout_cal >= UNDER_STABLE_CAL, (preset.id, name)
        assert not any("UNDER-STABLE" in w for w in r.warnings)
    if preset.two_stage:
        assert set(reports) == {"stack", "sustainer"}     # both flight configurations checked


def test_stability_check_does_not_change_flight():
    p = next(p for p in FLEET if p.id == "kestrel-g80")
    a = simulate(p.flight)
    preset_stability(p)
    b = fly_preset(p).flight
    assert np.array_equal(a.position, b.position)


# --------------------------------------------------- design-your-own CLI --

def test_template_validates_and_check_rocket_cli(tmp_path, capsys):
    import json
    import sys
    from sim.config import REPO_ROOT
    sys.path.insert(0, str(REPO_ROOT / "scripts"))
    import check_rocket
    tmpl = REPO_ROOT / "configs" / "custom_rocket_template.json"
    assert check_rocket.main([str(tmpl)]) == 0
    out = capsys.readouterr().out
    assert "static margin" in out and "apogee" in out
    bad = json.loads(tmpl.read_text(encoding="utf-8"))
    bad["geometry"]["fins"][0]["count"] = 7
    p = tmp_path / "bad.json"
    p.write_text(json.dumps(bad), encoding="utf-8")
    assert check_rocket.main([str(p)]) == 1
    assert "count must be 3 or 4" in capsys.readouterr().out
    unstable = json.loads(tmpl.read_text(encoding="utf-8"))
    unstable["geometry"]["fins"][0]["semispan_m"] = 0.005
    p.write_text(json.dumps(unstable), encoding="utf-8")
    assert check_rocket.main([str(p), "--no-flight"]) == 2
    assert "UNDER-STABLE" in capsys.readouterr().out
