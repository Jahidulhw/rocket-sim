"""Rocket geometry, Barrowman center of pressure, center of gravity, static margin.

This is a DESIGN check: it does not change the point-mass flight model (which
assumes the rocket always flies along its velocity vector, i.e. that it IS
stable). It tells you whether that assumption is justified.

Geometry schema (positions accumulate from the nose tip, nose-to-tail):

    {
      "nose":  {"shape": "ogive" | "cone" | "parabolic", "length_m", "diameter_m", "mass_kg"},
      "body":  [ {"type": "tube", "length_m", "diameter_m", "mass_kg"},
                 {"type": "transition", "length_m", "fore_diameter_m", "aft_diameter_m", "mass_kg"}, ... ],
      "fins":  [ {"count": 3 | 4, "root_chord_m", "tip_chord_m", "semispan_m", "sweep_m",
                  "root_le_from_tail_m", "mass_kg"} ],
      "masses": [ {"name", "mass_kg", "position_m"} ],      point masses (recovery, avionics, ...)
      "motor_aft_offset_m": 0.0                            motor aft end this far forward of the tail
    }

A booster stage uses the same schema WITHOUT a nose (it starts at the
sustainer's tail).

Barrowman method (J. S. Barrowman & J. A. Barrowman, "The Theoretical
Prediction of the Center of Pressure", NARAM-8, 1966), subsonic, small angle
of attack, body lift neglected:

  nose        CNa = 2;  X = k L  (cone 0.666, ogive 0.466, parabolic 0.5)
  transition  CNa = 2 [(d_aft/d)^2 - (d_fore/d)^2]
              X = X_p + (L/3) [1 + (1 - d_f/d_a) / (1 - (d_f/d_a)^2)]
  fins        CNa = [1 + R/(S+R)] * 4 N (S/d)^2 / (1 + sqrt(1 + (2 l_m / (C_r + C_t))^2))
              X = X_b + X_r (C_r + 2 C_t) / (3 (C_r + C_t))
                      + (1/6) [C_r + C_t - C_r C_t / (C_r + C_t)]
              l_m = sqrt(S^2 + (X_r + C_t/2 - C_r/2)^2)   (mid-chord line length)
  CP = sum(CNa_i X_i) / sum(CNa_i)

d = reference diameter (nose base), R = body radius at the fins. The fin
formula is for 3 or 4 fins (other counts need a different N factor), so only
those are accepted. Validated against R. Nakka's worked "Xi-41" example,
which also matches RASAero (tests/test_fleet_a4_stability.py).

Static margin = (CP - CG) / d, in calibers. Thresholds (see `assess`):
  < 1.0 caliber : UNDER-STABLE warning. The classic hobby rule of thumb; the
                  Barrowman CP moves FORWARD at angle of attack (body lift,
                  which this method neglects: about 1 caliber at 8 deg for a
                  slender rocket in Nakka's example), so a 1-caliber margin is
                  the minimum that survives a gusty rail exit.
  > 3.0 calibers: OVER-STABLE warning, checked AT LIFTOFF only. Margin far
                  beyond need makes the rocket weathercock strongly into a
                  crosswind; that happens where the angle of attack (~ wind /
                  airspeed) is large, i.e. at low rail-exit speed. At burnout
                  the rocket flies at hundreds of m/s and the angle of attack
                  is tiny, so a high burnout margin is harmless and, with a
                  heavy motor, unavoidable: burning the propellant out of the
                  tail moves the CG forward by 2+ calibers on a high-power
                  rocket. The burnout margin is still checked for UNDER-
                  stability (the lower of the two is the dangerous one).
  (Two to three calibers is typical for hobby rockets; minimum-diameter
  high-power rockets are often designed nearer 1.5-2.)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

NOSE_K = {"cone": 2.0 / 3.0, "ogive": 0.466, "parabolic": 0.5}
UNDER_STABLE_CAL = 1.0
OVER_STABLE_CAL = 3.0


@dataclass
class Component:
    kind: str          # nose | tube | transition | fins | mass
    x_fore: float      # m from nose tip
    length: float
    mass: float
    cg: float          # m from nose tip
    cna: float = 0.0   # normal force coefficient slope (per rad)
    xcp: float = 0.0   # component CP, m from nose tip
    name: str = ""


@dataclass
class Geometry:
    components: list
    length_m: float
    reference_diameter_m: float
    max_diameter_m: float
    aft_diameter_m: float
    motor_aft_offset_m: float = 0.0
    x0: float = 0.0          # where this stage starts (0 for the nose stage)

    @property
    def dry_mass_kg(self) -> float:
        return sum(c.mass for c in self.components)


@dataclass
class StabilityReport:
    cp_m: float
    cg_liftoff_m: float
    cg_burnout_m: float
    margin_liftoff_cal: float
    margin_burnout_cal: float
    reference_diameter_m: float
    components: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    @property
    def stable(self) -> bool:
        return min(self.margin_liftoff_cal, self.margin_burnout_cal) >= UNDER_STABLE_CAL


def _pos(d: dict, key: str, where: str) -> float:
    if key not in d:
        raise ValueError(f"{where}: missing {key!r}")
    v = d[key]
    if not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v) or v <= 0:
        raise ValueError(f"{where}: {key} must be a positive number (got {v!r})")
    return float(v)


def _nonneg(d: dict, key: str, where: str, default: float = 0.0) -> float:
    v = d.get(key, default)
    if not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v) or v < 0:
        raise ValueError(f"{where}: {key} must be >= 0 (got {v!r})")
    return float(v)


def build_geometry(g: dict, *, x0: float = 0.0, reference_diameter_m: float | None = None,
                   require_nose: bool = True) -> Geometry:
    """Validate a geometry dict and lay out its components from position x0."""
    allowed = {"nose", "body", "fins", "masses", "motor_aft_offset_m"}
    unknown = set(g) - allowed
    if unknown:
        raise ValueError(f"geometry: unknown keys {sorted(unknown)}")
    comps: list[Component] = []
    x = x0
    d_prev = None
    if "nose" in g:
        n = g["nose"]
        shape = n.get("shape")
        if shape not in NOSE_K:
            raise ValueError(f"geometry.nose: shape must be one of {sorted(NOSE_K)} (got {shape!r})")
        L, d = _pos(n, "length_m", "geometry.nose"), _pos(n, "diameter_m", "geometry.nose")
        if L < 0.5 * d:
            raise ValueError(f"geometry.nose: length {L} m is shorter than half its diameter: not a nose cone")
        m = _nonneg(n, "mass_kg", "geometry.nose")
        # Solid of revolution CG: cone at 2/3 L, ogive/parabolic close to that (~0.6 L).
        comps.append(Component("nose", x, L, m, x + (2.0 / 3.0 if shape == "cone" else 0.6) * L,
                               cna=2.0, xcp=x + NOSE_K[shape] * L, name=f"{shape} nose"))
        x += L
        d_prev = d
    elif require_nose:
        raise ValueError("geometry: missing 'nose'")
    d_ref = reference_diameter_m if reference_diameter_m is not None else d_prev
    if d_ref is None:
        raise ValueError("geometry: a stage without a nose needs the reference diameter of the stack")

    body = g.get("body")
    if not body:
        raise ValueError("geometry: 'body' must list at least one tube or transition")
    segs = []        # (x_fore, x_aft, d_fore, d_aft)
    for i, b in enumerate(body):
        where = f"geometry.body[{i}]"
        typ = b.get("type")
        L, m = _pos(b, "length_m", where), _nonneg(b, "mass_kg", where)
        if typ == "tube":
            d = _pos(b, "diameter_m", where)
            df = da = d
            comps.append(Component("tube", x, L, m, x + L / 2, name=f"tube {i}"))
        elif typ == "transition":
            df, da = _pos(b, "fore_diameter_m", where), _pos(b, "aft_diameter_m", where)
            r = df / da
            cna = 2.0 * ((da / d_ref) ** 2 - (df / d_ref) ** 2)
            xcp = x + (L / 3.0) * (1.0 + (1.0 - r) / (1.0 - r * r)) if abs(1.0 - r) > 1e-12 else x + L / 2
            # Frustum shell CG (thin wall): from the fore end, L (df + 2 da) / (3 (df + da)).
            cg = x + L * (df + 2 * da) / (3 * (df + da))
            comps.append(Component("transition", x, L, m, cg, cna=cna, xcp=xcp, name=f"transition {i}"))
        else:
            raise ValueError(f"{where}: type must be 'tube' or 'transition' (got {typ!r})")
        if d_prev is not None and abs(df - d_prev) > 0.01 * d_prev:
            raise ValueError(f"{where}: fore diameter {df} m does not match the {d_prev} m diameter ahead of it "
                             "(use a transition to change diameter)")
        segs.append((x, x + L, df, da))
        d_prev = da
        x += L
    length = x

    def radius_at(xq: float) -> float:
        for xf, xa, df, da in segs:
            if xf - 1e-12 <= xq <= xa + 1e-12:
                return 0.5 * (df + (da - df) * (xq - xf) / (xa - xf))
        raise ValueError(f"geometry: position {xq:.3f} m is not on the body")

    for i, f in enumerate(g.get("fins", [])):
        where = f"geometry.fins[{i}]"
        n = f.get("count")
        if n not in (3, 4):
            raise ValueError(f"{where}: count must be 3 or 4 (the Barrowman fin formula used here is for 3 or 4 fins)")
        cr, ct = _pos(f, "root_chord_m", where), _nonneg(f, "tip_chord_m", where)
        s, xr = _pos(f, "semispan_m", where), _nonneg(f, "sweep_m", where)
        from_tail = _nonneg(f, "root_le_from_tail_m", where)
        m = _nonneg(f, "mass_kg", where)
        if ct > cr * 2:
            raise ValueError(f"{where}: tip chord more than twice the root chord is not a sensible fin")
        xb = length - from_tail
        if xb < x0 or xb + cr > length + 1e-9:
            raise ValueError(f"{where}: fin root ({cr} m starting {from_tail} m from the tail) must lie on the body")
        R = radius_at(xb + 0.5 * cr)
        lm = math.sqrt(s * s + (xr + 0.5 * ct - 0.5 * cr) ** 2)
        cna = (1.0 + R / (s + R)) * (4.0 * n * (s / d_ref) ** 2) / (1.0 + math.sqrt(1.0 + (2.0 * lm / (cr + ct)) ** 2))
        xcp = xb + xr * (cr + 2 * ct) / (3 * (cr + ct)) + (cr + ct - cr * ct / (cr + ct)) / 6.0
        # Trapezoid area centroid (fins are flat plates) along the body axis.
        area = 0.5 * (cr + ct) * s
        cgx = xb + (xr * (cr + 2 * ct) + (cr * cr + cr * ct + ct * ct)) / (3 * (cr + ct)) if area > 0 else xb + cr / 2
        comps.append(Component("fins", xb, cr, m, cgx, cna=cna, xcp=xcp, name=f"{n} fins"))

    for i, pm in enumerate(g.get("masses", [])):
        where = f"geometry.masses[{i}]"
        m = _pos(pm, "mass_kg", where)
        xp = _nonneg(pm, "position_m", where)
        if not 0.0 <= xp <= length - x0 + 1e-9:
            raise ValueError(f"{where}: position {xp} m is outside the {length - x0:.3f} m long stage")
        comps.append(Component("mass", x0 + xp, 0.0, m, x0 + xp, name=pm.get("name", f"mass {i}")))

    off = _nonneg(g, "motor_aft_offset_m", "geometry")
    return Geometry(comps, length, d_ref, max(max(s_[2], s_[3]) for s_ in segs), segs[-1][3], off, x0)


def center_of_pressure(components) -> float:
    total = sum(c.cna for c in components)
    if total <= 0:
        raise ValueError("no positive normal-force slope: the rocket has no stabilising surfaces")
    return sum(c.cna * c.xcp for c in components) / total


def _motor_cg(geom: Geometry, motor) -> float:
    length = motor.length_m if getattr(motor, "length_m", 0.0) else 0.0
    if length <= 0.0:
        raise ValueError(f"motor {getattr(motor, 'name', '')} has no length in its .eng header")
    if length + geom.motor_aft_offset_m > geom.length_m - geom.x0:
        raise ValueError(f"motor ({length:.3f} m) is longer than the {geom.length_m - geom.x0:.3f} m airframe")
    if motor.diameter_m >= geom.aft_diameter_m:
        raise ValueError(f"motor diameter {motor.diameter_m * 1000:.0f} mm does not fit the "
                         f"{geom.aft_diameter_m * 1000:.0f} mm aft body")
    return geom.length_m - geom.motor_aft_offset_m - length / 2.0


def center_of_gravity(stages) -> tuple[float, float]:
    """CG at liftoff and at burnout of all motors, for [(geometry, motor), ...]."""
    m_l = m_b = s_l = s_b = 0.0
    for geom, motor in stages:
        for c in geom.components:
            m_l += c.mass
            s_l += c.mass * c.cg
        xm = _motor_cg(geom, motor)
        casing = motor.total_mass - motor.propellant_mass
        m_l += motor.total_mass
        s_l += motor.total_mass * xm
        m_b += casing
        s_b += casing * xm
    dry = sum(c.mass for geom, _ in stages for c in geom.components)
    s_dry = sum(c.mass * c.cg for geom, _ in stages for c in geom.components)
    return s_l / m_l, (s_dry + s_b) / (dry + m_b)


def assess(stages) -> StabilityReport:
    """Stability of the vehicle formed by [(geometry, motor), ...] (nose stage first)."""
    comps = [c for geom, _ in stages for c in geom.components]
    d = stages[0][0].reference_diameter_m
    cp = center_of_pressure(comps)
    cg_l, cg_b = center_of_gravity(stages)
    ml, mb = (cp - cg_l) / d, (cp - cg_b) / d
    warnings = []
    for label, m in (("liftoff", ml), ("burnout", mb)):
        if m < UNDER_STABLE_CAL:
            warnings.append(f"UNDER-STABLE at {label}: static margin {m:.2f} cal < {UNDER_STABLE_CAL:.1f} "
                            "(enlarge or move fins aft, or add nose weight)")
    if ml > OVER_STABLE_CAL:
        warnings.append(f"OVER-STABLE at liftoff: static margin {ml:.2f} cal > {OVER_STABLE_CAL:.1f} "
                        "(strong weathercocking in crosswind off the rail; consider smaller fins)")
    return StabilityReport(cp, cg_l, cg_b, ml, mb, d, comps, warnings)
