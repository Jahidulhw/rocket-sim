"""Mach-dependent drag coefficient for the rocket body.

The total drag coefficient of a slender finned rocket is nearly constant
at low subsonic speed, rises sharply through the transonic region as shock
waves form (wave drag), peaks just above Mach 1, and declines again at
supersonic speed. We model that with a per-rocket, C1-continuous empirical
curve scaled from the configured subsonic value cd0:

    M <= M_crit                 Cd = cd0                         subsonic plateau
    M_crit < M <= M_peak        cd0 -> k_peak * cd0   (smoothstep)  transonic rise
    M_peak < M <= M_sup         k_peak*cd0 -> k_sup*cd0 (smoothstep) supersonic decline
    M > M_sup                   Cd = k_sup * cd0

Default shape (approximate, read from the total-drag vs Mach curves for
typical sport/high-power rockets in S. Niskanen, "Development of an Open
Source model rocket simulation software" (OpenRocket technical
documentation, 2009, ch. 3), consistent with Barrowman-era wind-tunnel data):

    M_crit = 0.8    slender-body critical Mach: local flow first reaches Mach 1
    M_peak = 1.1    drag peak sits slightly supersonic
    k_peak = 1.9    peak total Cd ~ 1.6-2.2 x subsonic for finned bodies
    M_sup  = 2.0    end of the decline segment
    k_sup  = 1.35   ~1.3-1.5 x subsonic at Mach 2

Why smoothstep segments and not a physical wave-drag formula: Ackeret's
linear theory (~1/sqrt(M^2-1)) diverges at Mach 1, where the curve matters
most, and the real peak comes from geometry details we don't model (nose
shape, fin thickness, base drag). A smooth empirical fit with documented
parameters is the honest level of fidelity for a point-mass simulator, and
C1 continuity keeps the RK4 integrator at full order through Mach 1.

Assumptions: no Reynolds-number dependence, no power-on/power-off base drag
difference, Cd(M) independent of angle of attack (the rocket flies along
the velocity vector). Valid to ~Mach 3; the fleet stays below Mach 2.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MachDrag:
    mach_critical: float = 0.8
    mach_peak: float = 1.1
    peak_factor: float = 1.9
    mach_supersonic: float = 2.0
    supersonic_factor: float = 1.35

    def validate(self) -> None:
        if not 0.3 <= self.mach_critical < self.mach_peak < self.mach_supersonic <= 4.0:
            raise ValueError("Mach drag needs 0.3 <= mach_critical < mach_peak < mach_supersonic <= 4 "
                             "(the plateau must cover Mach 0.3 so slow rockets keep the constant-Cd model)")
        if not 1.0 <= self.supersonic_factor <= self.peak_factor:
            raise ValueError("Mach drag needs 1 <= supersonic_factor <= peak_factor")


def _smoothstep(x: float) -> float:
    """3x^2 - 2x^3 on [0, 1]: zero slope at both ends (C1 joints)."""
    return x * x * (3.0 - 2.0 * x)


def drag_coefficient(cd0: float, mach: float, p: MachDrag) -> float:
    """Body drag coefficient at Mach number `mach` for subsonic coefficient cd0."""
    if mach <= p.mach_critical:
        return cd0      # returned unchanged: exactly the constant-Cd model
    if mach <= p.mach_peak:
        s = _smoothstep((mach - p.mach_critical) / (p.mach_peak - p.mach_critical))
        return cd0 * (1.0 + (p.peak_factor - 1.0) * s)
    if mach <= p.mach_supersonic:
        s = _smoothstep((mach - p.mach_peak) / (p.mach_supersonic - p.mach_peak))
        return cd0 * (p.peak_factor + (p.supersonic_factor - p.peak_factor) * s)
    return cd0 * p.supersonic_factor
