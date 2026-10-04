"""International Standard Atmosphere (ISA) / U.S. Standard Atmosphere 1976.

Temperature is piecewise linear in altitude; within each layer the
hydrostatic equation integrates in closed form:

    layer base H_b, base temperature T_b, base pressure P_b, lapse L_b (K/m)
    T(h) = T_b + L_b (h - H_b)
    L_b != 0:  P(h) = P_b (T / T_b) ** (-g / (R L_b))
    L_b == 0:  P(h) = P_b exp(-g (h - H_b) / (R T_b))
    rho = P / (R T),   speed of sound a = sqrt(gamma R T)

Layers (base altitude, lapse), to the 1976 model's 84.852 km top:

    0 km       -6.5 K/km   troposphere
    11 km       0          tropopause
    20 km      +1.0        stratosphere
    32 km      +2.8        stratosphere
    47 km       0          stratopause
    51 km      -2.8        mesosphere
    71 km      -2.0        mesosphere

ALTITUDE CONVENTION. The standard defines the layers in GEOPOTENTIAL
altitude, which exists because gravity weakens with height. This simulator
uses constant gravity on a flat Earth, and in that world geopotential and
geometric altitude are the same thing, so the layers are applied directly
to the simulator's altitude. Consistent with the physics; versus the real
Earth it places each layer slightly low (by ~19 m at 11 km, ~160 m at 32 km,
i.e. ~2 % in density at 32 km). Hobby rockets never notice; the sounding
rocket preset (~10 km) sees ~0.2 %.

REGRESSION. The troposphere and 11-20 km density expressions are kept
verbatim from the pre-Phase-A model (normalised to rho0 = 1.225 exactly), so
the default rocket's flights are bit-for-bit unchanged. Higher layers chain
on from the 20 km density using exact layer ratios. `rho0` can be overridden
(hot/high launch site): it scales density only, keeping the profile shape.

Above the model top (84.852 km) a ValueError is raised rather than
extrapolating silently; no preset comes within a factor of 5 of it.
"""

from __future__ import annotations

import math

T0 = 288.15          # K, ISA sea-level temperature
P0 = 101325.0        # Pa, ISA sea-level pressure
RHO0 = 1.225         # kg/m^3, ISA sea-level density
LAPSE = 0.0065       # K/m, troposphere lapse rate
R_AIR = 287.05287    # J/(kg K), specific gas constant of dry air
G_ISA = 9.80665      # m/s^2, standard gravity used to define ISA
GAMMA = 1.4          # ratio of specific heats, dry air
H_TROPO = 11_000.0   # m
H_TOP = 84_852.0     # m, top of the 1976 model's 7th layer

# (base altitude m, lapse K/m)
_LAYERS = [(0.0, -0.0065), (11_000.0, 0.0), (20_000.0, 0.001), (32_000.0, 0.0028),
           (47_000.0, 0.0), (51_000.0, -0.0028), (71_000.0, -0.002)]

# Legacy troposphere / lower-stratosphere constants (verbatim, see REGRESSION).
_EXP = G_ISA / (R_AIR * LAPSE) - 1.0                     # ~4.2559
_T11 = T0 - LAPSE * H_TROPO
_RATIO11 = (_T11 / T0) ** _EXP


def _build_layers():
    """Base temperature, pressure ratio P_b/P0 and density ratio rho_b/RHO0 per layer."""
    out = []
    T_b, p_b = T0, 1.0
    for i, (h_b, lapse) in enumerate(_LAYERS):
        out.append((h_b, lapse, T_b, p_b))
        if i + 1 < len(_LAYERS):
            h_n = _LAYERS[i + 1][0]
            T_n = T_b + lapse * (h_n - h_b)
            p_b = p_b * ((T_n / T_b) ** (-G_ISA / (R_AIR * lapse)) if lapse != 0.0
                         else math.exp(-G_ISA * (h_n - h_b) / (R_AIR * T_b)))
            T_b = T_n
    return out


_BASES = _build_layers()
# Density ratio at 20 km from the verbatim legacy expression; layers above
# chain on from it, so density is continuous at 20 km by construction.
_SIGMA20 = _RATIO11 * math.exp(-G_ISA * (20_000.0 - H_TROPO) / (R_AIR * _T11))


def _layer(h: float):
    if h > H_TOP:
        raise ValueError(f"altitude {h:.0f} m is above the atmosphere model top ({H_TOP:.0f} m)")
    for base in reversed(_BASES):
        if h >= base[0]:
            return base
    return _BASES[0]     # below sea level: troposphere formula extends downward


def temperature(h: float) -> float:
    """Air temperature (K) at altitude h (m)."""
    h_b, lapse, T_b, _ = _layer(h)
    return T_b + lapse * (h - h_b)


def pressure(h: float) -> float:
    """Static pressure (Pa) at altitude h (m)."""
    h_b, lapse, T_b, p_b = _layer(h)
    T = T_b + lapse * (h - h_b)
    if lapse != 0.0:
        return P0 * p_b * (T / T_b) ** (-G_ISA / (R_AIR * lapse))
    return P0 * p_b * math.exp(-G_ISA * (h - h_b) / (R_AIR * T_b))


def speed_of_sound(h: float) -> float:
    """Speed of sound (m/s) at altitude h (m)."""
    return math.sqrt(GAMMA * R_AIR * temperature(h))


def density(h: float, rho0: float = RHO0) -> float:
    """Air density (kg/m^3) at altitude h (m) above mean sea level."""
    if h <= H_TROPO:
        return rho0 * ((T0 - LAPSE * h) / T0) ** _EXP
    if h <= 20_000.0:
        return rho0 * _RATIO11 * math.exp(-G_ISA * (h - H_TROPO) / (R_AIR * _T11))
    # rho/rho20 = (P/P20) * (T20/T): exact within the chained layers.
    _, _, T20, p20 = _BASES[2]
    return rho0 * _SIGMA20 * (pressure(h) / (P0 * p20)) * (T20 / temperature(h))
