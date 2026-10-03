"""International Standard Atmosphere (ISA) density.

Troposphere (0-11 km), linear temperature lapse:
    T(h)   = T0 - L*h
    rho(h) = rho0 * (T(h)/T0) ** (g/(R*L) - 1)
Lower stratosphere (11-20 km), isothermal:
    rho(h) = rho11 * exp(-g*(h - 11000)/(R*T11))

Hobby rockets stay far below 11 km; the stratosphere branch only keeps the
function well-defined. `rho0` can be overridden (e.g. hot/high launch site)
while keeping the same profile shape.
"""

from __future__ import annotations

import math

T0 = 288.15          # K, ISA sea-level temperature
RHO0 = 1.225         # kg/m^3, ISA sea-level density
LAPSE = 0.0065       # K/m, troposphere lapse rate
R_AIR = 287.05287    # J/(kg K), specific gas constant of dry air
G_ISA = 9.80665      # m/s^2, standard gravity used to define ISA
H_TROPO = 11_000.0   # m

_EXP = G_ISA / (R_AIR * LAPSE) - 1.0                     # ~4.2559
_T11 = T0 - LAPSE * H_TROPO
_RATIO11 = (_T11 / T0) ** _EXP


def density(h: float, rho0: float = RHO0) -> float:
    """Air density (kg/m^3) at geometric altitude h (m) above mean sea level."""
    if h <= H_TROPO:
        return rho0 * ((T0 - LAPSE * h) / T0) ** _EXP
    return rho0 * _RATIO11 * math.exp(-G_ISA * (h - H_TROPO) / (R_AIR * _T11))
