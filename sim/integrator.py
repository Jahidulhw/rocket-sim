"""Fixed-step integrators.

State vector layout (shared with physics.py):
    y = [px, py, pz, vx, vy, vz, m]

`f(t, y)` returns dy/dt = [v, a, dm/dt].

RK4 (default): 4th-order accurate, four force evaluations per step.
Semi-implicit (symplectic) Euler: 1st-order, one evaluation per step. It
updates velocity first and then moves with the *new* velocity, which keeps
energy bounded for conservative forces instead of drifting like explicit
Euler. It's here as an independent cross-check on RK4.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

Deriv = Callable[[float, np.ndarray], np.ndarray]


def rk4_step(f: Deriv, t: float, y: np.ndarray, h: float) -> np.ndarray:
    k1 = f(t, y)
    k2 = f(t + 0.5 * h, y + 0.5 * h * k1)
    k3 = f(t + 0.5 * h, y + 0.5 * h * k2)
    k4 = f(t + h, y + h * k3)
    return y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def semi_implicit_euler_step(f: Deriv, t: float, y: np.ndarray, h: float) -> np.ndarray:
    dy = f(t, y)
    out = np.empty_like(y)
    out[3:6] = y[3:6] + h * dy[3:6]   # v_{n+1} = v_n + a_n h
    out[0:3] = y[0:3] + h * out[3:6]  # p_{n+1} = p_n + v_{n+1} h
    out[6] = y[6] + h * dy[6]
    return out


INTEGRATORS: dict[str, Callable] = {
    "rk4": rk4_step,
    "euler": semi_implicit_euler_step,
}
