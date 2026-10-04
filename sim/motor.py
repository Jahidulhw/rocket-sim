"""Rocket motors.

A motor exposes a tiny interface used by the dynamics:
    thrust(t)       -> N
    mass_flow(t)    -> kg/s (positive number; mass decreases)
    burn_time       -> s, thrust is zero for t >= burn_time
    total_mass      -> kg, loaded motor mass at ignition
    propellant_mass -> kg
    breakpoints()   -> times where thrust has a kink/jump (the run loop
                       steps exactly onto these so RK4 stays accurate)
"""

from __future__ import annotations

from bisect import bisect_right

import numpy as np

from .config import MotorConfig, resolve_path


class ConstantThrustMotor:
    """Idealised motor: constant thrust for a fixed burn time."""

    def __init__(self, thrust_n: float, burn_time_s: float, total_mass_kg: float = 0.0,
                 propellant_mass_kg: float = 0.0, name: str = "constant"):
        self.name = name
        self.thrust_n = float(thrust_n)
        self.burn_time = float(burn_time_s)
        self.total_mass = float(total_mass_kg)
        self.propellant_mass = float(propellant_mass_kg)
        self.total_impulse = self.thrust_n * self.burn_time
        self.ejection_delay_s = None

    def thrust(self, t: float) -> float:
        return self.thrust_n if 0.0 <= t < self.burn_time else 0.0

    def mass_flow(self, t: float) -> float:
        # dm/dt proportional to thrust: propellant runs out exactly at burnout.
        if self.total_impulse <= 0.0:
            return 0.0
        return self.propellant_mass * self.thrust(t) / self.total_impulse

    def breakpoints(self) -> list[float]:
        return [self.burn_time] if self.burn_time > 0 else []


class ThrustCurveMotor:
    """Motor defined by a tabulated thrust curve (linear interpolation).

    `impulse_scale` multiplies thrust (and so total impulse) without changing
    burn time or propellant mass; Monte Carlo uses it for motor-to-motor
    variation. Mass flow stays proportional to thrust, so the propellant is
    still exactly used up at burnout.
    """

    def __init__(self, times, thrusts, propellant_mass_kg: float, total_mass_kg: float,
                 name: str = "", manufacturer: str = "", diameter_m: float = 0.0,
                 length_m: float = 0.0, delays_s=None, impulse_scale: float = 1.0):
        times = np.asarray(times, dtype=float)
        thrusts = np.asarray(thrusts, dtype=float)
        if times.ndim != 1 or times.size < 2 or times.shape != thrusts.shape:
            raise ValueError("thrust curve needs at least two (time, thrust) points")
        if np.any(np.diff(times) <= 0.0):
            raise ValueError("thrust curve times must be strictly increasing")
        if np.any(thrusts < 0.0) or times[0] < 0.0:
            raise ValueError("thrust curve has negative time or thrust")
        if times[0] > 0.0:
            # RASP convention: the curve implicitly starts at (0, 0).
            times = np.concatenate(([0.0], times))
            thrusts = np.concatenate(([0.0], thrusts))
        self.times = times
        self.thrusts = thrusts * impulse_scale
        self.impulse_scale = float(impulse_scale)
        self.burn_time = float(times[-1])
        self.propellant_mass = float(propellant_mass_kg)
        self.total_mass = float(total_mass_kg)
        self.name = name
        self.manufacturer = manufacturer
        self.diameter_m = diameter_m
        self.length_m = length_m
        self.delays_s = list(delays_s or [])
        self.ejection_delay_s = None
        # Exact integral of the piecewise-linear curve.
        self.total_impulse = float(np.sum(0.5 * (self.thrusts[1:] + self.thrusts[:-1])
                                          * np.diff(self.times)))
        self._t0, self._t1 = float(times[0]), self.burn_time
        # Plain Python lists make scalar interpolation cheaper than np.interp.
        self._tl, self._fl = self.times.tolist(), self.thrusts.tolist()

    def thrust(self, t: float) -> float:
        if t < self._t0 or t >= self._t1:
            return 0.0
        tl, fl = self._tl, self._fl
        i = bisect_right(tl, t) - 1
        w = (t - tl[i]) / (tl[i + 1] - tl[i])
        return fl[i] + w * (fl[i + 1] - fl[i])

    def mass_flow(self, t: float) -> float:
        if self.total_impulse <= 0.0:
            return 0.0
        return self.propellant_mass * self.thrust(t) / self.total_impulse

    def breakpoints(self) -> list[float]:
        # Every curve node is a kink in thrust; the last one is burnout.
        return [float(x) for x in self.times[1:]]

    def info(self) -> dict:
        return {"name": self.name, "manufacturer": self.manufacturer,
                "diameter_m": self.diameter_m, "length_m": self.length_m,
                "delays_s": self.delays_s, "propellant_mass_kg": self.propellant_mass,
                "total_mass_kg": self.total_mass, "total_impulse_ns": self.total_impulse,
                "burn_time_s": self.burn_time, "impulse_scale": self.impulse_scale,
                "ejection_delay_s": self.ejection_delay_s,
                "curve": {"t": self.times.tolist(), "thrust_n": self.thrusts.tolist()}}


class DelayedMotor:
    """A motor whose ignition is delayed to absolute time `ignition_time`.

    Passive staging: the sustainer motor lights at a fixed, pre-set delay after
    booster burnout (like a hobby booster burning through into the sustainer,
    or a pre-set timer), never on a flight-computer command. Times are
    shifted; burn_time is the ABSOLUTE burnout time, so ejection delays and
    the run loop's burnout logic work unchanged.
    """

    def __init__(self, motor, ignition_time: float):
        self.inner = motor
        self.ignition_time = float(ignition_time)
        self.name = getattr(motor, "name", "")
        self.burn_time = self.ignition_time + motor.burn_time
        self.total_mass = motor.total_mass
        self.propellant_mass = motor.propellant_mass
        self.total_impulse = motor.total_impulse
        self.ejection_delay_s = getattr(motor, "ejection_delay_s", None)

    def thrust(self, t: float) -> float:
        return self.inner.thrust(t - self.ignition_time)

    def mass_flow(self, t: float) -> float:
        return self.inner.mass_flow(t - self.ignition_time)

    def breakpoints(self) -> list[float]:
        return [self.ignition_time] + [self.ignition_time + b for b in self.inner.breakpoints()]

    def info(self) -> dict:
        d = self.inner.info() if hasattr(self.inner, "info") else {"name": self.name}
        return {**d, "ignition_time_s": self.ignition_time}


def _parse_delays(token: str) -> list[float]:
    """'0-3-5-7' -> [0, 3, 5, 7]; 'P' (plugged, no ejection charge) -> []."""
    if token.upper().startswith("P"):
        return []
    return [float(x) for x in token.split("-") if x]


def load_eng(path, impulse_scale: float = 1.0) -> ThrustCurveMotor:
    """Load the first motor from a RASP .eng file.

    Format: ';' starts a comment. The first non-comment line is the header
        name  diameter_mm  length_mm  delays  propellant_kg  total_kg  manufacturer
    followed by 'time thrust' pairs, terminated by a zero-thrust point.
    """
    header = None
    times, thrusts = [], []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.split(";", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            if header is None:
                if len(parts) < 7:
                    raise ValueError(f"{path}: malformed .eng header: {line!r}")
                header = parts
                continue
            if len(parts) != 2:
                break  # next motor's header (multi-motor file): stop at the first motor
            try:
                t, f = float(parts[0]), float(parts[1])
            except ValueError as exc:
                raise ValueError(f"{path}: bad data line {line!r}") from exc
            times.append(t)
            thrusts.append(f)
            if f == 0.0 and t > 0.0:
                break
    if header is None:
        raise ValueError(f"{path}: no .eng header found")
    if len(times) < 2:
        raise ValueError(f"{path}: thrust curve has fewer than two points")
    try:
        name, dia, length, delays, prop, total = header[0], *header[1:6]
        return ThrustCurveMotor(times, thrusts, float(prop), float(total), name=name,
                                manufacturer=" ".join(header[6:]),
                                diameter_m=float(dia) / 1000.0, length_m=float(length) / 1000.0,
                                delays_s=_parse_delays(delays), impulse_scale=impulse_scale)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path}: {exc}") from exc


def build_motor(cfg: MotorConfig):
    if cfg.kind == "constant":
        return ConstantThrustMotor(cfg.thrust_n, cfg.burn_time_s, cfg.total_mass_kg,
                                   cfg.propellant_mass_kg)
    if cfg.kind == "eng":
        motor = load_eng(resolve_path(cfg.eng_file), impulse_scale=cfg.impulse_scale)
        motor.ejection_delay_s = cfg.ejection_delay_s
        return motor
    raise ValueError(f"unknown motor kind {cfg.kind!r}")
