"""Flight run loop, flight phases and event detection.

The continuous state y = [p, v, m] is advanced by the chosen integrator.
Discrete mode flags (on pad / on rail / chute out) only change *between*
steps, at events.

Event timing:
  * events at a known time (thrust-curve nodes, burnout, parachute
    deployment) are hit exactly by shortening the step that would cross them;
  * state-triggered events (liftoff, rail exit, apogee, ground contact) are
    located inside the step where the sign change happened: bisection for
    liftoff, Newton for rail exit, linear interpolation for apogee/landing.

Phases (in order): PAD, RAIL, BOOST, COAST, DESCENT, LANDED.
APOGEE is an *event* marking the COAST -> DESCENT transition. A phase can be
legitimately absent (e.g. zero rail length has no RAIL; a projectile launched
with initial speed has no PAD/BOOST), but phases never repeat or go backwards.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .config import FlightConfig
from .integrator import INTEGRATORS
from .motor import build_motor
from .physics import Dynamics

PHASES = ["PAD", "RAIL", "BOOST", "COAST", "DESCENT", "LANDED"]
EVENT_ORDER = ["liftoff", "rail_exit", "burnout", "deploy", "apogee", "landing"]
_T_EPS = 1e-12


@dataclass
class Event:
    name: str
    t: float
    position: np.ndarray
    velocity: np.ndarray

    def to_dict(self) -> dict:
        return {"name": self.name, "t": float(self.t),
                "position": [float(x) for x in self.position],
                "velocity": [float(x) for x in self.velocity]}


@dataclass
class FlightResult:
    config: FlightConfig
    motor: object
    t: np.ndarray            # (N,)
    position: np.ndarray     # (N, 3)
    velocity: np.ndarray     # (N, 3)
    mass: np.ndarray         # (N,)
    phase: list              # (N,) phase name per sample
    events: dict             # name -> Event
    landed: bool
    deployment: dict = field(default_factory=dict)

    @property
    def apogee_m(self) -> float:
        if "apogee" in self.events:
            return float(self.events["apogee"].position[2])
        return float(np.max(self.position[:, 2]))

    @property
    def landing_point(self) -> np.ndarray:
        return self.position[-1]

    @property
    def landing_distance_m(self) -> float:
        return float(np.hypot(*self.landing_point[:2]))

    @property
    def flight_time_s(self) -> float:
        return float(self.t[-1])

    @property
    def max_speed_mps(self) -> float:
        return float(np.max(np.linalg.norm(self.velocity, axis=1)))

    def summary(self) -> dict:
        return {"apogee_m": self.apogee_m, "max_speed_mps": self.max_speed_mps,
                "flight_time_s": self.flight_time_s, "landed": self.landed,
                "landing_point_m": [float(x) for x in self.landing_point],
                "landing_distance_m": self.landing_distance_m,
                "deployment": self.deployment}


def _event(name, t, y) -> Event:
    return Event(name, float(t), y[0:3].copy(), y[3:6].copy())


def _left_limited(dyn: Dynamics, t_end: float, on_rail: bool, chute: bool):
    """Derivative function for one step ending at t_end.

    Stages evaluated at the step's end use the LEFT limit of time, so a step
    that ends exactly on a thrust discontinuity (burnout) never sees the
    post-jump value. Without this, RK4's k4 at t = burn_time gets zero thrust
    and the step loses ~1/6 of its impulse: an O(dt) error.
    """
    t_left = math.nextafter(t_end, -math.inf)
    return lambda tt, yy: dyn.derivatives(min(tt, t_left), yy, on_rail, chute)


def resolve_deploy_delay(cfg: FlightConfig, motor) -> float | None:
    if not cfg.recovery.enabled:
        return None
    if cfg.recovery.deploy_delay_s is not None:
        return cfg.recovery.deploy_delay_s
    return getattr(motor, "ejection_delay_s", None)


def simulate(cfg: FlightConfig, motor=None) -> FlightResult:
    motor = motor if motor is not None else build_motor(cfg.motor)
    dyn = Dynamics(cfg, motor)
    step = INTEGRATORS[cfg.sim.integrator]
    dt = cfg.sim.dt_s
    u = dyn.rail_dir
    rail_len = cfg.launch.rail_length_m

    y = np.zeros(7)
    y[3:6] = cfg.launch.initial_speed_mps * u
    y[6] = cfg.rocket.dry_mass_kg + motor.total_mass
    t = 0.0

    delay = resolve_deploy_delay(cfg, motor)
    t_deploy = motor.burn_time + delay if delay is not None else None
    true_apogee = None
    breakpoints = set(motor.breakpoints())
    if t_deploy is not None:
        breakpoints.add(t_deploy)
    breakpoints = sorted(b for b in breakpoints if b > 0.0)

    events: dict[str, Event] = {}
    on_pad = True
    on_rail = rail_len > 0.0
    burned_out = motor.burn_time <= 0.0
    apogee_passed = False
    chute = False
    landed = False

    def phase() -> str:
        if landed:
            return "LANDED"
        if on_pad:
            return "PAD"
        if on_rail:
            return "RAIL"
        if not burned_out:
            return "BOOST"
        return "DESCENT" if apogee_passed else "COAST"

    def pad_derivatives(tt):
        dy = np.zeros(7)
        dy[6] = -motor.mass_flow(tt)
        return dy

    if cfg.launch.initial_speed_mps > 0.0:
        on_pad = False
        events["liftoff"] = _event("liftoff", 0.0, y)

    ts, ys, phases = [t], [y.copy()], [phase()]

    def record(t_, y_):
        ts.append(t_)
        ys.append(y_.copy())
        phases.append(phase())

    while t < cfg.sim.max_time_s:
        # ---- step size: never step across a known discontinuity -------------
        h_nom = min(dt, dyn.max_stable_step(y, chute))
        h, t_new = h_nom, t + h_nom
        for bp in breakpoints:
            if bp > t + _T_EPS:
                if t_new >= bp - 1e-9 * h_nom:
                    h, t_new = bp - t, bp
                break

        if on_pad:
            # ---- held on the pad until thrust beats weight along the rail ----
            if dyn.rail_acceleration(t, y) > 0.0:
                on_pad = False
                events["liftoff"] = _event("liftoff", t, y)
            elif burned_out:
                break  # motor finished and we never moved: no flight
            else:
                # No motion, but the motor is already burning propellant, so
                # mass must still be integrated.
                t_left = math.nextafter(t_new, -math.inf)
                f_pad = (lambda tt, yy: pad_derivatives(min(tt, t_left)))
                y_end = step(f_pad, t, y, h)
                if dyn.rail_acceleration(t_left, y_end) > 0.0:
                    # Thrust overtakes weight inside this step: bisect for the
                    # exact liftoff time instead of snapping to the dt grid.
                    lo, hi = 0.0, h
                    for _ in range(50):
                        mid = 0.5 * (lo + hi)
                        if dyn.rail_acceleration(t + mid, step(f_pad, t, y, mid)) > 0.0:
                            hi = mid
                        else:
                            lo = mid
                    t_new = t + hi
                    y_end = step(f_pad, t, y, hi)
                    on_pad = False
                    events["liftoff"] = _event("liftoff", t_new, y_end)
                y, t = y_end, t_new
                if not burned_out and t >= motor.burn_time - _T_EPS:
                    burned_out = True
                    events["burnout"] = _event("burnout", t, y)
                record(t, y)
                continue

        f = _left_limited(dyn, t_new, on_rail, chute)
        y_new = step(f, t, y, h)

        if on_rail:
            s_old = float(y[0:3] @ u)
            s_new = float(y_new[0:3] @ u)
            if s_new < 0.0:
                y_new[0:6] = 0.0  # the rail stops the rocket sliding below the pad
            elif s_new >= rail_len:
                # The dynamics change at rail exit, so split the step there:
                # on-rail up to the exit time, free flight for the remainder.
                # (Integrating the overshoot with the rail constraint still on
                # is an O(dt) error.) Exit time: linear guess, then Newton on
                # s(h1) - L = 0 with ds/dt = v.u; runs once per flight.
                h1 = h * (rail_len - s_old) / (s_new - s_old)
                y_exit = y.copy()
                for _ in range(5):
                    y_exit = step(f, t, y, h1) if h1 > 0.0 else y.copy()
                    err = float(y_exit[0:3] @ u) - rail_len
                    speed = float(y_exit[3:6] @ u)
                    if abs(err) < 1e-12 or speed <= 0.0:
                        break
                    h1 = min(max(h1 - err / speed, 0.0), h)
                t_exit = t + h1
                on_rail = False
                events["rail_exit"] = _event("rail_exit", t_exit, y_exit)
                record(t_exit, y_exit)
                if t_new > t_exit:
                    y_new = step(_left_limited(dyn, t_new, False, chute), t_exit, y_exit, t_new - t_exit)
                else:
                    y_new = y_exit
        else:
            if not apogee_passed and y[5] > 0.0 >= y_new[5]:
                frac = y[5] / (y[5] - y_new[5])
                events["apogee"] = _event("apogee", t + frac * h, y + frac * (y_new - y))
                apogee_passed = True
            if y_new[2] < 0.0:
                frac = y[2] / (y[2] - y_new[2])
                t_land = t + frac * h
                y_land = y + frac * (y_new - y)
                y_land[2] = 0.0
                events["landing"] = _event("landing", t_land, y_land)
                landed = True
                record(t_land, y_land)
                break

        t, y = t_new, y_new
        if not burned_out and t >= motor.burn_time - _T_EPS:
            burned_out = True
            events["burnout"] = _event("burnout", t, y)
        if t_deploy is not None and not chute and t >= t_deploy - _T_EPS:
            chute = True
            events["deploy"] = _event("deploy", t, y)
            if y[5] > 0.0:
                # Still climbing: the chute will cause an early "apogee". To
                # judge deploy timing we need the TRUE apogee, i.e. where the
                # rocket would have peaked without the chute. Integrate a
                # chute-free shadow copy of the coast until vz = 0.
                true_apogee = _coast_to_apogee(dyn, step, dt, t, y)
        record(t, y)

    arr = np.array(ys)
    return FlightResult(config=cfg, motor=motor, t=np.array(ts), position=arr[:, 0:3],
                        velocity=arr[:, 3:6], mass=arr[:, 6], phase=phases, events=events,
                        landed=landed,
                        deployment=_deployment_summary(cfg, events, true_apogee))


def _coast_to_apogee(dyn: Dynamics, step, dt: float, t: float, y: np.ndarray,
                     max_time: float = 600.0) -> Event:
    """Free-flight, chute-free integration from (t, y) until vz crosses zero."""
    t_end = t + max_time
    f = lambda tt, yy: dyn.derivatives(tt, yy, False, False)
    while t < t_end:
        y_new = step(f, t, y, dt)
        if y_new[5] <= 0.0:
            frac = y[5] / (y[5] - y_new[5])
            return _event("true_apogee", t + frac * dt, y + frac * (y_new - y))
        t, y = t + dt, y_new
    raise RuntimeError("shadow coast never reached apogee")


def _deployment_summary(cfg: FlightConfig, events: dict, true_apogee: Event | None) -> dict:
    """Was the chute out before, near, or after TRUE apogee, and how fast were we going?

    True apogee = where the rocket would have peaked without a parachute. If
    the chute opened after apogee, that is simply the observed apogee event;
    if it opened while climbing, it comes from the chute-free shadow coast.
    """
    if "deploy" not in events:
        return {"deployed": False}
    dep = events["deploy"]
    if true_apogee is None:
        true_apogee = events.get("apogee")
    out = {"deployed": True, "t": dep.t, "altitude_m": float(dep.position[2]),
           "speed_mps": float(np.linalg.norm(dep.velocity))}
    if true_apogee is None:  # cannot happen for a flight that lands; kept for safety
        out.update(true_apogee_t=None, true_apogee_m=None, dt_from_apogee_s=None, timing="unknown")
        return out
    dt_ap = dep.t - true_apogee.t
    window = cfg.recovery.near_apogee_window_s
    out.update(true_apogee_t=true_apogee.t, true_apogee_m=float(true_apogee.position[2]),
               dt_from_apogee_s=dt_ap,
               timing="near" if abs(dt_ap) <= window else ("before" if dt_ap < 0 else "after"))
    return out
