"""Forces and accelerations on a point-mass rocket.

Forces:
    thrust  T(t) along the rail (on the rail) or along the unit velocity
            vector (free flight, "gravity turn")
    gravity m * (0, 0, -g)
    drag    D = -1/2 * rho(z) * Cd * A * |v_rel| * v_rel,   v_rel = v - wind

On the rail the rocket is a bead on a wire: only the component of the net
force along the rail direction accelerates it (the rail supplies the normal
force).
"""

from __future__ import annotations

import math

import numpy as np

from .aero import drag_coefficient
from .atmosphere import density, speed_of_sound
from .config import FlightConfig, WindConfig, mach_drag_params

G0 = 9.81                                   # m/s^2, standard gravity (constant, flat Earth)
GRAVITY = np.array([0.0, 0.0, -G0])
_MIN_SPEED = 1e-9                           # below this, the velocity direction is undefined


def drag_force(v_rel: np.ndarray, rho: float, cd: float, area: float) -> np.ndarray:
    """Quadratic drag opposing the air-relative velocity."""
    return (-0.5 * rho * cd * area * math.sqrt(v_rel @ v_rel)) * v_rel


def wind_velocity(z: float, w: WindConfig) -> np.ndarray:
    """Horizontal wind vector at height z (m above the pad).

    Constant base speed at and below the reference height; above it the speed
    grows with the power law  speed * (z / z_ref) ** shear_exponent.
    """
    if w.speed_mps == 0.0:
        return np.zeros(3)
    speed = w.speed_mps
    if w.shear_exponent != 0.0 and z > w.reference_height_m:
        speed *= (z / w.reference_height_m) ** w.shear_exponent
    a = math.radians(w.toward_deg)
    return np.array([speed * math.sin(a), speed * math.cos(a), 0.0])


def rail_direction(tilt_deg: float, azimuth_deg: float) -> np.ndarray:
    """Unit vector along the launch rail.

    tilt is measured from vertical; azimuth is a compass heading
    (0 = north = +y, 90 = east = +x).
    """
    t, a = math.radians(tilt_deg), math.radians(azimuth_deg)
    u = np.array([math.sin(t) * math.sin(a), math.sin(t) * math.cos(a), math.cos(t)])
    return u / math.sqrt(u @ u)


class Dynamics:
    """Evaluates dy/dt for the state y = [p, v, m] given the discrete flight mode."""

    def __init__(self, cfg: FlightConfig, motor):
        self.cfg = cfg
        self.motor = motor
        self.rail_dir = rail_direction(cfg.launch.tilt_deg, cfg.launch.azimuth_deg)
        self.body_area = math.pi * cfg.rocket.body_diameter_m ** 2 / 4.0
        self.has_air = cfg.atmosphere.model == "isa"
        self.rho0 = cfg.atmosphere.sea_level_density_kg_m3
        self.h0 = cfg.atmosphere.launch_altitude_m
        self.chute_area = math.pi * cfg.recovery.chute_diameter_m ** 2 / 4.0
        self.wind = cfg.wind
        self.calm = cfg.wind.speed_mps == 0.0
        # None = constant body Cd (the original model, used by the default rocket).
        self.mach_drag = mach_drag_params(cfg.rocket) if cfg.rocket.drag_model == "mach" else None

    def air_density(self, z: float) -> float:
        return density(self.h0 + z, self.rho0) if self.has_air else 0.0

    def net_force(self, t: float, y: np.ndarray, on_rail: bool, chute: bool = False) -> np.ndarray:
        v = y[3:6]
        m = y[6]
        thrust = self.motor.thrust(t)
        if on_rail:
            direction = self.rail_dir
        else:
            speed = math.sqrt(v @ v)
            direction = v / speed if speed > _MIN_SPEED else self.rail_dir
        force = thrust * direction + m * GRAVITY
        if self.has_air:
            # Wind enters ONLY here, through the air-relative velocity.
            v_rel = v if self.calm else v - wind_velocity(y[2], self.wind)
            if chute:  # chute replaces body drag once deployed (always subsonic: constant Cd)
                cd, area = self.cfg.recovery.chute_cd, self.chute_area
            else:
                cd, area = self.body_cd(y[2], v_rel), self.body_area
            force += drag_force(v_rel, self.air_density(y[2]), cd, area)
        return force

    def body_cd(self, z: float, v_rel: np.ndarray) -> float:
        """Body drag coefficient for air-relative velocity v_rel at height z."""
        if self.mach_drag is None:
            return self.cfg.rocket.cd
        mach = math.sqrt(v_rel @ v_rel) / speed_of_sound(self.h0 + z)
        return drag_coefficient(self.cfg.rocket.cd, mach, self.mach_drag)

    def drag_cd(self, z: float, v: np.ndarray) -> float:
        """Body Cd for ground-relative velocity v at height z (wind removed)."""
        v = np.asarray(v, dtype=float)
        return self.body_cd(z, v if self.calm else v - wind_velocity(z, self.wind))

    def derivatives(self, t: float, y: np.ndarray, on_rail: bool, chute: bool = False) -> np.ndarray:
        m = y[6]
        a = self.net_force(t, y, on_rail, chute) / m
        if on_rail:
            u = self.rail_dir
            a = (a @ u) * u
        dy = np.empty(7)
        dy[0:3] = y[3:6]
        dy[3:6] = a
        dy[6] = -self.motor.mass_flow(t)
        return dy

    def max_stable_step(self, y: np.ndarray, chute: bool) -> float:
        """Largest step that keeps quadratic drag well-resolved.

        Drag changes velocity on the time scale tau = m / (rho*Cd*A*|v_rel|).
        For the body this is ~1 s, harmless. A parachute opened at high speed
        gives tau ~ 10 ms, which makes the ODE stiff: RK4 goes unstable once
        dt/tau exceeds ~2.8. Capping the step at tau/2 keeps large-dt Monte
        Carlo runs safe without slowing ordinary flights.
        """
        if not (chute and self.has_air):
            return math.inf
        v = y[3:6] if self.calm else y[3:6] - wind_velocity(y[2], self.wind)
        k = self.air_density(y[2]) * self.cfg.recovery.chute_cd * self.chute_area * math.sqrt(v @ v)
        return math.inf if k <= 0.0 else 0.5 * y[6] / k

    def rail_acceleration(self, t: float, y: np.ndarray) -> float:
        """Acceleration along the rail if the rocket were free to slide (liftoff test)."""
        return float(self.net_force(t, y, on_rail=True) @ self.rail_dir / y[6])
