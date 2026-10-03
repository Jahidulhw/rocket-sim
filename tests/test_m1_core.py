"""Milestone 1: 3D point-mass core in vacuum.

Every test here compares the simulator against a closed-form textbook result
or against a property that must hold for *any* correct integrator
(energy conservation, symmetry, convergence, constraint satisfaction).
"""

import math

import numpy as np
import pytest

from sim.config import AtmosphereConfig, FlightConfig, LaunchConfig, MotorConfig, RocketConfig, SimConfig
from sim.flight import simulate
from sim.physics import G0, rail_direction

V0 = 50.0  # m/s, launch speed for the ballistic tests


def ballistic_config(v0=V0, theta_deg=45.0, integrator="rk4", dt=1e-3, azimuth_deg=0.0):
    """Zero-thrust projectile launched from the pad at elevation theta (from horizontal)."""
    return FlightConfig(
        rocket=RocketConfig(dry_mass_kg=1.0),
        motor=MotorConfig(kind="constant", thrust_n=0.0, burn_time_s=0.0),
        launch=LaunchConfig(rail_length_m=0.0, tilt_deg=90.0 - theta_deg,
                            azimuth_deg=azimuth_deg, initial_speed_mps=v0),
        atmosphere=AtmosphereConfig(model="vacuum"),
        sim=SimConfig(dt_s=dt, integrator=integrator),
    )


def thrust_config(integrator="rk4", dt=1e-3, tilt_deg=5.0):
    """Constant-thrust, constant-mass rocket on a 1 m rail, in vacuum."""
    return FlightConfig(
        rocket=RocketConfig(dry_mass_kg=0.5),
        motor=MotorConfig(kind="constant", thrust_n=15.0, burn_time_s=1.0),
        launch=LaunchConfig(rail_length_m=1.0, tilt_deg=tilt_deg, azimuth_deg=30.0),
        atmosphere=AtmosphereConfig(model="vacuum"),
        sim=SimConfig(dt_s=dt, integrator=integrator),
    )


def horizontal_range(result):
    return float(np.hypot(*result.landing_point[:2]))


@pytest.mark.parametrize("theta", [20.0, 45.0, 60.0, 80.0])
def test_range_and_max_height_match_textbook(theta):
    res = simulate(ballistic_config(theta_deg=theta))
    th = math.radians(theta)
    expected_range = V0**2 * math.sin(2 * th) / G0
    expected_height = (V0 * math.sin(th)) ** 2 / (2 * G0)
    assert horizontal_range(res) == pytest.approx(expected_range, rel=1e-3)
    assert res.apogee_m == pytest.approx(expected_height, rel=1e-3)


def test_azimuth_points_the_flight_in_the_right_compass_direction():
    # azimuth 90 deg = east = +x
    res = simulate(ballistic_config(theta_deg=45.0, azimuth_deg=90.0))
    x, y, _ = res.landing_point
    assert x > 0 and abs(y) < 1e-9 * abs(x)


def test_time_up_equals_time_down():
    res = simulate(ballistic_config(theta_deg=60.0))
    t_up = res.events["apogee"].t
    t_down = res.events["landing"].t - t_up
    assert t_up == pytest.approx(V0 * math.sin(math.radians(60.0)) / G0, rel=1e-6)
    assert abs(t_up - t_down) < 1e-5


@pytest.mark.parametrize("integrator,tol", [("rk4", 1e-7), ("euler", 1e-3)])
def test_mechanical_energy_conserved_in_vacuum(integrator, tol):
    res = simulate(ballistic_config(theta_deg=50.0, integrator=integrator))
    m = res.mass
    speed2 = np.sum(res.velocity**2, axis=1)
    energy = 0.5 * m * speed2 + m * G0 * res.position[:, 2]
    rel_dev = np.max(np.abs(energy - energy[0])) / energy[0]
    assert rel_dev < tol


def test_apogee_converges_when_dt_is_halved():
    a1 = simulate(thrust_config(dt=0.01)).apogee_m
    a2 = simulate(thrust_config(dt=0.005)).apogee_m
    assert abs(a1 - a2) / a2 < 1e-6


def test_semi_implicit_euler_is_first_order():
    """Halving dt should roughly halve Euler's apogee error (vs. a fine RK4 reference)."""
    ref = simulate(thrust_config(dt=1e-4)).apogee_m
    e1 = abs(simulate(thrust_config(integrator="euler", dt=0.004)).apogee_m - ref)
    e2 = abs(simulate(thrust_config(integrator="euler", dt=0.002)).apogee_m - ref)
    assert 1.6 < e1 / e2 < 2.4


@pytest.mark.parametrize("cfg", [ballistic_config(theta_deg=30.0), ballistic_config(theta_deg=85.0),
                                 thrust_config(tilt_deg=0.0), thrust_config(tilt_deg=30.0)],
                         ids=["ballistic30", "ballistic85", "thrust_vertical", "thrust_tilted"])
def test_no_trajectory_point_below_ground(cfg):
    res = simulate(cfg)
    assert res.landed
    assert np.min(res.position[:, 2]) >= 0.0
    assert res.position[-1, 2] == 0.0  # final point is the interpolated ground contact


def test_rk4_and_semi_implicit_euler_agree():
    """Stated tolerance: 0.5% on apogee and range at dt = 1 ms."""
    rk = simulate(thrust_config(integrator="rk4"))
    eu = simulate(thrust_config(integrator="euler"))
    assert eu.apogee_m == pytest.approx(rk.apogee_m, rel=5e-3)
    assert horizontal_range(eu) == pytest.approx(horizontal_range(rk), rel=5e-3)


def test_motion_on_rail_is_along_rail_and_exit_is_at_rail_length():
    cfg = thrust_config(tilt_deg=20.0)
    res = simulate(cfg)
    u = rail_direction(cfg.launch.tilt_deg, cfg.launch.azimuth_deg)
    t_exit = res.events["rail_exit"].t
    on_rail = res.t <= t_exit
    pos = res.position[on_rail]
    # Every on-rail point lies on the rail line: no component perpendicular to u.
    perp = pos - np.outer(pos @ u, u)
    assert np.max(np.linalg.norm(perp, axis=1)) < 1e-12
    exit_pos = res.events["rail_exit"].position
    assert exit_pos @ u == pytest.approx(cfg.launch.rail_length_m, rel=1e-9)
    # At rail exit the velocity is still along the rail.
    v = res.events["rail_exit"].velocity
    assert np.linalg.norm(np.cross(v, u)) < 1e-9 * np.linalg.norm(v)


def test_after_rail_thrust_follows_velocity_gravity_turn():
    """A tilted launch must curve over: flight-path angle decreases monotonically."""
    res = simulate(thrust_config(tilt_deg=10.0))
    after = res.t > res.events["rail_exit"].t
    v = res.velocity[after & (res.t < res.events["apogee"].t)]
    horiz = np.hypot(v[:, 0], v[:, 1])
    gamma = np.arctan2(v[:, 2], horiz)
    assert np.all(np.diff(gamma) <= 1e-12)


def test_underpowered_rocket_never_leaves_pad():
    cfg = thrust_config()
    cfg.motor.thrust_n = 0.5 * cfg.rocket.dry_mass_kg * G0  # thrust < weight
    res = simulate(cfg)
    assert "liftoff" not in res.events
    assert not res.landed
    assert np.all(res.position == 0.0)


def test_constant_mass_in_milestone_one_motor():
    res = simulate(thrust_config())
    assert np.all(res.mass == res.mass[0])
