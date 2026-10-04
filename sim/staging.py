"""Passive two-stage flight: booster + sustainer, with the spent booster as its
own tracked body.

Sequence (all timing is passive and pre-set; the flight computer never
commands staging):

  1. STACK      lift-off on the booster motor. The sustainer, its unlit motor
                and the booster fly as one body: booster thrust, stack drag
                (booster diameter, `stack_cd`), stack mass.
  2. SEPARATION at booster burnout. Drag separation: the spent booster has a
                much lower ballistic coefficient (mass / Cd A) than the
                sustainer, so it falls behind on its own. No separation
                impulse is modelled; both bodies start with the stack's
                position and velocity, which conserves momentum exactly.
  3. SUSTAINER  coasts for `sustainer_ignition_delay_s`, then its motor lights
                (like a hobby booster burning through into the sustainer, or a
                pre-set timer). The FC (SIL) rides here; recovery per its own
                RecoveryConfig (drogue/main).
     BOOSTER    spent casing + airframe; its chute is deployed by its own motor
                ejection charge at burnout + delay (passive, no electronics).

Mass bookkeeping at separation:
    stack mass = booster dry + booster casing (loaded - propellant)
               + sustainer dry + sustainer loaded motor
    booster    = booster dry + booster casing
    sustainer  = sustainer dry + sustainer loaded motor

Each phase reuses flight.simulate (one integrator for everything): the stack
with stop_time at separation, the two bodies with start=(t_sep, y_sep).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import FlightConfig
from .flight import Event, FlightResult, simulate
from .motor import DelayedMotor, build_motor


@dataclass
class BoosterSpec:
    """The booster stage. `flight` holds its rocket (airframe), motor and recovery;
    its other sections are the sustainer's."""
    flight: FlightConfig
    sustainer_ignition_delay_s: float
    stack_cd: float

    def validate(self) -> None:
        if not 0.0 <= self.sustainer_ignition_delay_s <= 10.0:
            raise ValueError("booster.sustainer_ignition_delay_s must be in [0, 10] s")
        if self.stack_cd <= 0:
            raise ValueError("booster.stack_cd must be positive")
        if self.flight.motor.kind != "eng":
            raise ValueError("booster motor must be a thrust-curve (.eng) motor")


@dataclass
class StagedResult:
    flight: FlightResult        # sustainer, lift-off to landing (stack + sustainer segments)
    booster: FlightResult       # booster, lift-off to landing (stack + booster segments)
    stack: FlightResult
    sustainer_segment: FlightResult
    booster_segment: FlightResult
    separation_t: float
    ignition_t: float


def stack_config(sustainer: FlightConfig, booster: BoosterSpec, sustainer_motor_mass_kg: float) -> FlightConfig:
    cfg = sustainer.copy()
    cfg.rocket = booster.flight.rocket.__class__(**{**booster.flight.rocket.__dict__})
    cfg.rocket.dry_mass_kg = (booster.flight.rocket.dry_mass_kg + sustainer.rocket.dry_mass_kg
                              + sustainer_motor_mass_kg)
    cfg.rocket.cd = booster.stack_cd
    cfg.motor = booster.flight.motor.__class__(**{**booster.flight.motor.__dict__})
    cfg.validate()
    return cfg


def _stitch(first: FlightResult, second: FlightResult, cfg: FlightConfig, motor, extra_events: dict) -> FlightResult:
    """Concatenate two segments that share the sample at the joint time."""
    events = {**first.events, **extra_events, **second.events}
    if "burnout" in first.events:   # the stack's burnout is the BOOSTER's
        events["booster_burnout"] = first.events["burnout"]
        if "burnout" not in second.events:
            del events["burnout"]
        else:
            events["burnout"] = second.events["burnout"]
    return FlightResult(config=cfg, motor=motor,
                        t=np.concatenate([first.t, second.t[1:]]),
                        position=np.concatenate([first.position, second.position[1:]]),
                        velocity=np.concatenate([first.velocity, second.velocity[1:]]),
                        mass=np.concatenate([first.mass, second.mass[1:]]),
                        phase=list(first.phase) + list(second.phase[1:]),
                        events=events, landed=second.landed, deployment=second.deployment,
                        next_tick_k=second.next_tick_k, final_state=second.final_state)


def simulate_staged(sustainer: FlightConfig, booster: BoosterSpec, controller=None) -> StagedResult:
    booster.validate()
    sust_motor = build_motor(sustainer.motor)
    stack_cfg = stack_config(sustainer, booster, sust_motor.total_mass)
    stack_motor = build_motor(booster.flight.motor)
    stack_motor.ejection_delay_s = None        # the booster's charge fires on the booster body, later
    t_sep = stack_motor.burn_time
    stack = simulate(stack_cfg, motor=stack_motor, controller=controller, stop_time=t_sep)
    if stack.landed or stack.t[-1] < t_sep - 1e-9:
        raise RuntimeError("stack never reached booster burnout in flight")
    y_sep = stack.final_state

    # Split the stack state into the two bodies (same position and velocity).
    booster_motor = build_motor(booster.flight.motor)
    casing = booster_motor.total_mass - booster_motor.propellant_mass
    y_boost = y_sep.copy()
    y_boost[6] = booster.flight.rocket.dry_mass_kg + casing
    y_sust = y_sep.copy()
    y_sust[6] = sustainer.rocket.dry_mass_kg + sust_motor.total_mass

    t_ign = t_sep + booster.sustainer_ignition_delay_s
    delayed = DelayedMotor(sust_motor, t_ign)
    sust_seg = simulate(sustainer, motor=delayed, controller=controller, start=(t_sep, y_sust),
                        first_tick_k=stack.next_tick_k)
    boost_seg = simulate(booster.flight, motor=booster_motor, start=(t_sep, y_boost))

    sep_event = {"separation": Event("separation", t_sep, y_sep[0:3].copy(), y_sep[3:6].copy())}
    return StagedResult(
        flight=_stitch(stack, sust_seg, sustainer, delayed, sep_event),
        booster=_stitch(stack, boost_seg, booster.flight, booster_motor, sep_event),
        stack=stack, sustainer_segment=sust_seg, booster_segment=boost_seg,
        separation_t=t_sep, ignition_t=t_ign)
