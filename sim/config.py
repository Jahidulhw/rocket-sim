"""Configuration dataclasses.

Everything that defines a flight lives in one `FlightConfig` tree so that a
flight is fully reproducible from a JSON file (and so the viewer can show the
exact inputs used). Unknown keys raise an error instead of being silently
ignored: a typo like "cd " in a config file should fail loudly.

Units are SI throughout. Coordinates: x = east, y = north, z = up.
"""

from __future__ import annotations

import copy
import json
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


@dataclass
class RocketConfig:
    dry_mass_kg: float = 0.040        # airframe + recovery, WITHOUT motor
    body_diameter_m: float = 0.0248   # sets the drag reference area
    cd: float = 0.75                  # body drag coefficient (power-on and coast)


@dataclass
class MotorConfig:
    kind: str = "eng"                 # "eng" (RASP thrust-curve file) or "constant"
    # --- kind == "eng" ------------------------------------------------------
    eng_file: str = "data/motors/Estes_C6.eng"   # relative to the repo root
    ejection_delay_s: float | None = 5.0          # C6-5; None = no ejection charge (plugged)
    impulse_scale: float = 1.0                    # thrust multiplier (Monte Carlo)
    # --- kind == "constant" -------------------------------------------------
    thrust_n: float = 0.0
    burn_time_s: float = 0.0
    total_mass_kg: float = 0.0        # loaded motor mass
    propellant_mass_kg: float = 0.0


@dataclass
class LaunchConfig:
    rail_length_m: float = 1.0
    tilt_deg: float = 0.0             # rail angle measured FROM VERTICAL (0 = straight up)
    azimuth_deg: float = 0.0          # compass heading the rail leans toward: 0 = +y (north), 90 = +x (east)
    initial_speed_mps: float = 0.0    # speed along the rail at t = 0 (0 for real launches; used by analytic tests)


@dataclass
class RecoveryConfig:
    enabled: bool = True
    chute_diameter_m: float = 0.305   # 12 in hobby parachute
    chute_cd: float = 0.8             # flat plastic hobby chute (typical 0.75-0.8)
    deploy_delay_s: float | None = None   # after burnout; None = use motor.ejection_delay_s
    near_apogee_window_s: float = 1.0     # |t_deploy - t_apogee| <= this counts as "near"


@dataclass
class WindConfig:
    speed_mps: float = 0.0            # base speed at and below reference_height_m
    toward_deg: float = 90.0          # compass direction the wind blows TOWARD (90 = toward +x/east)
    shear_exponent: float = 0.0       # power-law exponent above reference height (0 = uniform; ~1/7 open terrain)
    reference_height_m: float = 10.0


@dataclass
class AtmosphereConfig:
    model: str = "isa"                # "isa" (troposphere density profile) or "vacuum"
    sea_level_density_kg_m3: float = 1.225
    launch_altitude_m: float = 0.0    # pad height above sea level (density lookup only)


@dataclass
class SimConfig:
    dt_s: float = 0.001
    integrator: str = "rk4"           # "rk4" or "euler" (semi-implicit)
    max_time_s: float = 600.0


@dataclass
class FlightConfig:
    rocket: RocketConfig = field(default_factory=RocketConfig)
    motor: MotorConfig = field(default_factory=MotorConfig)
    launch: LaunchConfig = field(default_factory=LaunchConfig)
    atmosphere: AtmosphereConfig = field(default_factory=AtmosphereConfig)
    wind: WindConfig = field(default_factory=WindConfig)
    recovery: RecoveryConfig = field(default_factory=RecoveryConfig)
    sim: SimConfig = field(default_factory=SimConfig)

    def __post_init__(self):
        self.validate()

    def validate(self) -> None:
        if self.sim.dt_s <= 0:
            raise ValueError("sim.dt_s must be positive")
        if self.sim.integrator not in ("rk4", "euler"):
            raise ValueError(f"unknown integrator {self.sim.integrator!r}")
        if not 0.0 <= self.launch.tilt_deg < 90.0:
            raise ValueError("launch.tilt_deg must be in [0, 90)")
        if self.launch.rail_length_m < 0:
            raise ValueError("launch.rail_length_m must be >= 0")
        if self.rocket.dry_mass_kg <= 0:
            raise ValueError("rocket.dry_mass_kg must be positive")
        if self.motor.kind not in ("constant", "eng"):
            raise ValueError(f"unknown motor kind {self.motor.kind!r}")
        if self.wind.speed_mps < 0 or self.wind.reference_height_m <= 0:
            raise ValueError("wind speed must be >= 0 and reference height > 0")
        if self.recovery.chute_diameter_m <= 0 or self.recovery.chute_cd <= 0:
            raise ValueError("chute diameter and Cd must be positive")
        if self.atmosphere.model not in ("isa", "vacuum"):
            raise ValueError(f"unknown atmosphere model {self.atmosphere.model!r}")

    # ---- serialisation ----------------------------------------------------
    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "FlightConfig":
        sections = {f.name: f for f in fields(cls)}
        unknown = set(d) - set(sections)
        if unknown:
            raise ValueError(f"unknown config sections: {sorted(unknown)}")
        kwargs = {}
        for name, f in sections.items():
            sub_cls = f.default_factory().__class__
            kwargs[name] = _build(sub_cls, d.get(name, {}), name)
        return cls(**kwargs)

    @classmethod
    def load(cls, path: str | Path) -> "FlightConfig":
        with open(path, encoding="utf-8") as fh:
            return cls.from_dict(json.load(fh))

    def copy(self) -> "FlightConfig":
        return copy.deepcopy(self)


def _build(sub_cls, d: dict, section: str):
    names = {f.name for f in fields(sub_cls)}
    unknown = set(d) - names
    if unknown:
        raise ValueError(f"unknown keys in [{section}]: {sorted(unknown)}")
    return sub_cls(**d)


def apply_overrides(cfg: FlightConfig, overrides: list[str] | dict) -> FlightConfig:
    """Return a copy of cfg with dotted-path overrides applied.

    Accepts ["wind.speed_mps=6", "recovery.deploy_delay_s=null"] (values are
    parsed as JSON, falling back to plain strings) or {"wind.speed_mps": 6}.
    """
    d = cfg.to_dict()
    items = overrides.items() if isinstance(overrides, dict) else (_split(o) for o in overrides)
    for path, value in items:
        section, _, key = path.partition(".")
        if section not in d or key not in d[section]:
            raise ValueError(f"unknown config path {path!r}")
        d[section][key] = value
    return FlightConfig.from_dict(d)


def _split(item: str):
    path, sep, raw = item.partition("=")
    if not sep:
        raise ValueError(f"override must look like section.key=value, got {item!r}")
    try:
        value = json.loads(raw)
    except json.JSONDecodeError:
        value = raw
    return path.strip(), value


def resolve_path(p: str | Path) -> Path:
    """Resolve a data path relative to the repository root (unless absolute)."""
    p = Path(p)
    return p if p.is_absolute() else REPO_ROOT / p
