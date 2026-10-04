"""The rocket fleet: presets in configs/rockets/*.json.

A preset file:

    {
      "id": "...", "label": "...", "category": "...", "order": 1,
      "description": "...",
      "notes": {...},                 design justification (free-form, documented)
      "expected": {"apogee_m": [lo, hi], ...},   plausibility band, from hand estimates
      "flight": {...},                a full FlightConfig: the only stage, or the SUSTAINER
      "booster": null | {             two-stage only
          "rocket": {...}, "motor": {...}, "recovery": {...},
          "sustainer_ignition_delay_s": 1.0, "stack_cd": 0.55
      },
      "geometry": {...}               component dimensions/masses (stability check)
    }

Scope: hobby, high-power and generic research (sounding) rockets only.
Staging is passive; the flight computer only detects events and deploys
parachutes.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from .config import REPO_ROOT, FlightConfig
from .design import Geometry, StabilityReport, assess, build_geometry
from .flight import FlightResult, simulate
from .motor import build_motor
from .staging import BoosterSpec, StagedResult, simulate_staged

PRESET_DIR = REPO_ROOT / "configs" / "rockets"
CATEGORIES = ("hobby-small", "hobby-medium", "hobby-large", "high-power", "sounding-two-stage")
_TOP_KEYS = {"id", "label", "category", "order", "description", "notes", "expected", "flight", "booster",
             "geometry", "sil"}
_SIL_KEYS = {"fc", "sensors", "flight_overrides", "notes"}
# Derived from the preset's sensor model, never set by hand (R comes from the sensors).
_DERIVED_FC_KEYS = {"kf.baro_sigma_m", "kf.baro_quant_m", "kf.accel_sigma_mps2", "accel_full_scale_mps2",
                    "accel_ref_gate_mps2"}
_BOOSTER_KEYS = {"rocket", "motor", "recovery", "sustainer_ignition_delay_s", "stack_cd"}


@dataclass
class Preset:
    id: str
    label: str
    category: str
    order: int
    description: str
    flight: FlightConfig
    booster: BoosterSpec | None = None
    geometry: dict | None = None
    expected: dict = field(default_factory=dict)
    notes: dict = field(default_factory=dict)
    path: Path | None = None
    sil: dict = field(default_factory=dict)

    @property
    def two_stage(self) -> bool:
        return self.booster is not None


@dataclass
class PresetFlight:
    """Result of flying a preset: `flight` is the (sustainer) body carrying the
    flight computer; `booster` the spent booster for two-stage rockets."""
    preset: Preset
    flight: FlightResult
    booster: FlightResult | None = None
    staged: StagedResult | None = None


def preset_from_dict(d: dict, path: Path | None = None) -> Preset:
    where = path.name if path else "preset"
    unknown = set(d) - _TOP_KEYS
    if unknown:
        raise ValueError(f"{where}: unknown keys {sorted(unknown)}")
    for k in ("id", "label", "category", "description", "flight"):
        if k not in d:
            raise ValueError(f"{where}: missing required key {k!r}")
    if d["category"] not in CATEGORIES:
        raise ValueError(f"{where}: category {d['category']!r} not one of {CATEGORIES}")
    try:
        flight = FlightConfig.from_dict(d["flight"])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{where}: flight: {exc}") from exc
    booster = None
    if d.get("booster") is not None:
        b = d["booster"]
        unknown = set(b) - _BOOSTER_KEYS
        missing = _BOOSTER_KEYS - set(b)
        if unknown or missing:
            parts = ([f"unknown {sorted(unknown)}"] if unknown else []) + ([f"missing {sorted(missing)}"] if missing else [])
            raise ValueError(f"{where}: booster keys: " + ", ".join(parts))
        bd = {**d["flight"], "rocket": b["rocket"], "motor": b["motor"], "recovery": b["recovery"]}
        try:
            booster = BoosterSpec(FlightConfig.from_dict(bd), float(b["sustainer_ignition_delay_s"]),
                                  float(b["stack_cd"]))
            booster.validate()
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{where}: booster: {exc}") from exc
    if "geometry" not in d:
        raise ValueError(f"{where}: missing required key 'geometry' (needed for the stability check)")
    preset = Preset(id=d["id"], label=d["label"], category=d["category"], order=int(d.get("order", 99)),
                    description=d["description"], flight=flight, booster=booster, geometry=d["geometry"],
                    expected=d.get("expected", {}), notes=d.get("notes", {}), path=path, sil=d.get("sil") or {})
    try:
        geometries(preset)          # validates the geometry and its agreement with the flight config
        sil_settings(preset)        # validates the SIL section
        sil_flight_config_for(preset)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{where}: {exc}") from exc
    return preset


def sil_settings(preset: Preset) -> dict:
    """The preset's SIL sensor model and FC arguments. The filter's measurement
    noise, the accelerometer full scale and the reference gate are DERIVED from
    the sensor model (R comes from the sensors), not configured."""
    from .sensors import SensorConfig
    sil = preset.sil or {}
    unknown = set(sil) - _SIL_KEYS
    if unknown:
        raise ValueError(f"sil: unknown keys {sorted(unknown)}")
    sensors = SensorConfig(**sil.get("sensors", {}))
    sensors.validate()
    fc = dict(sil.get("fc", {}))
    clash = set(fc) & _DERIVED_FC_KEYS
    if clash:
        raise ValueError(f"sil.fc: {sorted(clash)} are derived from sil.sensors and must not be set")
    for k, v in fc.items():
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            raise ValueError(f"sil.fc.{k} must be a number")
    derived = {"kf.baro_sigma_m": sensors.baro_noise_sigma_m, "kf.baro_quant_m": sensors.baro_quant_m,
               "kf.accel_sigma_mps2": sensors.accel_noise_sigma_mps2,
               "accel_full_scale_mps2": sensors.accel_range_mps2,
               "accel_ref_gate_mps2": 4.0 * sensors.accel_noise_sigma_mps2}
    default = SensorConfig()
    args = []
    for k, v in sorted(derived.items()):     # only pass what differs from the FC's built-in defaults
        base = {"kf.baro_sigma_m": default.baro_noise_sigma_m, "kf.baro_quant_m": default.baro_quant_m,
                "kf.accel_sigma_mps2": default.accel_noise_sigma_mps2,
                "accel_full_scale_mps2": default.accel_range_mps2,
                "accel_ref_gate_mps2": 4.0 * default.accel_noise_sigma_mps2}[k]
        if v != base:
            args += ["--param", f"{k}={v:.10g}"]
    for k, v in fc.items():
        args += ["--param", f"{k}={v:.10g}"]
    return {"sensors": sensors, "fc_args": args, "fc": fc}


def sil_flight_config_for(preset: Preset) -> FlightConfig:
    """Flight config used under SIL (e.g. a longer motor delay so the ejection
    charge is a true backup that fires after the FC's apogee event)."""
    from .config import apply_overrides
    over = (preset.sil or {}).get("flight_overrides", {})
    return apply_overrides(preset.flight, over) if over else preset.flight


def _agree(geom: Geometry, rocket, label: str) -> None:
    """Geometry and flight config must describe the same airframe."""
    if abs(geom.dry_mass_kg - rocket.dry_mass_kg) > 0.02 * rocket.dry_mass_kg:
        raise ValueError(f"{label} geometry component masses sum to {geom.dry_mass_kg:.4f} kg but "
                         f"rocket.dry_mass_kg is {rocket.dry_mass_kg:.4f} kg (must agree within 2 %)")
    if abs(geom.max_diameter_m - rocket.body_diameter_m) > 0.01 * rocket.body_diameter_m:
        raise ValueError(f"{label} geometry max diameter {geom.max_diameter_m:.4f} m but "
                         f"rocket.body_diameter_m (drag reference) is {rocket.body_diameter_m:.4f} m")


def geometries(preset: Preset) -> dict:
    """Validated geometry per stage: {'sustainer': Geometry[, 'booster': Geometry]}."""
    g = preset.geometry
    if preset.booster is None:
        geom = build_geometry(g)
        _agree(geom, preset.flight.rocket, "")
        return {"sustainer": geom}
    if set(g) != {"sustainer", "booster"}:
        raise ValueError("two-stage geometry needs exactly 'sustainer' and 'booster' sections")
    sust = build_geometry(g["sustainer"])
    boost = build_geometry(g["booster"], x0=sust.length_m, reference_diameter_m=sust.reference_diameter_m,
                           require_nose=False)
    if "nose" in g["booster"]:
        raise ValueError("booster geometry must not have a nose (it starts at the sustainer's tail)")
    _agree(sust, preset.flight.rocket, "sustainer")
    _agree(boost, preset.booster.flight.rocket, "booster")
    return {"sustainer": sust, "booster": boost}


def preset_stability(preset: Preset) -> dict[str, StabilityReport]:
    """Static stability of every configuration the rocket flies in: the whole
    rocket, or for a two-stage the full stack AND the sustainer alone."""
    geo = geometries(preset)
    sust = (geo["sustainer"], build_motor(preset.flight.motor))
    if preset.booster is None:
        return {"rocket": assess([sust])}
    boost = (geo["booster"], build_motor(preset.booster.flight.motor))
    return {"stack": assess([sust, boost]), "sustainer": assess([sust])}


def load_preset(path: str | Path) -> Preset:
    path = Path(path)
    with open(path, encoding="utf-8") as fh:
        return preset_from_dict(json.load(fh), path)


def load_fleet(directory: Path = PRESET_DIR) -> list[Preset]:
    presets = [load_preset(p) for p in sorted(directory.glob("*.json"))]
    ids = [p.id for p in presets]
    if len(set(ids)) != len(ids):
        raise ValueError(f"duplicate preset ids: {ids}")
    return sorted(presets, key=lambda p: (p.order, p.id))


def motor_designation(motor_cfg) -> str:
    """How the motor is flown, e.g. 'B6-6', 'G80T-11', 'K940-P' (P = plugged, no ejection charge)."""
    if motor_cfg.kind != "eng":
        return "constant"
    common = Path(motor_cfg.eng_file).stem.split("_", 1)[-1]
    delay = motor_cfg.ejection_delay_s
    return f"{common}-{'P' if delay is None else f'{delay:g}'}"


def preset_motors(preset: Preset) -> str:
    sust = motor_designation(preset.flight.motor)
    return sust if preset.booster is None else f"{motor_designation(preset.booster.flight.motor)} + {sust}"


def fly_preset(preset: Preset, controller=None, flight: FlightConfig | None = None,
               booster: BoosterSpec | None = None) -> PresetFlight:
    """Fly a preset (optionally with a modified flight config, e.g. a coarser
    dt or Monte Carlo dispersions; `booster` likewise replaces the booster)."""
    cfg = flight or preset.flight
    if preset.booster is None:
        return PresetFlight(preset, simulate(cfg, controller=controller))
    staged = simulate_staged(cfg, booster or preset.booster, controller=controller)
    return PresetFlight(preset, staged.flight, staged.booster, staged)
