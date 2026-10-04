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
from .flight import FlightResult, simulate
from .staging import BoosterSpec, StagedResult, simulate_staged

PRESET_DIR = REPO_ROOT / "configs" / "rockets"
CATEGORIES = ("hobby-small", "hobby-medium", "hobby-large", "high-power", "sounding-two-stage")
_TOP_KEYS = {"id", "label", "category", "order", "description", "notes", "expected", "flight", "booster", "geometry"}
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
    return Preset(id=d["id"], label=d["label"], category=d["category"], order=int(d.get("order", 99)),
                  description=d["description"], flight=flight, booster=booster, geometry=d.get("geometry"),
                  expected=d.get("expected", {}), notes=d.get("notes", {}), path=path)


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


def fly_preset(preset: Preset, controller=None, flight: FlightConfig | None = None) -> PresetFlight:
    """Fly a preset (optionally with a modified flight config, e.g. a coarser dt)."""
    cfg = flight or preset.flight
    if preset.booster is None:
        return PresetFlight(preset, simulate(cfg, controller=controller))
    staged = simulate_staged(cfg, preset.booster, controller=controller)
    return PresetFlight(preset, staged.flight, staged.booster, staged)
