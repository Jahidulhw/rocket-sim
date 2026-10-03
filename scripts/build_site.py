"""Regenerate every dataset the web viewer loads, into web/data/.

    python scripts/build_site.py

Scenarios are the default config plus small overrides, so they stay in sync
with configs/default.json automatically.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from sim.config import FlightConfig, apply_overrides  # noqa: E402
from sim.export import flight_to_dict, montecarlo_to_dict, write_json  # noqa: E402
from sim.flight import simulate  # noqa: E402
from sim.montecarlo import MonteCarloConfig, default_workers, run_monte_carlo  # noqa: E402

DATA_DIR = REPO_ROOT / "web" / "data"
DEFAULT_CONFIG = REPO_ROOT / "configs" / "default.json"

FLIGHT_SCENARIOS = [
    {"id": "default", "label": "Default flight",
     "description": "Default rocket on an Estes C6-5, vertical 1 m rail, 2 m/s breeze toward east.",
     "overrides": {}},
    {"id": "windy", "label": "Windy day",
     "description": "7 m/s wind toward north-east (60 deg) at 10 m, increasing with height "
                    "(power law, exponent 1/7).",
     "overrides": {"wind.speed_mps": 7.0, "wind.toward_deg": 60.0, "wind.shear_exponent": 1 / 7}},
    {"id": "angled", "label": "Angled launch",
     "description": "Rail tilted 10 deg toward west (azimuth 270), into a 4 m/s wind blowing "
                    "toward east.",
     "overrides": {"launch.tilt_deg": 10.0, "launch.azimuth_deg": 270.0, "wind.speed_mps": 4.0}},
]


def build_flights(base: FlightConfig) -> list[dict]:
    entries = []
    for sc in FLIGHT_SCENARIOS:
        cfg = apply_overrides(base, sc["overrides"])
        res = simulate(cfg)
        data = flight_to_dict(res, label=sc["label"], description=sc["description"])
        path = write_json(data, DATA_DIR / f"{sc['id']}.json")
        s = res.summary()
        print(f"  {sc['id']:<8} apogee {s['apogee_m']:6.1f} m  landing {s['landing_distance_m']:6.1f} m  "
              f"samples {len(data['trajectory']['t']):5d}  -> {path.relative_to(REPO_ROOT)} "
              f"({path.stat().st_size / 1024:.0f} KiB)")
        entries.append({"id": sc["id"], "kind": "flight", "label": sc["label"],
                        "description": sc["description"], "file": path.name})
    return entries


def build_dispersion(base: FlightConfig) -> dict:
    mc = MonteCarloConfig()  # 500 runs, fixed seed, documented default dispersions
    t0 = time.perf_counter()
    res = run_monte_carlo(base, mc, workers=default_workers())
    desc = (f"{mc.n_runs} dispersed flights of the default configuration (seed {mc.seed}): wind speed "
            f"and direction, motor impulse, Cd, dry mass and rail angle varied.")
    path = write_json(montecarlo_to_dict(res, label="Dispersion (Monte Carlo)", description=desc),
                      DATA_DIR / "dispersion.json")
    st = res.stats
    print(f"  dispersion {mc.n_runs} runs in {time.perf_counter() - t0:.1f} s: apogee "
          f"{st['apogee']['mean_m']:.1f} +- {st['apogee']['std_m']:.1f} m, drift p95 {st['drift']['p95_m']:.1f} m"
          f"  -> {path.relative_to(REPO_ROOT)} ({path.stat().st_size / 1024:.0f} KiB)")
    return {"id": "dispersion", "kind": "montecarlo", "label": "Dispersion (Monte Carlo)",
            "description": desc, "file": path.name}


def main() -> int:
    t0 = time.perf_counter()
    base = FlightConfig.load(DEFAULT_CONFIG)
    print("Building flight datasets:")
    entries = build_flights(base)
    print("Building Monte Carlo dataset:")
    entries.append(build_dispersion(base))
    write_json({"schema_version": 1, "datasets": entries}, DATA_DIR / "index.json")
    print(f"Wrote web/data/index.json with {len(entries)} datasets in {time.perf_counter() - t0:.1f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
