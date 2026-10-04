"""Regenerate every dataset the web viewer loads, into web/data/.

    python scripts/build_site.py            # needs the C++ flight computer built (SIL datasets)
    python scripts/build_site.py --no-sil   # physics-only datasets

Scenarios are the default config plus small overrides, so they stay in sync
with configs/default.json automatically. The SIL datasets fly the C++ flight
computer closed-loop; a missing executable is an error (not a silent skip),
so a deploy can never quietly lose them.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from sim.config import FlightConfig, apply_overrides  # noqa: E402
from sim.export import flight_to_dict, montecarlo_to_dict, sil_flight_to_dict, write_json  # noqa: E402
from sim.faults import Fault  # noqa: E402
from sim.flight import simulate  # noqa: E402
from sim.montecarlo import MonteCarloConfig, default_workers, run_monte_carlo  # noqa: E402
from sim.sil import SilConfig, run_sil, sil_flight_config  # noqa: E402

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


SIL_SCENARIOS = [
    {"id": "sil_nominal", "label": "SIL nominal",
     "description": "Closed-loop software-in-the-loop flight: the C++ flight computer (Kalman filter) reads "
                    "noisy barometer/accelerometer frames at 100 Hz and decides when to deploy. C6-7 motor "
                    "charge as independent backup.",
     "faults": ()},
    {"id": "sil_faults", "label": "SIL faults",
     "description": "Same flight with injected faults: barometer spikes (+-40 m on 8% of frames, 2-12 s) and "
                    "a 0.6 s link blackout at 5.0 s. The FC gates the spikes out and predicts across the gap. "
                    "(Two simultaneous faults: beyond the single-fault requirements, shown for illustration.)",
     "faults": (Fault("spike", 2.0, 10.0, "baro", 40.0, 0.08), Fault("dropout", 5.0, 0.6))},
]


def build_sil(base: FlightConfig) -> list[dict]:
    cfg = sil_flight_config(base)
    entries = []
    for sc in SIL_SCENARIOS:
        res = run_sil(cfg, SilConfig(seed=1, fc_mode="kalman", faults=sc["faults"]))
        data = sil_flight_to_dict(res, "kalman", label=sc["label"], description=sc["description"])
        path = write_json(data, DATA_DIR / f"{sc['id']}.json")
        dep = data["sil"]["deploy"]
        print(f"  {sc['id']:<12} FC deploy {dep['dt_s']:+.3f} s vs true apogee ({dep['reason']}, chute by "
              f"{dep['mechanism']})  -> {path.relative_to(REPO_ROOT)} ({path.stat().st_size / 1024:.0f} KiB)")
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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-sil", action="store_true", help="skip the flight-computer (SIL) datasets")
    args = ap.parse_args(argv)
    t0 = time.perf_counter()
    base = FlightConfig.load(DEFAULT_CONFIG)
    print("Building flight datasets:")
    entries = build_flights(base)
    if not args.no_sil:
        print("Building SIL datasets (C++ flight computer):")
        entries += build_sil(base)
    print("Building Monte Carlo dataset:")
    entries.append(build_dispersion(base))
    write_json({"schema_version": 1, "datasets": entries}, DATA_DIR / "index.json")
    print(f"Wrote web/data/index.json with {len(entries)} datasets in {time.perf_counter() - t0:.1f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
