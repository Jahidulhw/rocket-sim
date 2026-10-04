"""Design-your-own: validate a rocket file, check its stability, and fly it.

    python scripts/check_rocket.py configs/custom_rocket_template.json
    python scripts/check_rocket.py configs/rockets/swift-k940.json

The file uses the preset schema (see configs/custom_rocket_template.json).
Impossible inputs are rejected with a message naming the field. The
stability check is a design check (Barrowman CP, CG from component masses):
the point-mass flight does not depend on it.

Exit code: 0 OK, 1 invalid file, 2 valid but under-stable.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from sim.atmosphere import speed_of_sound  # noqa: E402
from sim.fleet import fly_preset, load_preset, preset_motors, preset_stability  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--no-flight", action="store_true", help="only validate and check stability")
    args = ap.parse_args(argv)
    try:
        preset = load_preset(args.file)
    except (OSError, ValueError) as exc:
        print(f"INVALID: {exc}")
        return 1
    print(f"{preset.label}  [{preset.category}]  motor(s): {preset_motors(preset)}")
    stable = True
    for name, r in preset_stability(preset).items():
        print(f"  stability ({name}): CP {r.cp_m:.3f} m, CG {r.cg_liftoff_m:.3f} m at liftoff / "
              f"{r.cg_burnout_m:.3f} m at burnout -> static margin {r.margin_liftoff_cal:.2f} / "
              f"{r.margin_burnout_cal:.2f} calibers")
        for w in r.warnings:
            print(f"    WARNING: {w}")
        stable &= r.stable
    if not args.no_flight:
        res = fly_preset(preset)
        f = res.flight
        v = np.linalg.norm(f.velocity, axis=1)
        mach = float(np.max(v / np.array([speed_of_sound(z) for z in f.position[:, 2]])))
        d = f.deployment
        print(f"  flight: apogee {f.apogee_m:.0f} m, max speed {f.max_speed_mps:.0f} m/s (Mach {mach:.2f}), "
              f"lands {f.landing_distance_m:.0f} m from the pad after {f.flight_time_s:.0f} s; chute "
              + (f"{d['timing']} apogee by {d['mechanism']} at {d['speed_mps']:.1f} m/s" if d.get("deployed")
                 else "NOT deployed"))
        if res.booster is not None:
            print(f"  booster: apogee {res.booster.apogee_m:.0f} m, lands "
                  f"{res.booster.landing_distance_m:.0f} m from the pad")
    return 0 if stable else 2


if __name__ == "__main__":
    raise SystemExit(main())
