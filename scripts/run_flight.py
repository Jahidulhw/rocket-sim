"""Simulate one flight: print a summary, write viewer JSON and matplotlib plots.

Examples
    python scripts/run_flight.py
    python scripts/run_flight.py --set wind.speed_mps=6 --set launch.tilt_deg=5
    python scripts/run_flight.py --config configs/default.json --out out/flight --no-plots
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from sim.config import FlightConfig, apply_overrides  # noqa: E402
from sim.export import flight_to_dict, write_json  # noqa: E402
from sim.flight import PHASES, simulate  # noqa: E402

PHASE_COLORS = {"PAD": "#8a8f98", "RAIL": "#a855f7", "BOOST": "#f97316", "COAST": "#3b82f6",
                "DESCENT": "#10b981", "LANDED": "#6b7280"}


def print_summary(res) -> None:
    s = res.summary()
    d = res.deployment
    print(f"Motor            : {res.motor.name}  ({res.motor.total_impulse:.2f} N*s, "
          f"burn {res.motor.burn_time:.2f} s)")
    print(f"Apogee           : {s['apogee_m']:.1f} m  ({s['apogee_m'] * 3.28084:.0f} ft) "
          f"at t = {res.events['apogee'].t:.2f} s")
    print(f"Max speed        : {s['max_speed_mps']:.1f} m/s")
    print(f"Flight time      : {s['flight_time_s']:.1f} s  (landed: {s['landed']})")
    x, y, _ = s["landing_point_m"]
    print(f"Landing distance : {s['landing_distance_m']:.1f} m from pad  (east {x:+.1f} m, north {y:+.1f} m)")
    if d.get("deployed"):
        print(f"Deploy           : t = {d['t']:.2f} s, {d['timing']} apogee "
              f"({d['dt_from_apogee_s']:+.2f} s vs true apogee at {d['true_apogee_t']:.2f} s), "
              f"speed {d['speed_mps']:.1f} m/s, altitude {d['altitude_m']:.1f} m")
    else:
        print("Deploy           : no parachute deployment")


def make_plots(res, out_dir: Path) -> list[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    t, p, v = res.t, res.position, res.velocity
    phase = np.array(res.phase)
    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5), constrained_layout=True)

    ax = axes[0, 0]
    for ph in PHASES:
        m = phase == ph
        if m.any():
            ax.plot(t[m], p[m, 2], ".", ms=1.5, color=PHASE_COLORS[ph], label=ph)
    offsets = {"burnout": (8, -4), "apogee": (8, 4), "deploy": (8, -14), "landing": (-10, 8)}
    for name, off in offsets.items():
        if name in res.events:
            e = res.events[name]
            ax.plot(e.t, e.position[2], "kx")
            ax.annotate(name, (e.t, e.position[2]), textcoords="offset points", xytext=off,
                        fontsize=8, ha="right" if off[0] < 0 else "left")
    ax.set(xlabel="time (s)", ylabel="altitude (m)", title="Altitude")
    ax.legend(markerscale=6, fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, np.linalg.norm(v, axis=1), label="speed")
    ax.plot(t, v[:, 2], label="vertical speed")
    ax.set(xlabel="time (s)", ylabel="m/s", title="Speed")
    ax.set_xlim(0, min(t[-1], res.events.get("apogee", res.events["landing"]).t + 5))
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1, 0]
    ax.plot(p[:, 0], p[:, 1])
    ax.plot(0, 0, "k^", label="pad")
    ax.plot(*p[-1, :2], "rv", label="landing")
    ax.set(xlabel="east (m)", ylabel="north (m)", title="Ground track")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    axes[1, 1].remove()
    ax = fig.add_subplot(2, 2, 4, projection="3d")
    ax.plot(p[:, 0], p[:, 1], p[:, 2])
    # Same horizontal limits on both axes so a straight-line drift isn't stretched
    # into a fake curve by autoscaling (a vertical flight has north ~ 1e-14 m).
    cx, cy = (p[:, 0].max() + p[:, 0].min()) / 2, (p[:, 1].max() + p[:, 1].min()) / 2
    half = max(np.ptp(p[:, 0]), np.ptp(p[:, 1]), 1.0) / 2 * 1.1
    ax.set(xlim=(cx - half, cx + half), ylim=(cy - half, cy + half), zlim=(0, p[:, 2].max() * 1.05),
           xlabel="east (m)", ylabel="north (m)", zlabel="up (m)", title="3D trajectory")

    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "flight.png"
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return [path]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(REPO_ROOT / "configs" / "default.json"))
    ap.add_argument("--set", action="append", default=[], metavar="SECTION.KEY=VALUE",
                    help="override a config value (repeatable), e.g. --set wind.speed_mps=6")
    ap.add_argument("--out", default=str(REPO_ROOT / "out" / "flight"), help="output directory")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args(argv)

    cfg = apply_overrides(FlightConfig.load(args.config), args.set)
    res = simulate(cfg)
    print_summary(res)
    out = Path(args.out)
    if res.landed:
        print(f"Wrote {write_json(flight_to_dict(res, label='CLI flight'), out / 'flight.json')}")
    else:
        print("Flight did not land within sim.max_time_s; JSON not written.")
    if not args.no_plots:
        for p in make_plots(res, out):
            print(f"Wrote {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
