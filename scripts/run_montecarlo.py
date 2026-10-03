"""Monte Carlo dispersion: N dispersed flights -> summary stats, JSON and plots.

Examples
    python scripts/run_montecarlo.py                       # 500 runs, default seed
    python scripts/run_montecarlo.py --n 200 --seed 1 --workers 4
    python scripts/run_montecarlo.py --set wind.speed_mps=5 --wind-sigma 2
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from sim.config import FlightConfig, apply_overrides  # noqa: E402
from sim.export import montecarlo_to_dict, write_json  # noqa: E402
from sim.montecarlo import Dispersion, MonteCarloConfig, default_workers, run_monte_carlo  # noqa: E402


def print_stats(res, elapsed: float) -> None:
    s = res.stats
    lm, ls = s["landing"]["mean_m"], s["landing"]["std_m"]
    e = s["ellipse"]
    timing = {k: res.deploy_timing.count(k) for k in ("before", "near", "after", "none")}
    print(f"Runs             : {s['n_runs']}  (seed {res.mc.seed}, dt {res.mc.dt_s} s, {elapsed:.1f} s wall)")
    print(f"Apogee           : mean {s['apogee']['mean_m']:.1f} m, std {s['apogee']['std_m']:.1f} m, "
          f"range {s['apogee']['min_m']:.1f}-{s['apogee']['max_m']:.1f} m "
          f"(5-95%: {s['apogee']['p5_m']:.1f}-{s['apogee']['p95_m']:.1f})")
    print(f"Landing mean     : east {lm[0]:+.1f} m, north {lm[1]:+.1f} m  (std {ls[0]:.1f} / {ls[1]:.1f} m)")
    print(f"Drift distance   : mean {s['drift']['mean_m']:.1f} m, std {s['drift']['std_m']:.1f} m, "
          f"95th percentile {s['drift']['p95_m']:.1f} m, max {s['drift']['max_m']:.1f} m")
    print(f"2-sigma ellipse  : semi-axes {e['semi_major_m']:.1f} x {e['semi_minor_m']:.1f} m, "
          f"major axis {e['angle_deg']:+.1f} deg from east; contains {100 * e['fraction_inside']:.1f}% "
          f"of landings (Gaussian expectation {100 * e['expected_fraction_gaussian']:.1f}%)")
    print(f"Deploy timing    : {timing}")
    print(f"Nominal flight   : apogee {res.nominal.apogee_m:.1f} m, landing "
          f"({res.nominal.landing_point[0]:+.1f}, {res.nominal.landing_point[1]:+.1f}) m")


def make_plots(res, out_dir: Path) -> list[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.patches import Ellipse

    s = res.stats
    e = s["ellipse"]
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)

    xy = res.landing_xy
    ax.scatter(xy[:, 0], xy[:, 1], s=8, alpha=0.5, label=f"landing points (N={len(xy)})")
    ax.add_patch(Ellipse(e["center_m"], 2 * e["semi_major_m"], 2 * e["semi_minor_m"], angle=e["angle_deg"],
                         fill=False, lw=2, color="C3",
                         label=f"2σ ellipse ({100 * e['fraction_inside']:.0f}% inside)"))
    ax.plot(*s["landing"]["mean_m"], "C3+", ms=14, mew=2, label="mean landing")
    ax.plot(0, 0, "k^", ms=10, label="launch pad")
    ax.plot(*res.nominal.landing_point[:2], "kx", ms=10, mew=2, label="nominal landing")
    w = res.base_config.wind
    if w.speed_mps > 0:
        a = np.radians(w.toward_deg)
        L = 0.15 * max(np.ptp(xy[:, 0]), np.ptp(xy[:, 1]), 50)
        ax.annotate("", xy=(L * np.sin(a), L * np.cos(a)), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="->", color="C0", lw=2))
        ax.text(L * np.sin(a), L * np.cos(a), f" wind {w.speed_mps:g} m/s", color="C0", fontsize=9)
    ax.set(xlabel="east (m)", ylabel="north (m)", title="Landing dispersion (top-down)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="best")

    ax2.hist(res.apogee_m, bins=30, color="C2", alpha=0.8)
    ax2.axvline(s["apogee"]["mean_m"], color="k", lw=1.5, label=f"mean {s['apogee']['mean_m']:.1f} m")
    ax2.axvline(res.nominal.apogee_m, color="C3", ls="--", lw=1.5, label=f"nominal {res.nominal.apogee_m:.1f} m")
    ax2.set(xlabel="apogee (m)", ylabel="runs", title=f"Apogee distribution (std {s['apogee']['std_m']:.1f} m)")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "montecarlo.png"
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return [path]


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    d, mc = Dispersion(), MonteCarloConfig()
    ap.add_argument("--config", default=str(REPO_ROOT / "configs" / "default.json"))
    ap.add_argument("--set", action="append", default=[], metavar="SECTION.KEY=VALUE",
                    help="override a nominal config value (repeatable)")
    ap.add_argument("--n", type=int, default=mc.n_runs, help="number of runs")
    ap.add_argument("--seed", type=int, default=mc.seed)
    ap.add_argument("--dt", type=float, default=mc.dt_s, help="integration step for MC runs (s)")
    ap.add_argument("--workers", type=int, default=default_workers(), help="parallel processes")
    ap.add_argument("--wind-sigma", type=float, default=d.wind_speed_sigma_mps, help="m/s")
    ap.add_argument("--wind-dir-sigma", type=float, default=d.wind_dir_sigma_deg, help="deg")
    ap.add_argument("--impulse-sigma", type=float, default=d.impulse_sigma_frac, help="fraction")
    ap.add_argument("--cd-sigma", type=float, default=d.cd_sigma_frac, help="fraction")
    ap.add_argument("--mass-sigma", type=float, default=d.dry_mass_sigma_frac, help="fraction")
    ap.add_argument("--angle-sigma", type=float, default=d.launch_angle_sigma_deg, help="deg per axis")
    ap.add_argument("--out", default=str(REPO_ROOT / "out" / "montecarlo"))
    ap.add_argument("--no-plots", action="store_true")
    return ap


def mc_from_args(args) -> MonteCarloConfig:
    disp = Dispersion(args.wind_sigma, args.wind_dir_sigma, args.impulse_sigma, args.cd_sigma,
                      args.mass_sigma, args.angle_sigma)
    return MonteCarloConfig(n_runs=args.n, seed=args.seed, dt_s=args.dt, dispersion=disp)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    base = apply_overrides(FlightConfig.load(args.config), args.set)
    mc = mc_from_args(args)

    def progress(i, n):
        if i == n or i % max(1, n // 10) == 0:
            print(f"  {i}/{n} runs", flush=True)

    t0 = time.perf_counter()
    res = run_monte_carlo(base, mc, workers=args.workers, progress=progress)
    print_stats(res, time.perf_counter() - t0)
    out = Path(args.out)
    print(f"Wrote {write_json(montecarlo_to_dict(res, label='CLI Monte Carlo'), out / 'montecarlo.json')}")
    if not args.no_plots:
        for p in make_plots(res, out):
            print(f"Wrote {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
