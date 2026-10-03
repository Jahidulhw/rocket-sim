"""Fly closed-loop software-in-the-loop flights with the C++ flight computer.

    python scripts/run_sil.py                         # one flight, seed 1
    python scripts/run_sil.py --seed 7 --mode baseline
    python scripts/run_sil.py --seeds 50              # deploy-timing statistics over 50 seeds
    python scripts/run_sil.py --set wind.speed_mps=6

Single flight: prints FC events next to the true events, deploy timing and
mechanism, and writes the per-tick log to out/sil/sil_log.csv.
Multi-seed: prints and saves deploy-timing statistics to out/sil/.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from sim.config import FlightConfig, apply_overrides  # noqa: E402
from sim.sil import LOG_FIELDS, SilConfig, run_sil, sil_flight_config  # noqa: E402

OUT_DIR = REPO_ROOT / "out" / "sil"


def build_config(args) -> FlightConfig:
    cfg = sil_flight_config(FlightConfig.load(args.config))
    return apply_overrides(cfg, args.set) if args.set else cfg


def print_single(res, mode: str) -> None:
    ev, dep = res.flight.events, res.flight.deployment
    st = res.fc_state_times()
    truth_for = {"BOOST": "liftoff", "COAST": "burnout", "APOGEE": "apogee", "LANDED": "landing"}
    print(f"FC mode          : {mode}")
    print(f"{'FC state':<10} {'FC t (s)':>9}   {'true event':<10} {'true t (s)':>10} {'FC lag (s)':>10}")
    for s in ["PAD", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED"]:
        fc_t = st.get(s)
        name = truth_for.get(s, "")
        tr_t = ev[name].t if name in ev else None
        if name == "apogee" and dep.get("true_apogee_t") is not None:
            tr_t = dep["true_apogee_t"]
        lag = f"{fc_t - tr_t:+10.3f}" if fc_t is not None and tr_t is not None else ""
        fc_s = f"{fc_t:9.3f}" if fc_t is not None else f"{'-':>9}"
        tr_s = f"{tr_t:10.3f}" if tr_t is not None else f"{'':>10}"
        print(f"{s:<10} {fc_s}   {name:<10} {tr_s} {lag}")
    print()
    if dep.get("deployed"):
        print(f"Deploy           : t = {dep['t']:.3f} s by {dep['mechanism'].upper()}"
              f"{' (' + res.fc_deploy_reason + ')' if dep['mechanism'] == 'fc' else ''}, "
              f"{dep['dt_from_apogee_s']:+.3f} s vs true apogee ({dep['true_apogee_t']:.3f} s), "
              f"altitude {dep['altitude_m']:.1f} m, speed {dep['speed_mps']:.1f} m/s")
    else:
        print("Deploy           : NONE")
    if res.fc_deploy_t is not None and dep.get("mechanism") != "fc":
        print(f"FC command       : t = {res.fc_deploy_t:.3f} s (chute already out)")
    print(f"Apogee (true)    : {res.flight.apogee_m:.1f} m")
    print(f"Landed           : {res.flight.landed} at t = {res.flight.flight_time_s:.2f} s")
    print(f"FC health        : {'FAILED at t = %.2f s (%s)' % (res.failure_t, res.failure_reason) if res.fc_failed else 'ok'}"
          f", protocol errors {res.protocol_errors}, exit code {res.fc_returncode}")


def write_log_csv(res, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(LOG_FIELDS)
        for row in zip(*(res.log[k] for k in LOG_FIELDS)):
            w.writerow([f"{v:.6g}" if isinstance(v, (float, np.floating)) else v for v in row])
    return path


def timing_stats(dts: np.ndarray) -> dict:
    return {"n": int(dts.size), "mean_s": float(dts.mean()), "std_s": float(dts.std(ddof=1)) if dts.size > 1 else 0.0,
            "min_s": float(dts.min()), "max_s": float(dts.max()),
            "abs_p95_s": float(np.percentile(np.abs(dts), 95)),
            "within_0.5s": float(np.mean(np.abs(dts) <= 0.5))}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(REPO_ROOT / "configs" / "default.json"))
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    ap.add_argument("--mode", default="baseline", choices=["baseline", "stub"])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--seeds", type=int, default=0, help="run this many seeds (seed, seed+1, ...) and report statistics")
    ap.add_argument("--fc-exe", default=None)
    args = ap.parse_args(argv)
    cfg = build_config(args)

    if args.seeds <= 0:
        res = run_sil(cfg, SilConfig(seed=args.seed, fc_exe=args.fc_exe, fc_mode=args.mode))
        print_single(res, args.mode)
        print(f"Log              : {write_log_csv(res, OUT_DIR / 'sil_log.csv').relative_to(REPO_ROOT)}")
        return 0

    t0 = time.perf_counter()
    dts, mech = [], []
    for k in range(args.seeds):
        res = run_sil(cfg, SilConfig(seed=args.seed + k, fc_exe=args.fc_exe, fc_mode=args.mode,
                                     pre_launch_s=3.0, post_landing_s=0.0))
        dts.append(res.flight.deployment.get("dt_from_apogee_s", np.nan))
        mech.append(f"{res.deploy_mechanism}:{res.fc_deploy_reason}")
    stats = timing_stats(np.array(dts))
    stats["mechanisms"] = {m: mech.count(m) for m in sorted(set(mech))}
    print(f"{args.seeds} SIL flights, mode {args.mode}, seeds {args.seed}..{args.seed + args.seeds - 1} "
          f"({time.perf_counter() - t0:.1f} s)")
    print(f"deploy - true apogee: mean {stats['mean_s']:+.3f} s, std {stats['std_s']:.3f} s, "
          f"min {stats['min_s']:+.3f} s, max {stats['max_s']:+.3f} s, |.| p95 {stats['abs_p95_s']:.3f} s, "
          f"within 0.5 s: {100 * stats['within_0.5s']:.0f} %")
    print(f"mechanisms: {stats['mechanisms']}")
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / f"deploy_timing_{args.mode}.json"
    path.write_text(json.dumps({"mode": args.mode, "seed0": args.seed, **stats,
                                "dt_from_apogee_s": dts}, indent=1), encoding="utf-8")
    print(f"saved {path.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
