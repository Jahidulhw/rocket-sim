"""Monte Carlo SIL campaign with faults (feeds scripts/verification_report.py).

    python scripts/run_sil_montecarlo.py                      # 500 runs x 2 detectors + 1000 pad sits
    python scripts/run_sil_montecarlo.py --n 100 --pad-sits 50 --workers 4

Each run: a dispersed flight (wind, impulse, Cd, mass, rail angle; same
dispersions as run_montecarlo.py), nominal or with one random fault, flown by
the Kalman FC and by the baseline FC on identical inputs. Results go to
out/sil_mc/results.json.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from sim.config import FlightConfig  # noqa: E402
from sim.sil import sil_flight_config  # noqa: E402
from sim.sil_montecarlo import (SilMcConfig, default_workers, replay_determinism,  # noqa: E402
                                requirement_rates, run_campaign, run_pad_sits, timing_by_class)

OUT = REPO_ROOT / "out" / "sil_mc" / "results.json"


def _progress(label):
    def p(i, n):
        if i == n or i % max(1, n // 20) == 0:
            print(f"\r  {label}: {i}/{n}", end="" if i < n else "\n", flush=True)
    return p


def git_commit() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT, capture_output=True,
                              text=True, timeout=10).stdout.strip() or "unknown"
    except OSError:
        return "unknown"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--seed", type=int, default=20261003)
    ap.add_argument("--fault-fraction", type=float, default=0.6)
    ap.add_argument("--pad-sits", type=int, default=1000, help="REQ-002 campaign size (60 s each)")
    ap.add_argument("--replay", type=int, default=10, help="runs re-flown to check determinism (REQ-006)")
    ap.add_argument("--workers", type=int, default=default_workers())
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args(argv)

    base = sil_flight_config(FlightConfig.load(REPO_ROOT / "configs" / "default.json"))
    mc = SilMcConfig(n_runs=args.n, seed=args.seed, fault_fraction=args.fault_fraction)
    t0 = time.perf_counter()
    print(f"SIL Monte Carlo: {args.n} runs x {len(mc.modes)} detectors, seed {args.seed}, {args.workers} workers")
    records = run_campaign(base, mc, workers=args.workers, progress=_progress("flights"))
    pads = run_pad_sits(args.pad_sits, args.seed, workers=args.workers, progress=_progress("pad sits")) \
        if args.pad_sits > 0 else []

    # REQ-006: replay the worst Kalman runs (largest |deploy error|, failures first), serially.
    def badness(r):
        m = r["modes"]["kalman"]
        failed = any(v[0] == "fail" for v in m["verdicts"].values())
        return (failed, abs(m["dt_s"]) if m["dt_s"] is not None else 1e9)
    worst = [r["run"] for r in sorted(records, key=badness, reverse=True)[:args.replay]]
    replays = replay_determinism(base, mc, records, worst) if args.replay > 0 else []

    summary = {"requirements_kalman": requirement_rates(records, "kalman"),
               "requirements_baseline": requirement_rates(records, "baseline"),
               "timing_kalman": timing_by_class(records, "kalman"),
               "timing_baseline": timing_by_class(records, "baseline"),
               "pad_sits": {"n": len(pads), "false_launches": sum(p["false_launch"] for p in pads),
                            "deploys": sum(p["deploy"] for p in pads),
                            "health_events": sum(len(p["health_events"]) for p in pads)},
               "determinism": {"replayed": len(replays), "identical": sum(r["identical"] for r in replays)},
               "watchdog_retries": [{"run": r["run"], "mode": m, "first_failure": d["watchdog_retry"],
                                     "retry_failed": d["fc_failed"]}
                                    for r in records for m, d in r["modes"].items() if d["watchdog_retry"]]}
    out = {"generated_by": "scripts/run_sil_montecarlo.py", "commit": git_commit(),
           "elapsed_s": time.perf_counter() - t0, "config": {**asdict(mc), "workers": args.workers},
           "base_config": base.to_dict(), "summary": summary, "runs": records, "pad_sits": pads,
           "replays": replays}
    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1, default=str), encoding="utf-8")

    print(f"\nDone in {out['elapsed_s']:.0f} s -> {path}")
    print("Per-run requirement pass rates (Kalman FC):")
    for rid, d in sorted(summary["requirements_kalman"].items()):
        rate = f"{100 * d['pass'] / d['applicable']:.1f} %" if d["applicable"] else "n/a"
        print(f"  {rid}: {d['pass']}/{d['applicable']} pass ({rate})")
    ps = summary["pad_sits"]
    print(f"  REQ-002: {ps['n'] - ps['false_launches']}/{ps['n']} pad sits without false launch")
    dd = summary["determinism"]
    print(f"  REQ-006: {dd['identical']}/{dd['replayed']} replays identical")
    wr = summary["watchdog_retries"]
    print(f"Unexpected watchdog trips re-run: {len(wr)} (reproduced: {sum(w['retry_failed'] for w in wr)})")
    print("Deploy timing, nominal runs:")
    for mode in ("kalman", "baseline"):
        t = summary[f"timing_{mode}"]["nominal"]
        print(f"  {mode:<8} mean {t['dt_mean_s']:+.3f} s, |dt| p95 {t['dt_abs_p95_s']:.3f} s, "
              f"REQ-001 {t['req_pass']}/{t['req_applicable']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
