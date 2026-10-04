"""Fleet-wide SIL requirements campaign: every preset, nominal + single faults
+ pad sits + determinism replays, each with its own FC configuration and
sensor model.

    python scripts/run_sil_fleet.py                     # 500 runs + 1000 pad sits per preset
    python scripts/run_sil_fleet.py --n 40 --pad-sits 40 --presets swift-k940

Results: out/sil_fleet/<preset>.json and out/sil_fleet/summary.json, read by
scripts/verification_report.py for its fleet section. (The baseline-vs-Kalman
detector comparison stays in the C6 campaign, scripts/run_sil_montecarlo.py.)
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from sim.fleet import load_fleet, preset_motors  # noqa: E402
from sim.sil_montecarlo import (default_workers, preset_campaign_config, replay_determinism,  # noqa: E402
                                requirement_rates, run_campaign, run_pad_sits, timing_by_class)

OUT = REPO_ROOT / "out" / "sil_fleet"


def _progress(label):
    def p(i, n):
        if i == n or i % max(1, n // 10) == 0:
            print(f"\r    {label}: {i}/{n}", end="" if i < n else "\n", flush=True)
    return p


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--seed", type=int, default=20261004)
    ap.add_argument("--pad-sits", type=int, default=1000)
    ap.add_argument("--replay", type=int, default=5)
    ap.add_argument("--workers", type=int, default=default_workers())
    ap.add_argument("--presets", nargs="*", default=None)
    args = ap.parse_args(argv)

    fleet = [p for p in load_fleet() if not args.presets or p.id in args.presets]
    OUT.mkdir(parents=True, exist_ok=True)
    summary = {"seed": args.seed, "n_runs": args.n, "pad_sits": args.pad_sits, "presets": {}}
    t_all = time.perf_counter()
    for preset in fleet:
        t0 = time.perf_counter()
        print(f"  {preset.id}: {args.n} runs, {args.pad_sits} pad sits")
        base, mc = preset_campaign_config(preset.id, args.n, args.seed)
        records = run_campaign(base, mc, workers=args.workers, progress=_progress("flights"))
        pads = run_pad_sits(args.pad_sits, args.seed, workers=args.workers, progress=_progress("pad sits"),
                            preset_id=preset.id) if args.pad_sits else []

        def badness(r):
            m = r["modes"]["kalman"]
            return (any(v[0] == "fail" for v in m["verdicts"].values()), abs(m["dt_s"]) if m["dt_s"] is not None else 1e9)
        worst = [r["run"] for r in sorted(records, key=badness, reverse=True)[:args.replay]]
        replays = replay_determinism(base, mc, records, worst) if args.replay else []
        s = {"label": preset.label, "category": preset.category, "motors": preset_motors(preset),
             "two_stage": preset.two_stage, "fault_time_scale": mc.fault_time_scale,
             "requirements": requirement_rates(records, "kalman"),
             "timing": timing_by_class(records, "kalman"),
             "pad_sits": {"n": len(pads), "false_launches": sum(p["false_launch"] for p in pads),
                          "deploys": sum(p["deploy"] for p in pads),
                          "health_events": sum(len(p["health_events"]) for p in pads)},
             "determinism": {"replayed": len(replays), "identical": sum(r["identical"] for r in replays)},
             "watchdog_retries": sum(1 for r in records for d in r["modes"].values() if d["watchdog_retry"]),
             "elapsed_s": time.perf_counter() - t0}
        summary["presets"][preset.id] = s
        (OUT / f"{preset.id}.json").write_text(json.dumps({"summary": s, "config": asdict(mc), "runs": records,
                                                           "replays": replays}, indent=1, default=str),
                                               encoding="utf-8")
        fails = {rid: d["fail"] for rid, d in s["requirements"].items() if d["fail"]}
        print(f"    done in {s['elapsed_s']:.0f} s; failures: {fails or 'none'}; pad false launches "
              f"{s['pad_sits']['false_launches']}; replays {s['determinism']['identical']}/{s['determinism']['replayed']}")
    summary["elapsed_s"] = time.perf_counter() - t_all
    (OUT / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print(f"\nFleet campaign done in {summary['elapsed_s'] / 60:.1f} min -> {OUT / 'summary.json'}")
    rids = sorted({rid for s in summary["presets"].values() for rid in s["requirements"]})
    print(f"{'preset':14}" + "".join(f"{r[-3:]:>9}" for r in rids))
    for pid, s in summary["presets"].items():
        row = ""
        for rid in rids:
            d = s["requirements"][rid]
            row += f"{(str(d['pass']) + '/' + str(d['applicable'])) if d['applicable'] else '-':>9}"
        print(f"{pid:14}{row}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
