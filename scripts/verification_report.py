"""Generate docs/verification_report.md from the SIL Monte Carlo results and
the requirement-tagged test results.

    python scripts/run_sil_montecarlo.py      # first: out/sil_mc/results.json
    python scripts/verification_report.py     # runs tagged tests, writes the report
    python scripts/verification_report.py --skip-tests   # reuse out/verify/*.xml

Evidence per requirement:
  * per-run checks over the Monte Carlo campaign (sim/requirements.py),
  * the REQ-002 pad-sit campaign and REQ-006 replays,
  * every pytest test marked @pytest.mark.req(...) and every GoogleTest whose
    name ends in _REQnnn (traceability matrix, with actual results).
"""

from __future__ import annotations

import argparse
import json
import re
import shlex
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from sim.faults import Fault  # noqa: E402
from sim.requirements import REQUIREMENTS  # noqa: E402
from sim.sil import BUILD_HINT, find_fc_executable  # noqa: E402

RESULTS = REPO_ROOT / "out" / "sil_mc" / "results.json"
VERIFY_DIR = REPO_ROOT / "out" / "verify"
REPORT = REPO_ROOT / "docs" / "verification_report.md"
PER_RUN = ("REQ-001", "REQ-003", "REQ-004", "REQ-005", "REQ-009", "REQ-010")
CLASS_LABEL = {"nominal": "Nominal (no fault)", "dropout_blackout": "Link blackout", "dropout_random": "Random frame loss",
               "stuck_baro": "Stuck barometer", "stuck_accel": "Stuck accelerometer", "spike_baro": "Barometer spikes",
               "spike_accel": "Accelerometer spikes", "drift_baro": "Barometer drift",
               "drift_accel": "Accelerometer drift", "hang": "FC hang"}


# ------------------------------------------------------------ test runs --

def run_tests() -> None:
    VERIFY_DIR.mkdir(parents=True, exist_ok=True)
    gtest = find_fc_executable().parent / ("fc_tests.exe" if sys.platform == "win32" else "fc_tests")
    if not gtest.is_file():
        sys.exit(f"GoogleTest binary not found next to the FC ({gtest}); {BUILD_HINT}")
    print("Running GoogleTest suite ...")
    subprocess.run([str(gtest), f"--gtest_output=xml:{VERIFY_DIR / 'gtest.xml'}"], cwd=REPO_ROOT,
                   capture_output=True)
    print("Running requirement-tagged pytest tests ...")
    subprocess.run([sys.executable, "-m", "pytest", "-q", "-m", "req", f"--junitxml={VERIFY_DIR / 'pytest.xml'}"],
                   cwd=REPO_ROOT, capture_output=True)


def _outcome(case: ET.Element) -> str:
    if case.find("failure") is not None or case.find("error") is not None:
        return "fail"
    if case.find("skipped") is not None or case.get("status") == "notrun":
        return "skipped"
    return "pass"


def collect_tests() -> dict:
    """REQ id -> list of (framework, test name, outcome)."""
    trace: dict[str, list] = {rid: [] for rid in REQUIREMENTS}
    gx = VERIFY_DIR / "gtest.xml"
    if gx.is_file():
        for case in ET.parse(gx).getroot().iter("testcase"):
            name = f"{case.get('classname')}.{case.get('name')}"
            for num in re.findall(r"_REQ(\d{3})", case.get("name", "")):
                trace.setdefault(f"REQ-{num}", []).append(("GoogleTest", name, _outcome(case)))
    px = VERIFY_DIR / "pytest.xml"
    if px.is_file():
        for case in ET.parse(px).getroot().iter("testcase"):
            reqs = [p.get("value") for p in case.iter("property") if p.get("name") == "req"]
            name = f"{case.get('classname', '').split('.')[-1]}::{case.get('name')}"
            for rid in reqs:
                trace.setdefault(rid, []).append(("pytest", name, _outcome(case)))
    return trace


# --------------------------------------------------------------- report --

def pct(a: int, b: int) -> str:
    return f"{100.0 * a / b:.1f} %" if b else "n/a"


def replay_cmd(rec: dict, mode: str = "kalman") -> str:
    parts = ["python", "scripts/run_sil.py", "--mode", mode, "--seed", str(rec["sensor_seed"]),
             "--pre-launch", f"{rec['pre_launch_s']:g}", "--post-landing", f"{rec['post_landing_s']:g}"]
    if rec["fault"]:
        parts += ["--fault", Fault.from_dict(rec["fault"]).to_spec()]
    for k, v in rec["overrides"].items():
        parts += ["--set", f"{k}={v:.10g}"]
    return " ".join(shlex.quote(p) for p in parts)


def fault_desc(rec: dict) -> str:
    if not rec["fault"]:
        return "none"
    f = Fault.from_dict(rec["fault"])
    s = f"{f.label} @ {f.start_s:.2f} s"
    if f.kind not in ("hang",) and f.duration_s != float("inf"):
        s += f" for {f.duration_s:.2f} s"
    if f.kind in ("spike", "drift"):
        s += f", mag {f.magnitude:+.2f}"
    if f.kind in ("spike", "dropout") and f.probability < 1.0:
        s += f", p {f.probability:.2f}"
    return s


def build_report(res: dict, trace: dict) -> str:
    S = res["summary"]
    cfg = res["config"]
    runs = res["runs"]
    n_faulted = sum(1 for r in runs if r["fault"])
    L = []
    w = L.append
    w("# Flight computer verification report\n")
    w(f"*Generated by `scripts/verification_report.py` on {time.strftime('%Y-%m-%d %H:%M')} from commit "
      f"`{res['commit']}`. Do not edit by hand: re-run `scripts/run_sil_montecarlo.py` and this script.*\n")
    w(f"**Campaign:** {cfg['n_runs']} seeded SIL flights (seed {cfg['seed']}), {cfg['n_runs'] - n_faulted} nominal "
      f"and {n_faulted} with one random fault. Every flight was flown by **both** detectors (Kalman = the "
      f"default FC, and the raw-barometer baseline) on identical inputs. The dispersions are the same as "
      f"`run_montecarlo.py`: wind speed and direction, motor impulse, Cd, dry mass and rail angle. Pad sits: "
      f"{S['pad_sits']['n']} × 60 s. Campaign time: {res['elapsed_s'] / 60:.1f} min on {cfg['workers']} workers.\n")
    w("Requirement texts and rationale: [requirements.md](requirements.md). Design reasoning: "
      "[fc_walkthrough.md](fc_walkthrough.md).\n")

    # ---- verdicts
    w("## 1. Requirement verdicts (default Kalman FC)\n")
    w("| Requirement | Evidence | Result | Verdict |")
    w("|---|---|---|---|")
    rk = S["requirements_kalman"]
    overall_ok = True

    def tests_ok(rid):
        rows = trace.get(rid, [])
        return rows and all(o == "pass" for _, _, o in rows), sum(o == "pass" for _, _, o in rows), len(rows)

    for rid in REQUIREMENTS:
        t_ok, t_pass, t_n = tests_ok(rid)
        ev, result, ok = [], [], True
        if rid in PER_RUN:
            d = rk[rid]
            ev.append(f"{d['applicable']} Monte Carlo runs")
            result.append(f"{d['pass']}/{d['applicable']} pass ({pct(d['pass'], d['applicable'])})")
            ok &= d["applicable"] > 0 and d["fail"] == 0
        if rid == "REQ-002":
            ps = S["pad_sits"]
            ev.append(f"{ps['n']} pad sits × 60 s")
            result.append(f"{ps['false_launches']} false launches, {ps['health_events']} false sensor failures")
            ok &= ps["n"] >= 1000 and ps["false_launches"] == 0 and ps["deploys"] == 0
        if rid == "REQ-006":
            dd = S["determinism"]
            ev.append(f"{dd['replayed']} worst-case replays")
            result.append(f"{dd['identical']}/{dd['replayed']} bit-identical")
            ok &= dd["replayed"] > 0 and dd["identical"] == dd["replayed"]
        ev.append(f"{t_n} tagged tests")
        result.append(f"{t_pass}/{t_n} tests pass")
        ok &= bool(t_ok)
        overall_ok &= ok
        w(f"| **{rid}** {REQUIREMENTS[rid]} | {'; '.join(ev)} | {'; '.join(result)} | "
          f"{'✅ PASS' if ok else '❌ FAIL'} |")
    w("")
    w(f"**Overall: {'all requirements verified' if overall_ok else 'NOT all requirements verified (see above)'}.**\n")

    # ---- baseline vs kalman
    w("## 2. Baseline vs Kalman (identical inputs)\n")
    w("Deploy error = FC deploy command time − true apogee time. |dt| p95 is the 95th percentile of "
      "|error| over runs where that FC commanded deployment. \"Req\" is the class's governing requirement "
      "(REQ-001 for nominal runs, REQ-005 for hangs, REQ-004 otherwise).\n")
    w("| Fault class | Runs | Kalman mean | Kalman \\|dt\\| p95 | Kalman timer | Kalman req | "
      "Baseline mean | Baseline \\|dt\\| p95 | Baseline req |")
    w("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    tk, tb = S["timing_kalman"], S["timing_baseline"]
    for cls, k in tk.items():
        b = tb.get(cls, {})
        fm = lambda v, sgn=True: "–" if v != v else (f"{v:+.3f} s" if sgn else f"{v:.3f} s")  # noqa: E731
        w(f"| {CLASS_LABEL.get(cls, cls)} | {k['n']} | {fm(k['dt_mean_s'])} | {fm(k['dt_abs_p95_s'], False)} | "
          f"{k['timer_deploys']} | {k['req_pass']}/{k['req_applicable']} | {fm(b.get('dt_mean_s', float('nan')))} | "
          f"{fm(b.get('dt_abs_p95_s', float('nan')), False)} | {b.get('req_pass', 0)}/{b.get('req_applicable', 0)} |")
    w("")
    rb = S["requirements_baseline"]
    w("Per-run requirement pass rates for the **baseline** FC (for comparison only; requirements apply to the "
      "default FC): " + ", ".join(f"{rid} {rb[rid]['pass']}/{rb[rid]['applicable']}" for rid in PER_RUN) + ".\n")
    w("**Reading it:**")
    w("* On nominal flights, the Kalman FC deploys about 0.05 s after apogee. The baseline deploys about "
      "0.5 s after, and misses REQ-001's 0.5 s window on most flights.")
    w("* Under faults the gap widens. The baseline has no outlier defence (one barometer spike inflates its "
      "running maximum and it deploys seconds early) and no stuck-sensor isolation.")
    w("* \"Kalman timer\" counts runs where the FC detected a fault it couldn't isolate and deployed by its "
      "backup timer. That's the designed safe outcome (walkthrough §8.4).\n")

    # ---- mechanisms
    mech = {}
    for r in runs:
        m = r["modes"]["kalman"]
        key = f"{m['mechanism']}" + (f" ({m['reason']})" if m["mechanism"] == "fc" else "")
        mech[key] = mech.get(key, 0) + 1
    w("## 3. Which mechanism deployed the chute (Kalman FC)\n")
    w("| Mechanism | Runs |")
    w("|---|---:|")
    for k, v in sorted(mech.items(), key=lambda kv: -kv[1]):
        w(f"| {k} | {v} |")
    w("\nThe motor charge (C6-7, about 1.4 s after apogee) deploys first only when the FC hung before apogee "
      "(REQ-005), or in the rare case where the FC's own backup timer fires later than the charge.\n")

    # ---- traceability
    w("## 4. Traceability matrix\n")
    w("Requirement → verifying tests → result, read from the JUnit/GoogleTest XML of this run. Parametrised "
      "pytest cases are listed individually.\n")
    for rid in REQUIREMENTS:
        rows = trace.get(rid, [])
        n_pass = sum(o == "pass" for _, _, o in rows)
        w(f"<details><summary><b>{rid}</b>: {len(rows)} tests, {n_pass} pass</summary>\n")
        w("| Framework | Test | Result |")
        w("|---|---|---|")
        for fw, name, o in sorted(rows):
            w(f"| {fw} | `{name}` | {'✅' if o == 'pass' else ('⚠️ skipped' if o == 'skipped' else '❌')} {o} |")
        if rid in PER_RUN:
            w(f"| Monte Carlo | per-run check `sim/requirements.py` | {rk[rid]['pass']}/{rk[rid]['applicable']} |")
        w("\n</details>\n")

    # ---- worst cases
    w("## 5. Worst cases (replayable)\n")
    w("The ten Kalman-FC runs with the largest |deploy error| (any failure first). Each command reproduces the "
      "run exactly: the same sensor seed, fault and dispersion.\n")

    def badness(r):
        m = r["modes"]["kalman"]
        failed = any(v[0] == "fail" for v in m["verdicts"].values())
        return (failed, abs(m["dt_s"]) if m["dt_s"] is not None else 1e9)
    for r in sorted(runs, key=badness, reverse=True)[:10]:
        m = r["modes"]["kalman"]
        fails = [rid for rid, v in m["verdicts"].items() if v[0] == "fail"]
        dt = "no FC deploy" if m["dt_s"] is None else f"{m['dt_s']:+.3f} s"
        w(f"* **Run {r['run']}**: {CLASS_LABEL.get(r['fault_class'], r['fault_class'])} ({fault_desc(r)}). FC "
          f"deploy {dt} via {m['reason']}, chute by {m['mechanism']}"
          f"{', health: ' + '; '.join(m['health']) if m['health'] else ''}"
          f"{', **FAILS ' + ', '.join(fails) + '**' if fails else ''}.")
        w(f"  `{replay_cmd(r)}`")
    w("")
    wr = S.get("watchdog_retries", [])
    w("## 6. Harness notes\n")
    w("* The watchdog uses wall-clock time, the only wall-clock element in the loop. A first campaign with "
      "a 1.0 s timeout falsely declared one healthy FC failed under heavy host load: a replay of the run was "
      "clean. The timeout is now 3 s, and any watchdog trip on a run *without* an injected hang is re-run "
      "once. The FC is deterministic, so a genuine hang reproduces and a host artefact doesn't.")
    w(f"* Unexpected watchdog trips in this campaign: **{len(wr)}** "
      f"(reproduced on re-run: {sum(x['retry_failed'] for x in wr)})."
      + "".join(f" Run {x['run']} ({x['mode']}): {x['first_failure']}." for x in wr) + "\n")
    w("## 7. Limits of this verification\n")
    w("* It is SIL, not HIL. Lockstep doesn't exercise real-time deadlines or the CPU budget.")
    w("* Sensor models and fault magnitudes are engineering assumptions (see `sim/sensors.py` and "
      "`sim/faults.py`). They are not measured hardware data.")
    w("* Single faults only. Double faults are outside the requirements (walkthrough §8.8).")
    w("* Pass rates are for this campaign's sample size. Zero failures in N runs bounds the per-run failure "
      f"probability below about 3/N at 95 % confidence (the \"rule of three\"): for N = {cfg['n_runs']}, "
      f"below {3 / cfg['n_runs']:.1%}.")
    return "\n".join(L) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default=str(RESULTS))
    ap.add_argument("--skip-tests", action="store_true", help="reuse existing out/verify/*.xml")
    ap.add_argument("--out", default=str(REPORT))
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):  # Windows consoles default to cp1252
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    rp = Path(args.results)
    if not rp.is_file():
        sys.exit(f"{rp} not found: run scripts/run_sil_montecarlo.py first")
    if not args.skip_tests:
        run_tests()
    res = json.loads(rp.read_text(encoding="utf-8"))
    trace = collect_tests()
    text = build_report(res, trace)
    Path(args.out).write_text(text, encoding="utf-8")
    n_tests = sum(len(v) for v in trace.values())
    print(f"Wrote {Path(args.out).relative_to(REPO_ROOT)} ({n_tests} requirement-test links)")
    print(text.split("## 2.")[0].split("## 1.")[1])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
