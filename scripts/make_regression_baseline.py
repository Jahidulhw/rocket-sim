"""Record the default-C6 regression baseline into tests/data/c6_baseline.json.

    python scripts/make_regression_baseline.py

Run this ONLY to deliberately accept a new baseline (and say why in the
commit). It was first run on branch fleet-phase-a at main f28c3db, before any
Phase A change, so tests/test_regression_c6.py proves the fleet work leaves
the original rocket's physics and flight-computer results unchanged.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from sim.config import FlightConfig  # noqa: E402
from sim.faults import Fault  # noqa: E402
from sim.flight import simulate  # noqa: E402
from sim.montecarlo import MonteCarloConfig, run_monte_carlo  # noqa: E402
from sim.requirements import evaluate  # noqa: E402
from sim.sil import SilConfig, run_sil, sil_flight_config  # noqa: E402

OUT = REPO_ROOT / "tests" / "data" / "c6_baseline.json"
SAMPLE_TIMES = [0.5, 1.0, 2.0, 4.0, 7.0, 20.0, 60.0]


def flight_record(res) -> dict:
    import numpy as np
    d = res.deployment
    return {"apogee_m": res.apogee_m, "apogee_t": res.events["apogee"].t,
            "landing_xy": [float(v) for v in res.landing_point[:2]], "flight_time_s": res.flight_time_s,
            "max_speed_mps": res.max_speed_mps, "burnout_t": res.events["burnout"].t,
            "deploy_t": d["t"], "deploy_speed_mps": d["speed_mps"], "timing": d["timing"],
            "z_at": {str(t): float(np.interp(t, res.t, res.position[:, 2])) for t in SAMPLE_TIMES}}


def sil_record(res) -> dict:
    import numpy as np
    i = int(np.flatnonzero(res.log["fc_deploy"])[0]) if res.fc_deploy_t is not None else None
    return {"fc_deploy_t": res.fc_deploy_t, "reason": res.fc_deploy_reason, "mechanism": res.deploy_mechanism,
            "state_times": res.fc_state_times(), "health": [h.split("HEALTH ", 1)[1] for h in res.fc_health_events],
            "est_alt_at_deploy": None if i is None else float(res.log["est_alt"][i]),
            "verdicts": {rid: v.status for rid, v in evaluate(res).items()},
            "apogee_m": res.flight.apogee_m, "landing_xy": [float(v) for v in res.flight.landing_point[:2]]}


def main() -> int:
    base = FlightConfig.load(REPO_ROOT / "configs" / "default.json")
    out = {"generated_by": "scripts/make_regression_baseline.py",
           "commit": subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT,
                                    capture_output=True, text=True).stdout.strip()}
    out["open_loop"] = flight_record(simulate(base))
    mc = run_monte_carlo(base, MonteCarloConfig(n_runs=20, seed=7), workers=1)
    out["montecarlo_20"] = {"apogee_mean_m": mc.stats["apogee"]["mean_m"],
                            "landing_mean_m": mc.stats["landing"]["mean_m"],
                            "drift_p95_m": mc.stats["drift"]["p95_m"]}
    sil_cfg = sil_flight_config(base)
    out["sil_open_loop"] = flight_record(simulate(sil_cfg))
    out["sil"] = {
        "kalman_seed1": sil_record(run_sil(sil_cfg, SilConfig(seed=1, fc_mode="kalman"))),
        "baseline_seed1": sil_record(run_sil(sil_cfg, SilConfig(seed=1, fc_mode="baseline"))),
        "kalman_stuck_baro_seed2": sil_record(run_sil(sil_cfg, SilConfig(
            seed=2, fc_mode="kalman", faults=(Fault("stuck", 3.0, sensor="baro"),)))),
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(f"wrote {OUT.relative_to(REPO_ROOT)}")
    print(json.dumps({k: out[k] for k in ("open_loop", "montecarlo_20")}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
