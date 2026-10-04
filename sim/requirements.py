"""Flight-computer requirements: IDs, text, and per-run verification checks.

docs/requirements.md is the human-readable version (with rationale). Tests
are tagged with the IDs they verify: pytest `@pytest.mark.req("REQ-003")`,
GoogleTest names ending in `_REQ003`.

Per-run checks take a SilResult and return a Verdict. A check that does not
apply to a run (e.g. the nominal-accuracy requirement on a faulted run)
returns "n/a", never "pass".
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

NOMINAL_WINDOW_S = 0.5    # REQ-001
FAULT_WINDOW_S = 1.5      # REQ-004
LANDING_WINDOW_S = 10.0   # REQ-010
MAIN_BAND_M = 25.0        # REQ-013

REQUIREMENTS = {
    "REQ-001": "Nominal flight: the FC deploys within 0.5 s of true apogee.",
    "REQ-002": "Zero false launch detections across 1000 seeded 60 s pad sits.",
    "REQ-003": "The FC never commands deployment while the rocket is on the pad or under thrust, under any fault.",
    "REQ-004": "Under any single sensor or link fault, the FC deploys within 1.5 s of true apogee or via its "
               "backup timer; a run where the FC never deploys fails.",
    "REQ-005": "An FC hang is detected by the watchdog and the parachute is still deployed by an independent "
               "backup (motor ejection charge, or a pre-set backup timer on rockets flying plugged motors).",
    "REQ-006": "FC output is deterministic: identical input gives identical output.",
    "REQ-007": "A malformed, garbled or stale FC reply never causes a deployment.",
    "REQ-008": "The FC answers every input line with exactly one line and survives malformed input.",
    "REQ-009": "On a nominal flight the FC deploys from apogee detection, never from its backup timer.",
    "REQ-010": "On a nominal flight the FC reports LANDED within 10 s of touchdown.",
    "REQ-011": "On a multi-burn (staged) rocket the FC never commands any deployment from lift-off until the "
               "final motor burnout: not during either boost nor the inter-stage gap, under any fault.",
    "REQ-012": "On a multi-burn rocket the FC never mistakes staging for apogee or landing: it reports no "
               "APOGEE, DESCENT or LANDED state before the final motor burnout.",
    "REQ-013": "On a nominal dual-deploy flight the FC commands the main chute after the drogue, during descent, "
               "within 25 m of the configured main deploy altitude.",
}

SENSOR_FAULT_KINDS = ("dropout", "stuck", "spike", "drift")


@dataclass(frozen=True)
class Verdict:
    status: str          # "pass" | "fail" | "n/a"
    detail: str = ""

    @property
    def passed(self) -> bool:
        return self.status == "pass"


def _na(why: str) -> Verdict:
    return Verdict("n/a", why)


def _ok(cond: bool, detail: str) -> Verdict:
    return Verdict("pass" if cond else "fail", detail)


def true_apogee_t(res) -> float:
    d = res.flight.deployment
    if d.get("true_apogee_t") is not None:
        return float(d["true_apogee_t"])
    return float(res.flight.events["apogee"].t)


def fault_class(res) -> str:
    """'nominal', 'hang', a single sensor/link fault label, or 'multiple'."""
    if not res.faults:
        return "nominal"
    if len(res.faults) > 1:
        return "multiple"
    return res.faults[0].label


def check_req001(res) -> Verdict:
    if res.faults:
        return _na("faulted run")
    if res.fc_deploy_t is None:
        return Verdict("fail", "FC never commanded deploy")
    dt = res.fc_deploy_t - true_apogee_t(res)
    return _ok(abs(dt) <= NOMINAL_WINDOW_S and res.fc_deploy_reason == "apogee",
               f"FC deploy {dt:+.3f} s vs true apogee ({res.fc_deploy_reason})")


def check_req003(res) -> Verdict:
    ev = res.flight.events
    if "burnout" not in ev:
        return _na("no burnout in flight")
    t_burnout = ev["burnout"].t
    log = res.log
    cmd = log["fc_deploy"].astype(bool)
    early = cmd & (log["t"] < t_burnout)
    in_locked_state = cmd & np.isin(log["fc_state"], ["PAD", "BOOST"])
    phys = res.flight.events.get("deploy")
    phys_early = phys is not None and phys.t < t_burnout
    bad = early.any() or in_locked_state.any() or phys_early
    detail = (f"first FC command t={res.fc_deploy_t:.3f} s, burnout t={t_burnout:.3f} s"
              if res.fc_deploy_t is not None else f"no FC command; burnout t={t_burnout:.3f} s")
    return _ok(not bad, detail)


def check_req004(res) -> Verdict:
    if len(res.faults) != 1 or res.faults[0].kind not in SENSOR_FAULT_KINDS:
        return _na("not a single sensor/link fault run")
    if res.fc_deploy_t is None:
        return Verdict("fail", "FC never commanded deploy")
    dt = res.fc_deploy_t - true_apogee_t(res)
    reason = res.fc_deploy_reason
    return _ok(abs(dt) <= FAULT_WINDOW_S or reason == "backup_timer",
               f"FC deploy {dt:+.3f} s vs true apogee via {reason}")


def check_req005(res) -> Verdict:
    hangs = [f for f in res.faults if f.kind == "hang"]
    if not hangs:
        return _na("no FC hang injected")
    t_hang = hangs[0].start_s
    detected = res.fc_failed and res.failure_reason is not None and "watchdog_timeout" in res.failure_reason
    deployed = res.flight.deployment.get("deployed", False)
    fc_before = res.fc_deploy_t is not None and res.fc_deploy_t < t_hang
    # If the FC had not already deployed before hanging, the chute must have
    # come from an independent backup (motor charge or pre-set timer device).
    mech_ok = fc_before or res.deploy_mechanism in ("motor", "timer")
    return _ok(detected and deployed and mech_ok and res.flight.landed,
               f"watchdog {'tripped at %.2f s' % res.failure_t if detected else 'did NOT trip'}, "
               f"deployed by {res.deploy_mechanism}")


def check_req009(res) -> Verdict:
    if res.faults:
        return _na("faulted run")
    return _ok(res.fc_deploy_reason == "apogee", f"FC deploy reason: {res.fc_deploy_reason}")


def check_req010(res) -> Verdict:
    if res.faults:
        return _na("faulted run")
    if not res.flight.landed:
        return Verdict("fail", "flight did not land")
    t_landed = res.fc_state_times().get("LANDED")
    t_touch = res.flight.events["landing"].t
    if t_landed is None:
        last = float(res.log["t"][-1])
        if last - t_touch < LANDING_WINDOW_S:
            return _na(f"run ended {last - t_touch:.1f} s after touchdown (needs {LANDING_WINDOW_S:g} s)")
        return Verdict("fail", "LANDED never reported")
    return _ok(t_landed - t_touch <= LANDING_WINDOW_S, f"LANDED {t_landed - t_touch:+.2f} s after touchdown")


def _multi_burn(res) -> bool:
    return "booster_burnout" in res.flight.events


def check_req011(res) -> Verdict:
    if not _multi_burn(res):
        return _na("single-burn rocket")
    t_final = res.flight.events["burnout"].t
    log = res.log
    early = (log["fc_deploy"].astype(bool) | log["fc_main"].astype(bool)) & (log["t"] < t_final)
    t_sep = res.flight.events["booster_burnout"].t
    return _ok(not early.any(), f"final burnout {t_final:.2f} s (staging {t_sep:.2f} s); first FC command "
               + (f"{res.fc_deploy_t:.2f} s" if res.fc_deploy_t is not None else "none"))


def check_req012(res) -> Verdict:
    if not _multi_burn(res):
        return _na("single-burn rocket")
    t_final = res.flight.events["burnout"].t
    log = res.log
    bad = np.isin(log["fc_state"], ["APOGEE", "DESCENT", "LANDED"]) & (log["t"] < t_final)
    return _ok(not bad.any(), "no apogee/descent/landed before the final burnout" if not bad.any()
               else f"{log['fc_state'][bad][0]} reported at {log['t'][bad][0]:.2f} s, before final burnout {t_final:.2f} s")


def check_req013(res) -> Verdict:
    if res.faults:
        return _na("faulted run")
    alt = getattr(res, "main_altitude_m", None)
    if alt is None:
        return _na("single-deploy rocket")
    if res.fc_main_t is None:
        return Verdict("fail", "FC never commanded the main")
    log = res.log
    i = int(np.flatnonzero(log["fc_main"])[0])
    z, vz = float(log["z_true"][i]), float(log["vz_true"][i])
    after_drogue = res.fc_deploy_t is not None and res.fc_deploy_t <= res.fc_main_t
    return _ok(after_drogue and vz < 0 and abs(z - alt) <= MAIN_BAND_M,
               f"main commanded at true altitude {z:.1f} m (configured {alt:.0f} m), descending {-vz:.1f} m/s")


RUN_CHECKS = {"REQ-001": check_req001, "REQ-003": check_req003, "REQ-004": check_req004,
              "REQ-005": check_req005, "REQ-009": check_req009, "REQ-010": check_req010,
              "REQ-011": check_req011, "REQ-012": check_req012, "REQ-013": check_req013}


def evaluate(res) -> dict:
    """Verdict for every per-run requirement (REQ-002/006/007/008 need
    dedicated experiments or tests, not a single flight)."""
    return {rid: fn(res) for rid, fn in RUN_CHECKS.items()}


def logs_identical(a, b, fields=("t", "baro", "accel", "fc_state", "est_alt", "est_vel", "fc_deploy")) -> bool:
    """REQ-006 helper: two runs of identical input produced identical FC output."""
    for k in fields:
        x, y = a.log[k], b.log[k]
        if x.shape != y.shape:
            return False
        if x.dtype.kind == "f":
            if not np.array_equal(x, y, equal_nan=True):
                return False
        elif not np.array_equal(x, y):
            return False
    return True
