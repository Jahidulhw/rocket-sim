"""SIL milestone 6: viewer export of SIL flights."""

from __future__ import annotations

import json

import pytest

from sim.export import sil_flight_to_dict
from sim.faults import Fault
from sim.protocol import FC_STATES
from sim.sil import SilConfig, run_sil


@pytest.fixture(scope="module")
def exported(fc_exe, sil_base_cfg):
    faults = (Fault("spike", 2.0, 10.0, "baro", 40.0, 0.08), Fault("dropout", 5.0, 0.6),
              Fault("stuck", 60.0, sensor="baro"))
    res = run_sil(sil_base_cfg, SilConfig(seed=1, fc_exe=str(fc_exe), faults=faults, post_landing_s=2.0))
    return res, sil_flight_to_dict(res, "kalman", label="SIL faults")


def test_export_is_strict_json(exported):
    _, d = exported
    json.dumps(d, allow_nan=False)    # NaN must have become null
    assert d["kind"] == "flight" and "trajectory" in d   # still a normal flight dataset


def test_series_columns_align_and_cover_the_flight(exported):
    res, d = exported
    s = d["sil"]["series"]
    n = len(s["t"])
    assert all(len(v) == n for v in s.values())
    assert s["t"][0] == pytest.approx(0.0, abs=1e-9)
    assert s["t"][-1] <= res.flight.flight_time_s + 1e-6
    assert max(b - a for a, b in zip(s["t"], s["t"][1:])) == pytest.approx(0.02, abs=1e-6)  # 50 Hz
    # -1 encodes "no FC reply on this tick", which must only happen when the frame was not sent.
    assert set(s["fc_state"]) <= set(range(-1, len(FC_STATES)))
    assert all(sent == 0 for st, sent in zip(s["fc_state"], s["sent"]) if st == -1)


def test_fc_state_transitions_and_deploy(exported):
    res, d = exported
    sil = d["sil"]
    assert [tr["state"] for tr in sil["transitions"]][:4] == ["BOOST", "COAST", "APOGEE", "DESCENT"]
    dep = sil["deploy"]
    assert dep["fc_t"] == pytest.approx(res.fc_deploy_t)
    assert dep["mechanism"] == res.deploy_mechanism and dep["reason"] == res.fc_deploy_reason
    assert abs(dep["dt_s"]) < 0.5


def test_fault_intervals_dropped_frames_and_health(exported):
    _, d = exported
    sil = d["sil"]
    labels = {f["label"]: f for f in sil["faults"]}
    assert labels["spike_baro"]["start"] == 2.0 and labels["spike_baro"]["end"] == 12.0
    assert labels["dropout"]["end"] == pytest.approx(5.6)
    assert labels["stuck_baro"]["end"] == pytest.approx(d["summary"]["flight_time_s"], abs=1e-3)  # inf clipped
    s = sil["series"]
    blackout = [sent for t, sent in zip(s["t"], s["sent"]) if 5.0 <= t < 5.6]
    assert blackout and not any(blackout)
    assert any(est is None for t, est in zip(s["t"], s["est_alt"]) if 5.0 <= t < 5.6)
    assert any("baro FAILED" in h["text"] for h in sil["health"])
