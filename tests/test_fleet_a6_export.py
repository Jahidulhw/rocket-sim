"""Phase A, milestone A6: viewer datasets for the fleet."""

from __future__ import annotations

import json

import numpy as np
import pytest

from sim.export import LONG_FLIGHT_S, fleet_flight_to_dict
from sim.fleet import load_fleet
from sim.sil import SilConfig, run_sil_preset

FLEET = {p.id: p for p in load_fleet()}


@pytest.fixture(scope="module")
def argo(fc_exe):
    p = FLEET["argo-2stage"]
    res = run_sil_preset(p, SilConfig(seed=1, fc_exe=str(fc_exe), pre_launch_s=3.0, post_landing_s=12.0))
    return res, fleet_flight_to_dict(res, p, label=p.label)


def test_fleet_dataset_is_strict_json_with_geometry(argo):
    _, d = argo
    json.dumps(d, allow_nan=False)
    assert d["kind"] == "flight" and "sil" in d
    r = d["rocket"]
    assert r["two_stage"] and set(r["geometry"]) == {"sustainer", "booster"}
    assert r["geometry"]["sustainer"]["nose"]["shape"] == "ogive"


def test_booster_track_lands_separately(argo):
    res, d = argo
    b = d["booster"]
    n = len(b["t"])
    assert all(len(b[k]) == n for k in ("x", "y", "z", "vx", "vy", "vz"))
    assert b["z"][-1] == pytest.approx(0.0, abs=1e-6)                       # it lands
    names = [e["name"] for e in b["events"]]
    assert "separation" in names and "landing" in names and "deploy" in names
    assert abs(b["t"][-1] - d["summary"]["flight_time_s"]) > 60.0          # at a different time
    assert any(e["name"] == "separation" for e in d["events"])


def test_long_flights_are_downsampled(argo):
    res, d = argo
    assert res.flight.flight_time_s > LONG_FLIGHT_S
    dt = np.diff(d["trajectory"]["t"][:100])
    assert dt.max() == pytest.approx(1 / 20, abs=1e-3)                     # 20 Hz, not 60
    sdt = np.diff(d["sil"]["series"]["t"][:100])
    assert sdt.max() == pytest.approx(1 / 25, abs=1e-3)                    # 25 Hz, not 50
    assert len(json.dumps(d)) < 1.5e6                                       # ~1 MB


def test_short_flights_keep_full_rate(fc_exe):
    p = FLEET["sparrow-b6"]
    res = run_sil_preset(p, SilConfig(seed=1, fc_exe=str(fc_exe), pre_launch_s=2.0, post_landing_s=0.0))
    d = fleet_flight_to_dict(res, p, label=p.label)
    assert res.flight.flight_time_s < LONG_FLIGHT_S
    assert np.diff(d["trajectory"]["t"][:50]).max() == pytest.approx(1 / 60, abs=1e-4)
    assert "booster" not in d and d["rocket"]["two_stage"] is False
