"""Milestone 4: JSON export consumed by the browser viewer."""

import json
import math

import numpy as np
import pytest

from sim.config import REPO_ROOT, FlightConfig
from sim.export import SAMPLE_RATE_HZ, flight_to_dict, write_json
from sim.flight import PHASES, simulate

DEFAULT_CONFIG = REPO_ROOT / "configs" / "default.json"


@pytest.fixture(scope="module")
def flight():
    cfg = FlightConfig.load(DEFAULT_CONFIG)
    cfg.launch.tilt_deg = 10.0  # make x and y both non-trivial
    cfg.launch.azimuth_deg = 30.0
    return simulate(cfg)


@pytest.fixture(scope="module")
def exported(flight, tmp_path_factory):
    # Round-trip through a real file so we test what the browser actually reads.
    path = tmp_path_factory.mktemp("export") / "flight.json"
    write_json(flight_to_dict(flight, label="test"), path)
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def test_top_level_structure(exported):
    assert exported["kind"] == "flight"
    assert exported["schema_version"] == 1
    for key in ("meta", "summary", "trajectory", "events"):
        assert key in exported
    meta = exported["meta"]
    assert meta["phases"] == PHASES
    assert meta["sample_rate_hz"] == SAMPLE_RATE_HZ
    assert meta["units"]["position"] == "m"
    assert meta["motor"]["name"] == "C6"
    # The config embedded in the file reproduces the flight inputs exactly.
    assert FlightConfig.from_dict(meta["config"]).to_dict() == meta["config"]


def test_trajectory_columns_have_equal_length_and_valid_values(exported):
    tr = exported["trajectory"]
    cols = ["t", "x", "y", "z", "vx", "vy", "vz", "phase"]
    n = len(tr["t"])
    assert all(len(tr[c]) == n for c in cols)
    assert all(isinstance(p, int) and 0 <= p < len(PHASES) for p in tr["phase"])
    assert all(math.isfinite(v) for c in cols[:-1] for v in tr[c])
    assert min(tr["z"]) >= 0.0


def test_time_strictly_increasing_and_spans_the_flight(exported, flight):
    t = np.array(exported["trajectory"]["t"])
    assert t[0] == 0.0
    assert np.all(np.diff(t) > 0.0)
    assert t[-1] == pytest.approx(flight.flight_time_s, abs=1e-6)


def test_sample_count_matches_simulation_duration(exported, flight):
    # Uniform 60 Hz grid strictly before landing (with a 1 ms guard), plus the
    # exact landing sample.
    T = flight.flight_time_s
    expected = math.ceil((T - 1e-3) * SAMPLE_RATE_HZ) + 1
    assert len(exported["trajectory"]["t"]) == expected
    dt = np.diff(exported["trajectory"]["t"][:-1])
    assert np.allclose(dt, 1.0 / SAMPLE_RATE_HZ, atol=2e-6)


def test_samples_match_simulated_trajectory(exported, flight):
    tr = exported["trajectory"]
    t = np.array(tr["t"])
    for col, k in (("x", 0), ("y", 1), ("z", 2)):
        sim_interp = np.interp(t, flight.t, flight.position[:, k])
        assert np.max(np.abs(np.array(tr[col]) - sim_interp)) < 1e-3
    end = flight.landing_point
    assert [tr["x"][-1], tr["y"][-1], tr["z"][-1]] == pytest.approx(list(end), abs=1e-3)


def test_phase_per_sample_matches_simulation(exported, flight):
    tr = exported["trajectory"]
    sim_idx = np.searchsorted(flight.t, np.array(tr["t"]), side="right") - 1
    expected = [PHASES.index(flight.phase[i]) for i in sim_idx]
    assert tr["phase"] == expected
    assert PHASES[tr["phase"][-1]] == "LANDED"


def test_events_match_simulation(exported, flight):
    ev = exported["events"]
    assert [e["name"] for e in ev] == sorted(flight.events, key=lambda n: flight.events[n].t)
    for e in ev:
        src = flight.events[e["name"]]
        assert e["t"] == pytest.approx(src.t, abs=1e-6)
        assert e["position"] == pytest.approx(list(src.position), abs=1e-3)
        assert e["velocity"] == pytest.approx(list(src.velocity), abs=1e-3)


def test_summary_matches_simulation(exported, flight):
    s = exported["summary"]
    assert s["apogee_m"] == pytest.approx(flight.apogee_m, abs=1e-3)
    assert s["landing_distance_m"] == pytest.approx(flight.landing_distance_m, abs=1e-3)
    assert s["deployment"]["timing"] == flight.deployment["timing"]


def test_export_rejects_flight_that_did_not_land(flight):
    cfg = flight.config.copy()
    cfg.sim.max_time_s = 3.0
    with pytest.raises(ValueError):
        flight_to_dict(simulate(cfg))
