"""Config loading/validation: bad inputs must fail loudly, not silently."""

import json

import pytest

from sim.config import REPO_ROOT, FlightConfig, apply_overrides

DEFAULT_CONFIG = REPO_ROOT / "configs" / "default.json"


def test_default_config_round_trips():
    cfg = FlightConfig.load(DEFAULT_CONFIG)
    assert FlightConfig.from_dict(cfg.to_dict()) == cfg
    with open(DEFAULT_CONFIG, encoding="utf-8") as fh:
        assert cfg.to_dict() == json.load(fh)   # file documents every field explicitly


def test_unknown_keys_and_sections_rejected():
    with pytest.raises(ValueError):
        FlightConfig.from_dict({"rocket": {"cd ": 0.5}})
    with pytest.raises(ValueError):
        FlightConfig.from_dict({"rockets": {}})


@pytest.mark.parametrize("override", ["sim.dt_s=0", "launch.tilt_deg=95", "sim.integrator=\"verlet\"",
                                      "rocket.dry_mass_kg=-1", "wind.speed_mps=-2"])
def test_invalid_values_rejected(override):
    with pytest.raises(ValueError):
        apply_overrides(FlightConfig.load(DEFAULT_CONFIG), [override])


def test_overrides_parse_json_values_and_do_not_mutate_input():
    cfg = FlightConfig.load(DEFAULT_CONFIG)
    new = apply_overrides(cfg, ["wind.speed_mps=6.5", "recovery.deploy_delay_s=null",
                                "sim.integrator=euler"])
    assert new.wind.speed_mps == 6.5
    assert new.recovery.deploy_delay_s is None
    assert new.sim.integrator == "euler"
    assert cfg.wind.speed_mps == 2.0
    with pytest.raises(ValueError):
        apply_overrides(cfg, ["wind.gust=3"])
