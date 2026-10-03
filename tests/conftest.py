"""Shared fixtures for the SIL tests.

SIL tests need the compiled flight computer. If it is missing they FAIL with
build instructions rather than skip: a silently skipped SIL suite would let
CI go green without ever running the flight computer.
"""

from __future__ import annotations

import pytest

from sim.config import REPO_ROOT, FlightConfig
from sim.sil import BUILD_HINT, find_fc_executable, sil_flight_config


@pytest.fixture(scope="session")
def fc_exe():
    try:
        return find_fc_executable()
    except FileNotFoundError as exc:
        pytest.fail(f"{exc}\n{BUILD_HINT}", pytrace=False)


@pytest.fixture(scope="session")
def sil_base_cfg() -> FlightConfig:
    """Default config with the C6-7 backup charge used by every SIL scenario."""
    return sil_flight_config(FlightConfig.load(REPO_ROOT / "configs" / "default.json"))


def pytest_collection_modifyitems(items):
    """Copy @pytest.mark.req IDs into each test's user_properties, so they
    appear in JUnit XML (--junitxml) for the traceability matrix."""
    for item in items:
        for marker in item.iter_markers("req"):
            for rid in marker.args:
                item.user_properties.append(("req", rid))
