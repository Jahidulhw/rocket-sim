"""Fault injection for SIL runs.

Each fault has a start time and a duration (sim time, s; ignition at t = 0)
and acts on the sensor frames between the sensor models and the FC:

    kind      sensor       effect while active
    --------  -----------  -----------------------------------------------------
    dropout   (link)       frame not sent, with per-frame `probability` (1 = blackout)
    stuck     baro|accel   value frozen at the first reading inside the window
    spike     baro|accel   with per-frame `probability`, add +-`magnitude` (random sign)
    drift     baro|accel   add a bias growing at `magnitude` units/s from start
    hang      (FC)         the FC process stops responding at `start_s` (FC test flag
                           --inject-hang-at); duration is ignored, a hang is forever

Randomness (which frames drop/spike, spike signs) comes from its own seeded
stream, separate from sensor noise, and every fault draws the same number of
numbers every tick whether active or not. So adding a fault never shifts the
sensor noise, and a faulted run differs from its nominal twin ONLY by the fault.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

KINDS = ("dropout", "stuck", "spike", "drift", "hang")
SENSORS = ("baro", "accel")


@dataclass(frozen=True)
class Fault:
    kind: str
    start_s: float
    duration_s: float = math.inf
    sensor: str = "baro"
    magnitude: float = 0.0
    probability: float = 1.0

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f"unknown fault kind {self.kind!r}; expected one of {KINDS}")
        if self.kind in ("stuck", "spike", "drift") and self.sensor not in SENSORS:
            raise ValueError(f"fault sensor must be one of {SENSORS}")
        if self.duration_s <= 0 or not 0.0 <= self.probability <= 1.0:
            raise ValueError("fault duration must be > 0 and probability in [0, 1]")
        if self.kind in ("spike", "drift") and self.magnitude == 0.0:
            raise ValueError(f"{self.kind} fault needs a non-zero magnitude")

    def active(self, t: float) -> bool:
        return self.start_s <= t < self.start_s + self.duration_s

    @property
    def label(self) -> str:
        if self.kind in ("dropout", "hang"):
            return self.kind
        return f"{self.kind}_{self.sensor}"

    def to_dict(self) -> dict:
        d = asdict(self)
        d["duration_s"] = None if math.isinf(self.duration_s) else self.duration_s
        return d

    def to_spec(self) -> str:
        """Inverse of parse(): the CLI form, e.g. for replay commands."""
        dur = "inf" if math.isinf(self.duration_s) else f"{self.duration_s:.6g}"
        return f"{self.kind}:{self.start_s:.6g}:{dur}:{self.sensor}:{self.magnitude:.6g}:{self.probability:.6g}"

    @classmethod
    def from_dict(cls, d: dict) -> "Fault":
        d = dict(d)
        if d.get("duration_s") is None:
            d["duration_s"] = math.inf
        return cls(**d)

    @classmethod
    def parse(cls, spec: str) -> "Fault":
        """CLI form  kind:start[:duration[:sensor[:magnitude[:probability]]]],
        e.g. 'stuck:3:inf:baro', 'spike:2:6:accel:80:0.1', 'hang:4'."""
        parts = spec.split(":")
        if len(parts) < 2:
            raise ValueError(f"fault spec needs at least kind:start, got {spec!r}")
        kw = {"kind": parts[0], "start_s": float(parts[1])}
        if len(parts) > 2:
            kw["duration_s"] = float(parts[2])
        if len(parts) > 3:
            kw["sensor"] = parts[3]
        if len(parts) > 4:
            kw["magnitude"] = float(parts[4])
        if len(parts) > 5:
            kw["probability"] = float(parts[5])
        return cls(**kw)


class FaultInjector:
    """Applies a list of faults to each tick's sensor values."""

    def __init__(self, faults, rng: np.random.Generator):
        self.faults = list(faults)
        self.rng = rng
        self._held: dict[int, float] = {}

    @property
    def hang_at_s(self) -> float | None:
        hangs = [f.start_s for f in self.faults if f.kind == "hang"]
        return min(hangs) if hangs else None

    def apply(self, t: float, baro: float, accel: float) -> tuple[bool, float, float, list[str]]:
        """Returns (send, baro, accel, labels of faults active at t)."""
        send, active = True, []
        vals = {"baro": baro, "accel": accel}
        for i, f in enumerate(self.faults):
            u, sign_u = self.rng.random(2)   # always drawn: keeps the stream aligned
            if f.kind == "hang" or not f.active(t):
                if f.kind == "hang" and t >= f.start_s:
                    active.append(f.label)
                continue
            active.append(f.label)
            if f.kind == "dropout":
                if u < f.probability:
                    send = False
            elif f.kind == "stuck":
                vals[f.sensor] = self._held.setdefault(i, vals[f.sensor])
            elif f.kind == "spike":
                if u < f.probability:
                    vals[f.sensor] += f.magnitude if sign_u < 0.5 else -f.magnitude
            elif f.kind == "drift":
                vals[f.sensor] += f.magnitude * (t - f.start_s)
        return send, vals["baro"], vals["accel"], active
