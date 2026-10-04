"""Simulator side of the SIL line protocol (see docs/protocol.md).

    Sim -> FC:  "S <t> <baro_alt_m> <accel_mps2>"    then "END" at shutdown
    FC -> Sim:  "R <t> <state> <est_alt> <est_vel> <deploy 0-3>"
                deploy bitmask: 1 = drogue/primary chute, 2 = main chute
                "E <reason>"   (the FC rejected our line)

Parsing mirrors the C++ side: strict, and any deviation is an error, never a
guess. A reply that cannot be parsed is treated as *no command* (in
particular, never as a deploy command).
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

FC_STATES = ("PAD", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED")
MAX_LINE_LENGTH = 256

# A decimal number as both sides write it: optional '-', digits, optional
# fraction, optional exponent. No '+', no hex, no inf/nan.
_NUM = r"-?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
_REPLY_RE = re.compile(rf"R ({_NUM}) ([A-Z]+) ({_NUM}) ({_NUM}) ([0-3])")


class ProtocolError(ValueError):
    """A line from the FC that violates the protocol."""


@dataclass(frozen=True)
class FcReply:
    t: float
    state: str
    est_alt_m: float
    est_vel_mps: float
    deploy: bool                 # bit 0: primary (drogue) chute
    deploy_main: bool = False    # bit 1: main chute (dual deploy)

    @property
    def command(self) -> int:
        return int(self.deploy) | (2 if self.deploy_main else 0)


def format_sensor_frame(t: float, baro_alt_m: float, accel_mps2: float) -> str:
    """Sensor frame WITHOUT newline. t to 1 us, baro to 1 mm, accel to 0.1 mm/s^2."""
    for v in (t, baro_alt_m, accel_mps2):
        if not math.isfinite(v):
            raise ValueError(f"cannot send non-finite value {v!r}")
    return f"S {t:.6f} {baro_alt_m:.3f} {accel_mps2:.4f}"


def parse_reply(line: str) -> FcReply:
    """Parse one FC line. Raises ProtocolError for anything but a valid R line
    (including an FC "E <reason>" line, whose reason is in the message)."""
    if line.endswith("\n"):
        line = line[:-1]
    if line.endswith("\r"):
        line = line[:-1]
    if len(line) > MAX_LINE_LENGTH:
        raise ProtocolError("line_too_long")
    if line.startswith("E "):
        raise ProtocolError(f"fc_rejected_input: {line[2:]}")
    m = _REPLY_RE.fullmatch(line)
    if not m:
        raise ProtocolError(f"malformed reply: {line!r}")
    t, state, alt, vel, dep = m.groups()
    if state not in FC_STATES:
        raise ProtocolError(f"unknown state {state!r}")
    vals = [float(t), float(alt), float(vel)]
    if not all(math.isfinite(v) for v in vals):
        raise ProtocolError(f"non-finite number in {line!r}")
    mask = int(dep)
    return FcReply(vals[0], state, vals[1], vals[2], bool(mask & 1), bool(mask & 2))


def time_matches(sent_t: float, echoed_t: float) -> bool:
    """The FC echoes our stamp to 1 us; anything else is a stale/foreign reply."""
    return abs(sent_t - echoed_t) <= 1e-6
