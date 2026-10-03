"""Software-in-the-loop runner: the C++ flight computer flies the simulated rocket.

Each 100 Hz tick, in strict lockstep:

    truth (z, a_z) --> sensor models --> "S t baro accel" --> FC process
    physics <-- deploy command <-- parse/validate <-- "R t state alt vel deploy"

The simulator blocks until the FC answers (or the watchdog fires), so the
FC's view of time is exactly the frame stamps: no race between a fast sim and
a slow FC, and a run is reproducible from its seed.

Timeline of one run (sim time, ignition at t = 0):

    t < 0          pre-launch pad sit: rocket at rest, FC sees pad frames
    0 <= t < land  simulate() calls SilLink.tick() at every k * period
    t > land       post-landing frames at rest so the FC can detect landing

Failure handling:
  * Watchdog: no reply within `watchdog_timeout_s` of wall-clock time, or the
    process exited / closed its pipe -> the FC is declared FAILED, no further
    frames are sent, and it can no longer deploy. The motor's ejection charge
    is independent of the FC and still fires (the backup).
  * A malformed reply, an "E" reply, or a reply echoing the wrong time stamp
    is counted as a protocol error and treated as NO command for that tick:
    garbage must never be able to fire a pyro charge.

The watchdog timeout is the only wall-clock quantity in the loop. It decides
*whether* an FC is dead, never *what* a live FC computes, so it does not
affect determinism; it is set far above the FC's ~microsecond tick cost.
"""

from __future__ import annotations

import os
import queue
import subprocess
import threading
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .config import REPO_ROOT, FlightConfig
from .flight import FlightResult, simulate
from .protocol import ProtocolError, format_sensor_frame, parse_reply, time_matches
from .sensors import SensorConfig, SensorSuite

FC_EXE_ENV = "ROCKET_FC_EXE"
_EXE_NAME = "flight_computer.exe" if os.name == "nt" else "flight_computer"
_BUILD_DIR = REPO_ROOT / "fc" / "build"
BUILD_HINT = ("build it with:  cmake -S fc -B fc/build  &&  cmake --build fc/build --config Release"
              f"   (or point {FC_EXE_ENV} at the executable)")


def find_fc_executable(explicit: str | os.PathLike | None = None) -> Path:
    """Locate the flight computer: explicit path, then $ROCKET_FC_EXE, then fc/build.

    fc/build/Release/ is where multi-config generators (Visual Studio) put it;
    fc/build/ is where single-config generators (Makefiles, Ninja on Linux) do.
    """
    if explicit is None:
        explicit = os.environ.get(FC_EXE_ENV) or None
    if explicit is not None:
        p = Path(explicit)
        if not p.is_file():
            raise FileNotFoundError(f"flight computer not found at {p}")
        return p
    for sub in ("Release", "RelWithDebInfo", "Debug", ""):
        p = _BUILD_DIR / sub / _EXE_NAME
        if p.is_file():
            return p
    raise FileNotFoundError(f"flight computer executable not found under {_BUILD_DIR}; {BUILD_HINT}")


class FcFailure(RuntimeError):
    """The FC process stopped serving the protocol (hung, crashed or exited)."""

    def __init__(self, kind: str, detail: str = ""):
        super().__init__(f"{kind}: {detail}" if detail else kind)
        self.kind = kind  # "watchdog_timeout" | "exited" | "broken_pipe"


class FcProcess:
    """The flight computer as a child process speaking the line protocol.

    Pipes are binary and lines are encoded/decoded here, so behaviour is the
    same on Windows and Linux (no text-mode newline translation surprises).
    A reader thread moves stdout lines into a queue; the main thread waits on
    the queue with a timeout. That is the portable way to read a pipe with a
    timeout (select() does not work on pipes on Windows).
    """

    def __init__(self, exe: str | os.PathLike, args=(), timeout_s: float = 2.0):
        self.exe = Path(exe)
        self.args = [str(a) for a in args]
        self.timeout_s = timeout_s
        self.stderr_lines: list[str] = []
        self.returncode: int | None = None
        self._lines: queue.Queue = queue.Queue()
        self._proc = subprocess.Popen(
            [str(self.exe), *self.args], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, bufsize=0)
        threading.Thread(target=self._pump_stdout, daemon=True).start()
        threading.Thread(target=self._pump_stderr, daemon=True).start()

    def _pump_stdout(self):
        for raw in iter(self._proc.stdout.readline, b""):
            self._lines.put(raw.decode("ascii", errors="replace").rstrip("\r\n"))
        self._lines.put(None)  # EOF sentinel

    def _pump_stderr(self):
        # Drained continuously: an unread stderr pipe that fills up would
        # block the child and look exactly like a hang.
        for raw in iter(self._proc.stderr.readline, b""):
            self.stderr_lines.append(raw.decode("utf-8", errors="replace").rstrip("\r\n"))

    def exchange(self, line: str) -> str:
        """Send one line, return the FC's one-line answer. Raises FcFailure."""
        try:
            self._proc.stdin.write((line + "\n").encode("ascii"))
            self._proc.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise FcFailure("broken_pipe", str(exc)) from None
        try:
            reply = self._lines.get(timeout=self.timeout_s)
        except queue.Empty:
            raise FcFailure("watchdog_timeout", f"no reply within {self.timeout_s:g} s") from None
        if reply is None:
            raise FcFailure("exited", f"FC closed stdout (exit code {self._proc.poll()})")
        return reply

    def close(self, grace_s: float = 2.0) -> int | None:
        """Send END and wait; kill the process if it does not exit (a hung FC)."""
        if self._proc.poll() is None:
            try:
                self._proc.stdin.write(b"END\n")
                self._proc.stdin.flush()
                self._proc.stdin.close()
            except (BrokenPipeError, OSError):
                pass
            try:
                self._proc.wait(timeout=grace_s)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()
        self.returncode = self._proc.returncode
        return self.returncode

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


@dataclass
class SilConfig:
    sensors: SensorConfig = field(default_factory=SensorConfig)
    seed: int = 0
    fc_exe: str | None = None          # None: find_fc_executable()
    fc_args: tuple = ()
    watchdog_timeout_s: float = 2.0
    pre_launch_s: float = 10.0         # pad sit before ignition (FC calibration, false-launch exposure)
    post_landing_s: float = 10.0       # at-rest frames after touchdown (FC landing detection)


LOG_FIELDS = ("t", "z_true", "vz_true", "az_true", "baro", "accel", "sent", "reply_ok",
              "fc_state", "est_alt", "est_vel", "fc_deploy", "error")


@dataclass
class SilResult:
    flight: FlightResult
    log: dict                          # LOG_FIELDS -> np.ndarray, one entry per tick
    fc_failed: bool
    failure_t: float | None
    failure_reason: str | None
    protocol_errors: int
    fc_deploy_t: float | None          # first tick the FC commanded deploy (even if the chute was already out)
    fc_returncode: int | None
    fc_stderr: list

    @property
    def deploy_mechanism(self) -> str | None:
        """'fc' or 'motor' (which deployed the chute first), None if it never deployed."""
        return self.flight.deployment.get("mechanism")


class SilLink:
    """simulate() controller: turns truth into sensor frames and FC replies into commands."""

    def __init__(self, fc: FcProcess, sensors: SensorSuite):
        self.fc = fc
        self.sensors = sensors
        self.period_s = sensors.cfg.period_s
        self.in_flight_ticks = 0
        self.failed = False
        self.failure_t: float | None = None
        self.failure_reason: str | None = None
        self.protocol_errors = 0
        self.fc_deploy_t: float | None = None
        self.log = {k: [] for k in LOG_FIELDS}

    # simulate() interface ------------------------------------------------
    def tick(self, t: float, y: np.ndarray, accel: np.ndarray) -> bool:
        self.in_flight_ticks += 1
        return self.exchange(t, float(y[2]), float(y[5]), float(accel[2]))

    # ---------------------------------------------------------------------
    def exchange(self, t: float, z: float, vz: float, az: float) -> bool:
        """One tick. Returns the FC's deploy command (False on any failure)."""
        baro, acc = self.sensors.sample(z, az)   # always drawn: keeps the noise stream aligned
        row = {"t": t, "z_true": z, "vz_true": vz, "az_true": az, "baro": baro, "accel": acc,
               "sent": False, "reply_ok": False, "fc_state": "", "est_alt": np.nan,
               "est_vel": np.nan, "fc_deploy": False, "error": ""}
        command = False
        if self.failed:
            row["error"] = "fc_failed"
        else:
            row["sent"] = True
            try:
                raw = self.fc.exchange(format_sensor_frame(t, baro, acc))
                reply = parse_reply(raw)
                if not time_matches(t, reply.t):
                    raise ProtocolError(f"time stamp mismatch: sent {t:.6f}, got {reply.t:.6f}")
            except FcFailure as exc:
                self.failed, self.failure_t, self.failure_reason = True, t, str(exc)
                row["error"] = f"fc_failure: {exc}"
            except ProtocolError as exc:
                self.protocol_errors += 1
                row["error"] = f"protocol: {exc}"
            else:
                row.update(reply_ok=True, fc_state=reply.state, est_alt=reply.est_alt_m,
                           est_vel=reply.est_vel_mps, fc_deploy=reply.deploy)
                command = reply.deploy
                if command and self.fc_deploy_t is None:
                    self.fc_deploy_t = t
        for k, v in row.items():
            self.log[k].append(v)
        return command

    def log_arrays(self) -> dict:
        return {k: np.array(v) for k, v in self.log.items()}


def run_sil(cfg: FlightConfig, sil: SilConfig | None = None) -> SilResult:
    """Fly one closed-loop SIL flight (pad sit, flight, post-landing)."""
    sil = sil or SilConfig()
    sensors = SensorSuite(sil.sensors, np.random.default_rng(sil.seed))
    period = sensors.cfg.period_s
    with FcProcess(find_fc_executable(sil.fc_exe), sil.fc_args, sil.watchdog_timeout_s) as fc:
        link = SilLink(fc, sensors)
        # Ticks are k * period on one integer grid across all three segments,
        # so stamps never accumulate floating-point drift.
        n_pre = round(sil.pre_launch_s / period)
        for k in range(-n_pre, 0):
            link.exchange(k * period, 0.0, 0.0, 0.0)
        flight = simulate(cfg, controller=link)
        if flight.landed:
            k0 = link.in_flight_ticks
            for k in range(k0, k0 + round(sil.post_landing_s / period)):
                link.exchange(k * period, 0.0, 0.0, 0.0)
        fc.close()
    return SilResult(flight=flight, log=link.log_arrays(), fc_failed=link.failed,
                     failure_t=link.failure_t, failure_reason=link.failure_reason,
                     protocol_errors=link.protocol_errors, fc_deploy_t=link.fc_deploy_t,
                     fc_returncode=fc.returncode, fc_stderr=list(fc.stderr_lines))


def sil_flight_config(base: FlightConfig) -> FlightConfig:
    """SIL scenario: the default rocket on a C6-7 instead of a C6-5.

    With electronic deployment the motor's ejection charge is the BACKUP, so
    its delay must be longer than the expected time from burnout to apogee:
    a C6-5 fires ~0.6 s *before* apogee on this rocket (it would always beat
    the FC), while a C6-7 fires ~1.4 s after it. This is the standard
    hobby practice for dual-redundant deployment.
    """
    cfg = base.copy()
    cfg.motor.ejection_delay_s = 7.0
    cfg.recovery.deploy_delay_s = None   # use the motor's delay
    return cfg

