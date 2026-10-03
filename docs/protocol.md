# SIL protocol: simulator ⇄ flight computer

The flight computer (FC) is a separate process (`fc/build/.../flight_computer`).
The simulator talks to it over the FC's **stdin/stdout** using ASCII text, one
line per message, with lines ending in `\n`. The FC ignores a single trailing
`\r`, because Windows pipes may deliver CRLF. The FC writes diagnostics only to
**stderr**, so stdout carries nothing but protocol.

Implementations: [`fc/src/protocol.cpp`](../fc/src/protocol.cpp) (FC side) and
[`sim/protocol.py`](../sim/protocol.py) (sim side), driven by
[`sim/sil.py`](../sim/sil.py).

## Messages

### Sim → FC

| Line | Meaning |
|---|---|
| `S <t> <baro_alt_m> <accel_mps2>` | One sensor frame per 100 Hz tick |
| `END` | Orderly shutdown: the FC exits with code 0 and sends no reply |

* `t`: simulator time in seconds, written to 1 µs (`%.6f`). Ignition is
  t = 0, so pre-launch pad frames have **negative** stamps. The stamps are the
  FC's **only clock**: it computes every dt from them and never reads the wall
  clock.
* `baro_alt_m`: barometric altitude in metres (`%.3f`), including sensor noise,
  bias and quantization.
* `accel_mps2`: vertical **specific force** in m/s², positive up (`%.4f`). It
  reads **+9.81 at rest on the pad**, 0 in free fall, and is negative while
  coasting upward, because drag pushes down. The FC gets kinematic vertical
  acceleration as `a = accel − g`. See [`sim/sensors.py`](../sim/sensors.py).

### FC → Sim

| Line | Meaning |
|---|---|
| `R <t> <state> <est_alt> <est_vel> <deploy>` | Reply to an accepted `S` frame |
| `E <reason>` | The FC rejected the input line; `<reason>` is one token |

* `t`: the frame's stamp echoed back (`%.6f`).
* `state`: one of `PAD BOOST COAST APOGEE DESCENT LANDED`.
* `est_alt`, `est_vel`: the FC's altitude (m) and vertical velocity (m/s)
  estimates, written `%.3f`.
* `deploy`: `1` = deployment commanded, `0` = not commanded. The flag is
  **latched**: once the FC sends `1`, every later reply also carries `1`. If one
  reply is lost or garbled, the next one still carries the command, and the
  command can never be withdrawn. In PAD and BOOST it is always `0` (the boost
  lockout).
* **Why the FC deployed** can be read from `state` in the first reply with
  `deploy = 1`:
  * `APOGEE` means the apogee detector fired.
  * `DESCENT` means the backup timer fired. The FC goes straight from COAST to
    DESCENT, because it never confirmed apogee.

  This keeps the reply format fixed while making the deploy mechanism
  traceable.

FC command line:
`flight_computer [--mode kalman|baseline|stub] [--param key=value] [--inject-hang-at <t>]`.

* `kalman` is the default. `baseline` is the raw-barometer detector, kept for
  comparison.
* `stub` is a plumbing test double: it echoes the barometer, always reports
  `PAD` and never deploys.
* `--param` overrides a numeric tuning value, for tuning and mistuning studies
  (see `fc/src/config.cpp`). Unknown keys are rejected.

The FC's `E` reasons are `line_too_long`, `empty_line`, `empty_field`,
`bad_field_count`, `unknown_tag`, `bad_number` and `non_increasing_time`.

## Lockstep

Each tick the simulator sends **exactly one line** and blocks until it gets
**exactly one line** back. The FC answers every line it receives, including
garbage, which gets an `E` line. So the two sides can never drift out of step
by one message.

Why lockstep, rather than letting the FC run freely on its own thread or clock:

* **Deterministic.** The FC's input depends only on the seed and the physics,
  never on OS scheduling. The same seed gives byte-identical logs.
* **No time base mismatch.** The sim may run much faster or slower than real
  time. The FC sees "real" time anyway, because its time *is* the stamps.
* **Causality is exact.** A deploy command answering frame *t* is applied at
  *t*. The simulator then integrates onward from that instant with the chute
  out.

What this costs: lockstep doesn't model FC compute latency or a missed
real-time deadline. It is a functional SIL test, not a timing test. On real
hardware, deadline overruns would be caught in hardware-in-the-loop (HIL)
testing. A hung FC is still detected; see the watchdog below.

## Validation rules (both sides)

Parsing is strict. Anything that deviates is rejected, never "best-effort"
interpreted. A misread altitude is worse than a missing one: a missing frame
is visible, a wrong value is silent.

* Single spaces separate fields. Leading, trailing or double spaces and tabs
  are rejected.
* The field count must be exact, and tags (`S`, `R`, `E`, `END`) are case
  sensitive.
* Numbers must be plain decimals with an optional leading `-`, an optional
  fraction and an optional exponent. Every character of the token must be
  consumed. No `+`, hex, `inf` or `nan`, and nothing that overflows to infinity.
* Lines longer than 256 characters are rejected.
* **FC only:** the frame time must strictly increase, or the frame gets
  `E non_increasing_time`. A repeated or backwards stamp would make dt ≤ 0 and
  corrupt the filter. A rejected frame does not advance the FC's clock.
* **Sim only:** the echoed `t` must match the sent stamp within 1 µs, the
  `%.6f` rounding. A mismatch means a stale or foreign reply.

The FC parses numbers with `std::from_chars`. It is locale-independent, so a
German locale can't turn `1.5` into `1`. It never throws or allocates, and it
reports how many characters it consumed.

### What the simulator does with a bad reply

Each of these counts as one **protocol error** and means **no command** for
that tick:

* a malformed `R` line,
* an `E` line,
* a time-stamp mismatch.

In particular, a garbage reply can never fire the parachute, however many `1`
characters it contains. Protocol errors are counted and logged per tick.

## Watchdog

`sim/sil.py` waits for each reply for at most `watchdog_timeout_s`
(default 2 s of wall-clock time). The FC is declared **failed** if:

* the timeout expires (a hung FC),
* the FC closes stdout or exits (a crash), or
* writing to its stdin fails (broken pipe).

After a failure the simulator logs the time and reason and stops sending
frames. The FC can no longer deploy anything. The motor's **ejection charge**
is independent of the FC and still fires at burnout + delay; it is the backup
deployment. At the end of the run the simulator sends `END`. If the process
doesn't exit within a grace period, it is killed.

The timeout is the only wall-clock quantity in the loop. It decides *whether*
an FC is dead, never *what* a live FC computes, so it doesn't affect
determinism. A healthy FC answers in microseconds, so 2 s gives a huge margin
against false trips on a loaded CI machine.

Test hook: `flight_computer --inject-hang-at <t>` makes the real FC process stop
replying at the first frame with stamp ≥ t. It blocks forever and never exits
on its own. The watchdog is therefore tested against a genuinely hung process,
not a mock.

## Example session

```
sim → S -0.010000 0.400 10.1032
fc  → R -0.010000 PAD 0.400 0.000 0
sim → S 0.000000 0.600 9.6611
fc  → R 0.000000 PAD 0.600 0.000 0
sim → S 0.010000 0.6 x
fc  → E bad_number
sim → END
      (FC exits 0)
```
