# Flight computer walkthrough: design decisions and reasoning

This document explains **why** the flight computer (FC) in [`fc/`](../fc/) is
built the way it is: every threshold, every state transition, the alternatives
I rejected, and what goes wrong when a value is mistuned. It assumes you know
C++ and skips syntax.

It is updated as each milestone lands:
* State machine and baseline detector: milestone 2 (this version).
* Kalman filter: milestone 3.
* Faults and requirements: milestone 4.

**Scope.** The FC only detects flight events and commands parachute deployment.
It has no guidance, steering, attitude control or targeting. Its single output
is one bit: deploy / don't deploy.

---

## 1. Architecture in one paragraph

The FC is a separate process. The simulator sends it one sensor frame per
100 Hz tick and blocks until it gets a reply (strict lockstep, see
[protocol.md](protocol.md)). The process has three layers:

1. `main.cpp`: the I/O loop. It reads a line, writes a line, flushes.
2. `Runner`: the protocol session. It rejects malformed lines and enforces
   strictly increasing time.
3. `StateMachine`: the flight logic. It is pure computation: one frame in, one
   decision out.

Everything below `main.cpp` is a static library with no I/O, so the GoogleTest
suite runs exactly the code the executable runs, without a process or a pipe.

**The FC's only clock is the frame timestamp.** It never reads the wall clock.
That gives two things:
* **Determinism.** The same input gives byte-identical output (REQ-006).
* **Real dt.** dt comes from the timestamps, so timers stay correct when
  frames are dropped or arrive irregularly.

The real-hardware equivalent is timestamping each sample from a hardware
timer when it is captured.

---

## 2. Inputs, and what they look like in flight

| Sensor | What it measures | Noise σ | Bias | Other |
|---|---|---|---|---|
| Barometer | Altitude (m) | 0.5 m | +0.5 m | 0.1 m quantization |
| Accelerometer | Vertical **specific force** (m/s²), +up | 0.5 m/s² | +0.2 m/s² | Saturates at ±24 g |

Specific force is what an accelerometer physically reports: acceleration minus
gravity. The flight-phase signatures, from the default C6 flight:

| Phase | Specific force | Why |
|---|---|---|
| Pad | +9.8 | The pad pushes up against gravity |
| Boost | +42 … +225 | Thrust (peak about 22 g) |
| Coast, climbing | −43 → 0 | Drag points *down*, adding to gravity, so the accelerometer reads negative |
| Apogee | ≈ 0 | Almost free fall |
| Chute opening | Spike to about +30 | Snatch load |
| Under chute / landed | +9.8 | Steady terminal velocity, or at rest |

The **sign change at burnout** (from +40 to −40) is the cleanest event in the
whole flight. That's why burnout is detected from the accelerometer.

---

## 3. Ground reference

Every altitude the FC uses is **above ground level (AGL)**: the barometer
reading minus a ground reference learned on the pad.

* **Why.** A barometer's absolute reading carries the field elevation, the
  day's weather and a reference-pressure bias. AGL removes all of these. The
  FC therefore works unchanged at any launch site, and the 0.5 m sensor bias
  cancels exactly.
* **How.**
  1. For the first 1 s of frames, the reference is a plain mean.
  2. After that it is an exponential moving average with τ = 2 s and a
     correct dt: α = 1 − e^(−dt/τ).
  3. The EMA only accepts samples within **±3 m** of the current reference.
  4. The reference is frozen at launch.
* **Why the ±3 m gate.** My first design had no gate, and a unit test caught
  the flaw. If the reference keeps averaging while the rocket climbs its first
  15 m, it creeps up by about 2 m. That delays the barometric backup launch
  detection. The gate stops the creep, and it also stops a barometer spike on
  the pad from corrupting the reference.
* **What τ trades off.**
  * Too short (0.1 s): the reference is as noisy as a raw sample, and it
    follows any slow pressure disturbance.
  * Too long (60 s): it can't follow weather drift during a long pad wait.
  * With τ = 2 s the residual noise is about σ·√(dt/2τ) ≈ 0.03 m. The
    tracking lag is only rate × τ. Weather changes millimetres per second.
* **Rejected: no arming delay.** Real altimeters often refuse to arm until a
  few seconds of stable pad data are in. Here the reference starts from the
  first frame. That's acceptable because the SIL always gives a 10 s pad sit.
  An arming delay plus a pad self-test (does the accelerometer read 1 g ± 0.5?
  is the barometer steady?) would be the next step on real hardware.

---

## 4. Persistence filters ("N consecutive samples")

Most transitions need a condition to hold for N consecutive samples. That is a
**minimum-duration filter**: a single bad sample, such as an outlier, a bump or
an electrical glitch, can never cause a transition alone.

**Frame gaps break the run.** If more than 50 ms (5 periods) passes between
frames, the counter restarts. "Five consecutive samples" is meant to show the
condition held for about 50 ms. Two samples on either side of a 1 s hole prove
nothing about the hole.
* *Downside:* a heavy frame-drop fault delays detections. The backup timer
  covers that case.
* *Rejected alternative:* pure time-based persistence ("held for ≥ 50 ms of
  timestamps"). With a gap, that would accept the hole as evidence.

The **event time** recorded is the start of the confirming run, not the sample
that completed it. That's more accurate: launch is placed when the
acceleration began, not 40 ms later. This matters because the backup timer
counts from launch.

---

## 5. State transitions

```
PAD ──launch──▶ BOOST ──burnout──▶ COAST ──apogee──▶ APOGEE ──next frame──▶ DESCENT ──stable──▶ LANDED
                                     └────── backup timer (deploy) ──────────────▶┘
```

States only move forward (unit test `StatesNeverGoBackwards`). All values are
in [`fc/include/fc/config.hpp`](../fc/include/fc/config.hpp).

### 5.1 PAD → BOOST: launch detection

| Path | Condition | Persistence |
|---|---|---|
| **Primary** | specific force > **25 m/s²** | 5 consecutive samples (50 ms) |
| **Backup** | AGL > **15 m** | 10 consecutive samples |

**Why 25 m/s².** On the pad the reading is 9.81 + 0.2 bias with σ 0.5.
* 25 is 30 σ above that, so Gaussian noise can't reach it, even over 1000
  sixty-second pad sits (REQ-002).
* During boost the reading exceeds 42 from about 40 ms after ignition to
  burnout, so detection is fast. In SIL, BOOST is reported **+0.06 s** after
  true liftoff.
* *Too low* (say 12 m/s²): someone bumping the rail, or the airframe swaying
  in a gust, launches the FC on the pad.
* *Too high* (say 60 m/s²): the C6's sustain phase sits at 42–60 m/s², so
  launch would only register during the initial spike, or fall back to the
  slow barometric path. A softer motor might never trigger it at all.

**Why 5 samples.**
* *Too low* (1): a single outlier launches the FC.
* *Too high* (50, i.e. 0.5 s): the launch time comes late, and a short-burn
  motor (an A10 burns 0.3 s) might not hold the condition long enough.

**Why a barometric backup.** Without it, the accelerometer is a **single point
of failure**: a dead or stuck accelerometer would keep the FC on the pad for
the whole flight, and it would never deploy.
* 15 m is 30 σ of barometer noise. The rocket reaches it about 0.9 s after
  liftoff.
* *Too low* (2 m): wind gusting over the static port, or someone walking past
  the vent, triggers it.
* *Too high* (100 m): the backup takes about 1.6 s and degrades the backup
  timer's reference.

**What a false launch costs.** It is designed to be safe. See 5.2: a rocket
still sitting on the pad can never leave BOOST, and BOOST can never deploy
(unit test `FalseLaunchOnPadNeverReachesDeployableState_REQ003`).

### 5.2 BOOST → COAST: burnout

| Path | Condition |
|---|---|
| **Primary** | specific force < **5 m/s²** for 5 samples |
| **Fallback** | > **3 s** since launch **and** AGL > **30 m** |

**Why 5 m/s².** At burnout the reading falls from about +40 to about −43,
because drag now adds to gravity. It stays negative for the whole climb.
* The threshold sits in a gap about 80 m/s² wide. In SIL, COAST is reported
  **+0.02 s** after true burnout.
* *Too high* (45): it fires during the C6's sustain phase (about 42), ending
  the lockout while the motor is still burning.
* *Too low* (−45): it never fires, because the most negative reading is −43,
  and the FC waits for the fallback.

**Why a fallback, and why it has two conditions.**
* A stuck accelerometer must not trap the FC in BOOST, where deployment is
  locked out.
* *Time alone* would be dangerous. After a false launch on the pad, 3 s would
  pass and the FC would reach COAST, a deployable state, while still sitting on
  the pad.
* *Altitude alone* would also be dangerous: during the burn, the rocket passes
  30 m long before burnout.
* Requiring both means a real flight with a stuck accelerometer reaches COAST
  at launch + 3 s. A rocket sitting on the pad never does.
* *max_boost too short* (shorter than the 1.86 s burn): even a healthy flight
  reaches COAST while the motor burns, which breaks the lockout.
* *max_boost too long* (8 s): with a stuck accelerometer, COAST arrives near
  apogee, and the backup timer has no margin left.

### 5.3 Boost lockout (deployment impossible in PAD or BOOST)

**Why.** Deploying under thrust or at maximum speed is the most destructive
recovery failure:
* The chute opens at about 100 m/s, the shock cord zippers the airframe, and
  the canopy shreds.
* Or the motor drives the rocket *into* its own recovery gear.

So a deploy must be *structurally impossible* in those states, not merely
unlikely. It is enforced twice:

1. **By construction.** Only the COAST handler can command deployment (apogee
   or timer). PAD and BOOST have no code path that sets the deploy flag.
2. **At the output.** The deploy bit is ANDed with "state is neither PAD nor
   BOOST" before it leaves the FC. A future refactor that accidentally adds a
   deploy path still can't fire in boost.

This is defence in depth for the one requirement (REQ-003) whose violation
destroys the vehicle. Unit tests deliberately present the apogee signature (a
falling barometer) in PAD and in BOOST, and check that the FC ignores it.

### 5.4 COAST → APOGEE: baseline detector

**Condition:** AGL altitude is **≥ 1.5 m below the highest AGL seen since
launch** for **10** consecutive samples (100 ms). Then the FC commands
deployment.

**Rejected: "barometer strictly decreasing for M samples".** This is the
textbook version, and noise defeats it.
* For independent noise, the chance that M consecutive samples happen to fall
  in strictly decreasing order is 1/M!. For M = 5 that's 1/120.
* Near apogee the true altitude changes far less than the 0.5 m noise between
  samples. So random "decreasing" streaks turn up within seconds, before the
  real apogee, and fire the chute while the rocket is still climbing.
* Quantization makes it worse: equal readings break real streaks.

**Why "below the running maximum".** It compares against a robust reference,
the peak, and asks for a clear margin.
* The running maximum itself is biased upward: it is the luckiest noisy sample
  near the top, about +2σ ≈ +1 m.
* So the rocket must really fall by about 1 m + 1.5 m + noise before 10
  samples in a row agree. Under gravity that takes about 0.5 s.

**Measured in SIL.** Over 50 seeds with nominal noise, the baseline deploys
**+0.509 ± 0.085 s after true apogee** (range +0.25 to +0.64 s). It is never
early, and only 34 % of deployments land within 0.5 s.

**Too low or too high:**

| Mistuning | Effect |
|---|---|
| Margin too low (0.3 m) | Noise satisfies it during the slow final climb, so the chute fires early with the rocket still moving |
| Margin too high (10 m) | About 1.5 s late, and the chute opens at about 15 m/s descending, with a hard snatch |
| Samples too low (2) | Same failure as a low margin: one unlucky pair of samples fires the chute |
| Samples too high (100) | Adds 1 s of lag |

**This lag is structural.** A detector working on raw altitude must wait until
the altitude has *visibly* dropped. A velocity estimate that crosses zero sees
apogee *as it happens*. That is the case for the Kalman filter (milestone 3).

### 5.5 Backup timer

**Condition:** still in COAST **8.5 s after launch** → deploy, and go straight
to DESCENT.

* **Why 8.5 s.** In simulation, nominal apogee comes about 7.4 s after launch.
  Monte Carlo dispersion moves it by about ±0.2 s (1σ).
  * 8.5 s is about 5σ late, so the timer never pre-empts a working detector.
    It is also early enough that the rocket is only about 10 m/s into its fall.
  * Hobby altimeters set this the same way, from a pre-flight simulation.
* *Too short* (below the apogee time): the timer fires before apogee **on every
  flight**, at speed. This is the worst failure, because it is silent and
  systematic.
* *Too long* (15 s): in a failure case the chute opens at about 60 m/s
  descending, and lower down.
* **Why go straight to DESCENT, skipping APOGEE.** The FC never confirmed
  apogee, so it doesn't claim one. It also makes the deploy reason visible in
  the reply stream (see [protocol.md](protocol.md)) without changing the
  protocol.
* **Why count from launch, per the spec.** I considered counting from
  burnout. That would survive a false launch on the pad followed by a real
  launch much later. But then the timer depends on burnout detection, the
  less robust event. The accelerometer-fails-high-on-the-pad case is better
  handled by a pre-launch self-test (see §3).
* **The motor's ejection charge is a separate, independent backup.** It is a
  C6-**7**, firing at burnout + 7 s ≈ 1.4 s after apogee. The FC's own timer
  covers FC-internal detection failures. The motor charge covers the FC
  failing entirely (REQ-005).

### 5.6 APOGEE → DESCENT

This happens on the next frame. APOGEE is a one-sample **event state**, so the
reply stream records the exact frame where the decision was made.

### 5.7 DESCENT → LANDED

**Condition:** AGL stays within **±3 m** of a reference sample for **5 s**.
Leaving the band, or a frame gap, restarts the window with the current sample
as the new reference.

**Why these values.**
* Under the chute the rocket sinks at about 3.8 m/s, so it leaves a 3 m band in
  under 1 s. It can't fake 5 s of stability, even at 1.5 m/s (unit test).
* Between two samples, barometer noise has σ√2 ≈ 0.71 m. 3 m is 4.2 σ, so on
  the ground a restart caused by noise happens about once every 40,000
  samples.

**Mistuning.**
* *Band too tight or duration too long:* LANDED never comes.
* *Band too loose or duration too short:* LANDED is declared while still
  descending slowly.
* This transition isn't safety-critical, because the chute is already out. On
  hardware it would trigger the locator beacon, flush the flight log and enter
  low power.

**Measured.** LANDED comes 4–8 s after touchdown. The window may start just
above the ground and restart once. That's why the SIL sends 15 s of
post-landing frames.

### 5.8 The deploy command is latched

Once commanded, `deploy = 1` in every later reply.
* A lost or garbled reply (which the simulator treats as "no command") can't
  cancel a deployment.
* A deploy can't be "undone", which is physically true anyway.

Real pyro channels fire for a fixed on-time. Latching is the protocol-level
equivalent of "command until confirmed".

---

## 6. Other design decisions

| Decision | Why | Rejected alternative |
|---|---|---|
| FC is a separate process, not a library linked into Python | It's a real integration boundary. Crashes, hangs and malformed output are possible and get tested. The FC binary could later run unchanged on a different machine or in HIL | pybind11 extension: faster, but a crash takes down the simulator, and nothing about the interface gets tested |
| Strict parsing (exact fields, full-token numbers, `std::from_chars`) | A misparsed altitude is silent, while a rejected frame is visible. `from_chars` is locale-independent and doesn't throw | `strtod`/`stringstream`: locale-dependent, and accepts trailing garbage |
| Every input line gets exactly one reply, including `E` for garbage | Lockstep can never slip by one line | Silently dropping bad lines: the simulator would wait for a reply that never comes, and the watchdog would trip on a healthy FC |
| A malformed reply means "no command" on the simulator side | Garbage must never fire a pyro charge | Reusing the last valid command: hides FC faults |
| Accelerometer reads specific force (+g at rest) | That's what real accelerometers report. The FC must handle the gravity offset, as on real hardware | Feeding kinematic acceleration: easier, but unrealistic |
| Baseline mode reports `est_vel = 0` | It has no velocity estimate. Making one up (a noisy finite difference) would be dishonest | Finite difference: velocity noise about 70 m/s at 100 Hz |
| Thresholds live in one `FcConfig` struct | One place to review, justify and test, and tests can override values | Constants spread through the logic |
| Test noise uses mt19937 + Box–Muller | `std::normal_distribution` differs between MSVC and libstdc++, so seeded tests would see different numbers on Windows and Linux CI | `std::normal_distribution` |

---

## 7. Kalman filter

*Added in milestone 3.*

## 8. Faults: detection and tolerance

*Added in milestone 4.*
