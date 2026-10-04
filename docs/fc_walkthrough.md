# Flight computer walkthrough: design decisions and reasoning

This document explains **why** the flight computer (FC) in [`fc/`](../fc/) is
built the way it is: every threshold, every state transition, the alternatives
I rejected, and what goes wrong when a value is mistuned. It assumes you know
C++ and skips syntax.

It is updated as each milestone lands:
* State machine and baseline detector: milestone 2.
* Kalman filter: milestone 3.
* Faults and requirements: milestone 4 (this version).

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

**Condition:** a **low-pass-filtered** AGL (an EMA with τ = 0.5 s, residual
noise about 0.05 m) stays within **±3 m** of a reference for **5 s**. Leaving
the band, or a frame gap, restarts the window with the current *filtered* value
as the new reference.

**Why these values.**
* Under the chute the rocket sinks at about 3.8 m/s, so it leaves a 3 m band in
  under 1 s. It can't fake 5 s of stability, even at 1.5 m/s (unit test). The
  0.5 s filter lag doesn't change that.
* On the ground, the filtered altitude moves by centimetres, so the window
  completes 5 s after it starts.

**Bug found by the Monte Carlo (and why the filter is there).** My first
version ran the band test on *raw* samples.
1. My analysis said a noise-induced restart happens about once per 40,000
   samples, because 3 m is 4.2σ of the difference between two random samples.
2. That analysis was wrong. A restart adopts **the sample that broke the band**
   as its new reference, and that sample is, by construction, a noise extreme.
3. On the ground, the reference then ping-ponged between about ±1.6 m
   extremes, and each restart only needed the *opposite* extreme.
4. Six of the 207 nominal Monte Carlo runs reported LANDED later than 10 s
   after touchdown, or never (REQ-010).

Filtering removes the selection bias: a restart can no longer pick a tail
value. The C++ regression test `LandingNotDefeatedByNoiseExtremes_REQ010`
fails on the old code (4 of 60 seeds late) and passes on the new code.

*Lesson:* the failure came from an unexamined *selection* effect, not from the
noise level itself. It took a large campaign, not a handful of runs, to show
it.

**Mistuning.**
* *Band too tight or duration too long:* LANDED never comes.
* *Band too loose or duration too short:* LANDED is declared while still
  descending slowly.
* *Filter τ too long* (say 5 s): the filter lags a slow descent, and LANDED can
  come early.
* This transition isn't safety-critical, because the chute is already out. On
  hardware it would trigger the locator beacon, flush the flight log and enter
  low power.

**Measured.** All 207 nominal Monte Carlo runs report LANDED within 10 s of
touchdown, typically about 5 s after.

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

Code: [`fc/include/fc/kalman.hpp`](../fc/include/fc/kalman.hpp) and
[`fc/src/kalman.cpp`](../fc/src/kalman.cpp). The filter is the **default
apogee detector**. The baseline (§5.4) stays selectable with `--mode
baseline`, and the two modes differ *only* in the apogee decision. Launch,
burnout, lockout, the timer and landing are identical, so the comparison is
fair.

### 7.1 Why a filter at all

The baseline must wait until the altitude has *visibly fallen*: about 2.5 m,
or roughly 0.5 s. Velocity crosses zero *at* apogee. But differentiating the
raw barometer gives velocity noise of about σ·√2 / dt ≈ 70 m/s at 100 Hz,
which is useless.

The accelerometer is smooth, but integrating it drifts: any bias integrates
into a velocity error that grows linearly. The barometer is noisy, but it
doesn't drift.

The Kalman filter fuses the two optimally:
* The accelerometer supplies the **short-term** shape.
* The barometer pins the **long-term** level.

### 7.2 State vector

```
x = [ h   altitude above ground (m)
      v   vertical velocity (m/s, +up)
      a   vertical kinematic acceleration (m/s², +up; −9.81 in free fall) ]
```

**Why `a` is a state, not a control input.** The common alternative is a
2-state [h, v] filter that uses the accelerometer as a known input u in the
prediction. I rejected it, for three reasons:

1. The accelerometer is a **noisy measurement**, not a perfect input. As a
   measurement it gets weighted by its R. As an input its noise enters the
   state unfiltered.
2. Each accelerometer sample gets an **innovation**. That's what allows
   accelerometer outliers to be gated out in milestone 4. An input can't be
   rejected.
3. If accelerometer samples are missing, the 3-state filter keeps predicting
   with the last acceleration. A 2-state filter has nothing to integrate.

### 7.3 The matrices

**F: state transition**, constant acceleration over the step dt.

```
F(dt) = [ 1  dt  dt²/2 ]        h' = h + v·dt + a·dt²/2
        [ 0  1   dt    ]        v' = v + a·dt
        [ 0  0   1     ]        a' = a
```

This is exact kinematics *if* the acceleration is constant over dt. It isn't,
since drag changes as speed changes. Q accounts for the mismatch.

**Q: process noise.** It is modelled as continuous white-noise **jerk** with
spectral density q:

```
Q(dt) = q · [ dt⁵/20  dt⁴/8  dt³/6 ]
            [ dt⁴/8   dt³/3  dt²/2 ]
            [ dt³/6   dt²/2  dt    ]
```

* **Physically:** "the acceleration is not really constant; it wanders like a
  random walk, at a rate set by q". In coast it changes from −52 m/s² at
  burnout to −9.8 m/s² at apogee as drag fades.
* **Why this form and not a hand-picked diagonal:** it is the *exact
  integral* of the white jerk over the step. Two steps of dt therefore
  accumulate exactly the uncertainty of one step of 2dt, F·Q(dt)·Fᵀ + Q(dt) =
  Q(2dt) (unit test `ProcessNoiseIsExactWhiteJerkIntegral`).
  * A skipped frame or irregular timing automatically gets the right
    uncertainty growth.
  * A per-step Q tuned for 10 ms would be wrong for a 500 ms gap.

**H: measurement models.** Each sensor observes one state directly:

```
barometer:      H_b = [1 0 0]     z_b = h_AGL
accelerometer:  H_a = [0 0 1]     z_a = accel − accel_ref  (kinematic, bias removed)
```

**R: measurement noise**, taken from the sensor model, not tuned:

```
R_b = σ_b² + Δ²/12 = 0.5² + 0.1²/12 = 0.2508 m²      (white noise + quantization)
R_a = σ_a²         = 0.5²           = 0.25 (m/s²)²
```

* Quantization to a step Δ adds uniformly distributed error with variance
  Δ²/12, independent of the Gaussian noise, so the variances add.
* **Biases are deliberately *not* put into R.** R models *zero-mean white*
  noise. A bias is a constant offset, and inflating R can't remove it, only
  slow the filter down. Biases are removed instead by the pad references:
  * the ground reference, for the barometer (§3);
  * `accel_ref`, for the accelerometer. On the pad the true acceleration is
    exactly 0, so the mean pad reading *is* g + bias. Subtracting it removes
    gravity and the bias in one step. Unit test:
    `AccelReferenceLearnsGravityPlusBiasOnPad`.

### 7.4 Implementation choices

| Choice | Why |
|---|---|
| **Two sequential scalar updates** per frame (barometer, then accelerometer) | With diagonal R this equals one joint update exactly. The innovation variance S is then a scalar, so the filter never inverts a matrix: no numerical risk, no linear-algebra library |
| **Joseph-form covariance update** P = (I−KH)P(I−KH)ᵀ + KRKᵀ, then symmetrise | Algebraically equal to (I−KH)P, but it stays symmetric positive semidefinite under rounding. The short form can lose definiteness over long runs, after which the filter believes it is *more* certain than possible and diverges. Unit test: 20,000 steps with dt from 0.1 ms to 2 s, checking PSD after every predict and update |
| **dt from timestamps** in every predict | Dropped frames are simply a longer predict step (unit test `HandlesSkippedFrames`). The same noiseless data at 100 Hz or on an irregular grid gives the same answer (`IrregularDtGivesSameAnswerOnExactModel`) |
| **Fixed-size `std::array` matrices** | No heap, no dependencies, deterministic. That suits an embedded FC |
| **Start on the pad** at x = [h_AGL, 0, 0], P = diag(R_b, 1, 1) | The filter converges during the pad sit. The first second shows a small transient (|v| up to about 0.7 m/s) while `accel_ref` is still being learned. After that, pad velocity stays within ±0.2 m/s |

### 7.5 Apogee from the filter

**Condition:** in COAST, the estimated velocity `v < 0` for **5 consecutive
samples** (50 ms).

* Near apogee v falls at about 9.8 m/s², so 50 ms after the true crossing,
  v ≈ −0.5 m/s. That is about 6× the estimate's noise (0.08 m/s RMS), so a
  single noisy crossing can't confirm.
* The window is the main source of lag: about 40 ms plus filter error.
* *Too few samples* (1): in clean flights it's actually fine, but it's
  fragile against a single bad estimate, such as an outlier slipping
  through.
* *Too many* (50): adds 0.5 s and hands back the baseline's whole advantage.
* *Rejected: predictive deploy* (fire when v < +0.4 m/s, i.e. "apogee in
  40 ms"). It removes the systematic +0.046 s, but turns a harmless known delay
  into a risk of deploying while still climbing. Deploying 46 ms late means
  the rocket is descending at 0.45 m/s, which is physically irrelevant.

### 7.6 How Q and R were tuned, and what mistuning does

**R comes from the sensor model** (§7.3), so no tuning is needed. **q is the
only real tuning knob.** I chose it with a closed-loop SIL sweep: 20 seeded
nominal flights per value, using `--param kf.jerk_psd=q`.

| q (m²/s⁵) | Deploy − true apogee (mean ± std) | Velocity RMS in coast | Velocity bias near apogee |
|---:|---:|---:|---:|
| 0.01 | **−0.044 ± 0.009 s (early)** | 3.51 m/s | −0.92 m/s |
| 0.1 | +0.038 ± 0.009 s | 0.42 m/s | −0.08 m/s |
| 1 | +0.046 ± 0.010 s | 0.081 m/s | +0.00 m/s |
| **10 (chosen)** | **+0.047 ± 0.010 s** | **0.073 m/s** | +0.01 m/s |
| 100 | +0.047 ± 0.010 s | 0.072 m/s | +0.01 m/s |
| 1000 | +0.046 ± 0.010 s | 0.074 m/s | +0.01 m/s |
| 10000 | +0.049 ± 0.011 s | 0.118 m/s | +0.02 m/s |

**Reading the table:**
* **q too small: a systematic, dangerous error.** The model insists the
  acceleration hardly changes, so `a` lags the real (fading) drag. The
  integrated velocity is biased low, and it crosses zero *before* apogee. At
  q = 0.01 the FC deploys 44 ms early on every flight, and with a stronger
  mismatch it would be earlier still. A pytest locks this failure mode in
  (`test_mistuned_process_noise_biases_velocity_and_deploys_early`). Since milestone 4, the innovation-consistency check catches this mistuning: the stiff model keeps disagreeing with the barometer, so the FC declares itself inconsistent and deploys on the backup timer instead of early (`test_mistuned_filter_is_caught_by_consistency_check`).
* **q too large: a random, benign error.** The filter trusts every
  accelerometer sample, and the estimate gets noisier.
* **The plateau runs from 1 to 1000.** q = 10 sits two decades from either
  edge, so the design is insensitive to the exact value.
  * Physical sanity check: the √(q·1 s) ≈ 3 m/s² of acceleration wander per
    √s is the same order as how fast drag deceleration actually changes in
    coast (about 7.6 m/s² per s on average).

**Mistuning R** (q = 10, 20 seeds each):

| Filter R | Deploy − apogee | Velocity RMS | Altitude RMS | Effect |
|---|---:|---:|---:|---|
| Matched to the sensor model | +0.047 ± 0.010 s | **0.073** | **0.068** | Best on every metric, as theory predicts for a matched filter |
| R_b × 0.01 | +0.049 ± 0.018 s | 0.171 | 0.095 | Over-trusts the barometer, so its noise leaks into velocity |
| R_b × 100 | +0.046 ± 0.017 s | 0.118 | 0.152 | Ignores the barometer, so it drifts more on the accelerometer |
| R_a × 0.01 | +0.046 ± 0.013 s | 0.106 | 0.123 | Over-trusts accelerometer noise |
| R_a × 100 | +0.049 ± 0.018 s | 0.675 | 0.158 | `a` becomes sluggish and velocity lags the real dynamics |

**Takeaway.** Deploy *timing* is robust to large R errors, because apogee is a
zero crossing of a steep signal (dv/dt = −g). Estimate *quality* is not:
mismatched R costs 1.5–9× in velocity accuracy. So getting the *model* right
(q) matters more than getting the noise numbers exactly right.

### 7.7 Results: baseline vs Kalman

There are 50 seeded nominal SIL flights per detector. Up to deployment both
detectors see **identical sensor noise**, because the noise stream is drawn
per tick, independent of the FC.

| Detector | Mean | Std | Min | Max | |error| p95 | Within 0.5 s |
|---|---:|---:|---:|---:|---:|---:|
| Baseline (raw barometer) | +0.509 s | 0.085 s | +0.252 s | +0.642 s | 0.618 s | 34 % |
| **Kalman** | **+0.046 s** | **0.008 s** | +0.022 s | +0.062 s | 0.062 s | **100 %** |

Kalman is closer to true apogee on **50/50** seeds. It is about **11× less
late** and **10× less variable**. Its remaining +0.046 s is almost entirely
the deliberate 5-sample confirmation window.

Reproduce: `python scripts/run_sil.py --compare --seeds 50`.

### 7.8 Known gap (closed in milestone 4, see §8)

A plain Kalman filter trusts every measurement. If the barometer **sticks**
while the rocket climbs, the filter is told "altitude is constant", drags the
velocity toward zero, and could declare apogee early, at speed. Spikes and
outliers pull the estimate the same way. Milestone 4 adds **innovation
gating** for this.

## 8. Faults: detection and tolerance

Fault injection lives in [`sim/faults.py`](../sim/faults.py). Fault handling in
the FC lives in `StateMachine::check_health` and `run_filter`
([`fc/src/state_machine.cpp`](../fc/src/state_machine.cpp)).

### 8.1 Fault model

| Kind | Sensor | Effect while active | Typical test values |
|---|---|---|---|
| `dropout` | link | Frame not sent, with per-frame probability (1 = blackout) | 1.5 s blackout across apogee; 30 % random loss |
| `stuck` | baro / accel | Value frozen at the first reading in the window | Frozen in boost (0.5 s) or coast (3 s, 5 s) |
| `spike` | baro / accel | ± magnitude added with per-frame probability | Baro ±50 m, accel ±80 m/s², 10 % of frames |
| `drift` | baro / accel | Bias growing linearly from start | Baro ±2 m/s; accel ±0.5 m/s² per s |
| `hang` | FC | Process stops responding forever (`--inject-hang-at`) | On the pad, in boost, in coast, after deploy |

**Fault randomness is isolated.** Which frames drop or spike, and the spike
signs, come from a seeded stream that is separate from sensor noise. Every
fault draws the same numbers every tick, active or not. A faulted run
therefore differs from its nominal twin *only* by the fault, which a test
checks (`test_faults_do_not_shift_sensor_noise`). That's what makes
"nominal vs faulted" a controlled experiment.

### 8.2 Innovation gating (outliers)

For each measurement the filter already predicts what it expects to see, and
how uncertain that prediction is:

* innovation y = z − Hx,
* innovation variance S = HPHᵀ + R.

The gate rejects the measurement if **y² > 5² · S**, that is, if it lies more
than 5σ from what the filter expects. A rejected measurement changes neither
the state nor the covariance.

**Why 5σ.**
* For a correct filter, |y| > 5σ happens by chance about 6 × 10⁻⁷ of the
  time. Nominal flights effectively never lose a good sample (no rejections
  in 30 noisy test flights).
* Outliers bigger than about 2.6 m (baro) or 2.8 m/s² (accel) are refused.
* *Too tight* (2σ): about 5 % of good data is thrown away. Worse, ordinary
  model mismatch (the filter's own lag) starts tripping the consecutive-
  rejection limit below, and nominal flights would fall back to the timer.
* *Too loose* (20σ): spikes of up to about 10 m leak in and kick the velocity
  estimate.

**Why gate only in COAST.** Ignition, burnout and the chute snatch are *real*
steps in acceleration: about +200 m/s² at ignition and −80 m/s² at burnout.
A gate would reject them as outliers, and then reject every following sample
too, because the filter would never catch up. It would diverge exactly at
burnout. In COAST the true motion is smooth (gravity plus slowly fading
drag), so a big innovation there really does mean a bad measurement. COAST is
also the only state where the apogee decision is made, so gating is applied
exactly where it matters and nowhere it hurts.

**Effect.** With 10 % of frames spiking (baro ±50 m or accel ±80 m/s²), the
Kalman FC still deploys +0.05 s after apogee, the same as nominal. The
raw-barometer baseline, fed the same spikes, deploys **seconds early**: one
spike inflates its running maximum. That's the strongest single argument for
the filter (`test_baseline_detector_is_fooled_by_baro_spikes`).

### 8.3 Stuck sensors and graceful degradation

**Detection.** The FC counts how many readings in a row are *bit-identical*.
A live sensor with real noise essentially never repeats exactly:

* **Barometer** (σ 0.5 m, 0.1 m steps): two readings share a step about 6 % of
  the time. **10 in a row** happens by chance about 5 × 10⁻¹² per window.
* **Accelerometer** (sent to 10⁻⁴ m/s²): **5 identical readings** are
  essentially impossible when the sensor is live.
* **Exception:** readings at accelerometer full scale (±24 g) never count.
  A saturated accelerometer legitimately repeats its limit during a hot boost,
  and must not be declared dead for it.

**Degradation once a sensor is declared failed** (latched, logged as a
`HEALTH` event):

| Failed sensor | Kalman filter | Apogee decision | Launch / burnout | Landing |
|---|---|---|---|---|
| Barometer | Accelerometer only (dead reckoning, bias calibrated on the pad) | Kalman v < 0 | Accelerometer paths only | Not detected (no altitude) |
| Accelerometer | Barometer only | **Baseline** raw-barometer detector | Baro launch backup; burnout via time + altitude fallback | Normal |

**Measured.**
* Barometer stuck at 3 s: the FC isolates it at 3.09 s, flies 4.3 s on the
  accelerometer alone, and deploys +0.04–0.05 s after apogee. Dead reckoning
  is good for a few seconds because the pad calibration removed the bias.
* Accelerometer stuck in boost: detected, burnout comes from the fallback at
  launch + 3 s, and apogee from the baseline at about +0.54 s.

**Why a dedicated stuck detector instead of relying on the gate.** With two
sensors, a gate alone can tell *that* they disagree, not *which one* is wrong.
Consider an accelerometer stuck at a coast value:
1. The filter's acceleration freezes, so velocity drifts away from the truth.
2. The *correct* barometer now produces big innovations and gets gated.
3. A rule like "blame the gated sensor" would throw away the good barometer
   and keep the stuck accelerometer.

The bit-identical test identifies the stuck sensor *directly*, independent of
the filter.

*Limitation:* a sensor that fails "noisily wrong" (alive, but offset) isn't
caught by this. It is caught by 8.4 instead.

### 8.4 Disagreement the FC can't isolate: fall back to the timer

If either sensor is gated out for **25 frames in a row** (0.25 s) in COAST,
that's not an outlier. Either that sensor or the estimate is persistently
wrong, and with two sensors the FC can't always tell which.

The FC then declares the estimator **inconsistent** (latched), disables apogee
detection entirely (both the Kalman and the baseline detector), and lets the
**backup timer** deploy at launch + 8.5 s.

* *Why the timer and not "trust the barometer".* The barometer may be the
  faulty one, for example after an offset jump. The timer is the only
  decision source that depends on neither sensor. It is guaranteed to deploy,
  and REQ-004 explicitly accepts it.
* *Measured cost.* Accelerometer drift of ±0.5 m/s² per s (deliberately
  exaggerated; real MEMS thermal drift is far slower) leads to an inconsistent
  estimator, and the timer deploys at +1.13 s, 11 m/s descending. That is
  inside REQ-004, and a much better outcome than trusting a drifting estimate.
* *25 frames:* longer than the 10 frames the stuck detector needs, so a stuck
  barometer is isolated (graceful degradation) *before* it could be declared
  "inconsistent" (timer fallback).
* *Rejected: re-sync after N rejections* (accept the measurement and reset
  P). Against a stuck or offset sensor, that drags the filter onto the bad
  value every N frames. It trades a safe, late deploy for an unsafe, possibly
  early one.
* *Rejected, but future work: a 4-state filter with an accelerometer-bias
  state.* The barometer makes a slowly varying accelerometer bias observable,
  so drift would be *estimated* instead of *tripping* the gate. It costs
  tuning complexity, and it can mask a real accelerometer failure as "bias".
  The static bias is already handled by the pad calibration.

### 8.5 Per-fault summary

Results are for seed 21, Kalman FC (12 fault cases in
`test_sil_m4_faults.py`); Monte Carlo statistics are in the
[verification report](verification_report.md).

| Fault | How the FC detects / tolerates it | Outcome | Requirement |
|---|---|---|---|
| Blackout across apogee (1.5 s) | Kalman predicts across the gap (dt from timestamps); persistence counters reset on gaps | Deploys on the first frames after the gap, about +0.6 s | REQ-004 |
| 30 % random frame loss | Same; dt-correct Q | About +0.08 s | REQ-004 |
| Stuck barometer | Stuck detector → accelerometer-only filter | About +0.05 s | REQ-004 |
| Stuck accelerometer | Stuck detector → baseline detector, burnout fallback | About +0.5 s | REQ-004 |
| Baro / accel spikes | Innovation gate (COAST) | About +0.05 s | REQ-004 |
| Baro drift ±2 m/s | Not detected. It looks like a velocity offset | −0.15 to +0.26 s | REQ-004 |
| Accel drift ±0.5 m/s² per s | Persistent rejections → inconsistent → timer | Timer, +1.13 s | REQ-004 |
| FC hang (any phase) | Simulator watchdog (no reply in 2 s) | Motor C6-7 charge deploys, about +1.4 s | REQ-005 |
| Any fault in boost | Lockout (2 layers) + 2.5 s deploy inhibit | Never deploys in boost | REQ-003 |

### 8.6 Redundancy: the motor charge is an independent backup

The physics simulation keeps the motor's ejection charge active in every SIL
run: burnout + 7 s for a C6-7, about 1.4 s after apogee. It doesn't depend on
the FC in any way.

Each flight logs **which mechanism deployed first**:
`deployment["mechanism"]` is `fc` or `motor`, and the FC's own reason
(`apogee` / `backup_timer`) comes from its reply stream.

The result is three layers:
1. the FC's apogee detector;
2. the FC's backup timer, for detection failures;
3. the motor charge, for a dead FC.

Each layer covers the failure modes of the one above it. This mirrors real
dual-deploy hobby practice.

### 8.7 Deploy inhibit

No apogee decision is accepted until **2.5 s after launch**, longer than the
C6's 1.86 s burn, whatever the burnout detector concluded.

* This is defence in depth for REQ-003. A fault that fakes burnout early (an
  accelerometer dropout during boost, a negative drift) plus a fault that fakes
  apogee would otherwise need only one more coincidence to fire under thrust.
* The cost is nothing on real flights: apogee is about 7 s after launch.
* *Too long* (say 8 s) would collide with the apogee time.
* *Motor-specific:* a longer-burning motor needs this raised, like the timer.

### 8.8 What is not handled (honest limits)

* **Double faults.** Examples: barometer stuck *and* accelerometer drifting,
  or a stuck accelerometer during baro spikes, where the baseline fallback has
  no gate. Requirements are written for single faults.
* **Slow barometer drift** shifts apogee timing by about drift/g. It's
  undetectable from inside the FC without a third reference.
* **No altitude after a barometer failure** means no landing detection.
* **A false launch from an accelerometer that fails high on the pad** is safe
  (no deploy), but it corrupts the timer's time base for a real launch later. A
  pre-launch self-test (accelerometer reads 1 g ± 0.5 g, barometer steady,
  before arming) is the right fix on hardware.
