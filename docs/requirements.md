# Flight computer requirements

These are the verifiable requirements for the SIL flight computer (`fc/`).

* **Machine-readable copy:** [`sim/requirements.py`](../sim/requirements.py),
  which also holds the per-run checks.
* **How tests are tagged:** each test carries the IDs it verifies.
  * pytest: `@pytest.mark.req("REQ-00N")`
  * GoogleTest: a name ending in `_REQ00N`
* **Results:** the traceability matrix and pass rates are generated into
  [`verification_report.md`](verification_report.md) by
  `scripts/verification_report.py`.

**Scope.** The FC detects flight events and commands parachute deployment
only. There is no guidance, steering or attitude control.

**Definitions.**

| Term | Meaning |
|---|---|
| **True apogee** | Where the rocket would peak with no parachute. If the chute opened while climbing, it comes from the simulator's chute-free shadow coast (see the README) |
| **FC deploy time** | The first tick where the FC's reply carries `deploy = 1`, even if the motor charge had already deployed the chute |
| **Nominal** | Nominal sensor noise and dispersions, with no injected fault |
| **Single fault** | Exactly one injected fault from [`sim/faults.py`](../sim/faults.py) |

---

## Requirements

### REQ-001: Nominal deploy accuracy
**The FC shall command deployment within 0.5 s of true apogee on a nominal
flight.**

* *Rationale.* A chute opened at apogee sees the lowest speed (a few m/s), so
  the lowest shock load and drift.
  * At 0.5 s after apogee the rocket is falling at about 5 m/s, which a hobby
    chute absorbs easily.
  * At 0.5 s *before* apogee it is climbing at about 5 m/s, which is the same
    load and acceptable.
  * Beyond about ±1.5 s, loads and drift grow quickly.
* *Verification.*
  * Analysis by Monte Carlo SIL (all nominal runs).
  * Tests: `test_kalman_sil_flight_deploys_at_apogee`,
    `test_baseline_vs_kalman_same_seeds`,
    `test_nominal_run_passes_all_applicable_requirements`.
  * C++: `KalmanApogeeOnNoisyParabola`.
* *Applies to* the default (Kalman) FC. The raw-barometer baseline is reported
  for comparison and does **not** meet this requirement reliably (34 % of
  flights within 0.5 s).

### REQ-002: No false launch
**Zero false launch detections across 1000 seeded 60 s pad sits.**

* *Rationale.* A false launch arms the FC on the pad. Everything after it
  (timers, the burnout fallback) runs from a wrong time base. Pad sits are
  long (minutes), so the false-trigger rate per second must be effectively
  zero.
* *Verification.*
  * C++: `NoFalseLaunchIn1000SixtySecondPadSits_REQ002` (1000 × 6000 frames,
    sensor-model noise), `NoLaunchOnSixtySecondsOfPadNoise_REQ002`.
  * Python, through the real process: `test_no_false_launch_on_pad_through_real_fc`.
  * The 1000-sit campaign in `scripts/run_sil_montecarlo.py`.
  * Also requires no false *sensor-failure* declarations on the pad.

### REQ-003: No deployment on the pad or under thrust
**The FC shall never command deployment while the rocket is on the pad or under
thrust (true phase PAD, RAIL or BOOST), under any fault, and never while its own
state is PAD or BOOST.**

* *Rationale.* This is the one failure that destroys the vehicle. A chute
  opening at about 100 m/s or under thrust zippers the airframe or shreds the
  canopy.
* *Design.*
  * Deployment is possible only from COAST.
  * The output bit is masked in PAD and BOOST.
  * There is a 2.5 s post-launch deploy inhibit.
  * A false launch on the pad can't leave BOOST (the burnout fallback needs
    altitude).
* *Verification.*
  * The per-run check (truth burnout time vs. FC commands) on every
    Monte Carlo run, nominal and faulted.
  * Tests: `test_single_fault_meets_requirements[*]`,
    `test_fc_hang_detected_and_chute_still_deploys[*]`,
    `test_no_deploy_command_in_pad_or_boost`.
  * C++: `NoDeployInPadOrBoostEvenIfBaroFalls_REQ003`,
    `FalseLaunchOnPadNeverReachesDeployableState_REQ003`,
    `DeployInhibitBlocksApogeeSoonAfterLaunch_REQ003`.

### REQ-004: Deploy under any single fault
**Under any single sensor or link fault (dropout, stuck, spike or drift), the FC
shall command deployment within 1.5 s of true apogee, or via its backup timer. A
run where the FC never commands deployment fails.**

* *Rationale.* Graceful degradation. A fault may cost accuracy, but it must
  never cost the deployment. The backup timer is an accepted outcome: it is the
  designed answer when the FC detects a fault it can't isolate.
* *Verification.*
  * Monte Carlo SIL with a random single fault per run.
  * Tests: `test_single_fault_meets_requirements[*]` (12 fault cases),
    `test_stuck_baro_is_isolated_*`, `test_stuck_accel_is_isolated_*`,
    `test_spikes_are_gated_out`,
    `test_unisolated_disagreement_falls_back_to_backup_timer`.
  * C++: `*_REQ004` in `test_faults.cpp`.
  * FC hangs are covered by REQ-005, not here.

### REQ-005: FC hang tolerated
**If the FC stops responding, the watchdog shall detect it and the parachute
shall still deploy, by an independent backup if the FC had not already
deployed: the motor's ejection charge, or, on rockets flying plugged motors,
a pre-set backup timer device.**

* *Rationale.* Software can hang. Real hobby rockets keep the motor ejection
  charge as an independent backup to electronic deployment. With a C6-7 it
  fires about 1.4 s after apogee.
* *Verification.*
  * Tests: `test_fc_hang_detected_and_chute_still_deploys[*]` (hang on the pad,
    in boost, in coast, and after deployment), `test_watchdog_fires_on_hung_fc`,
    `test_watchdog_detects_exited_fc`,
    `test_sil_watchdog_end_to_end_backup_charge_deploys`.
  * Monte Carlo hang runs.
  * The hang is genuine: the FC process blocks, via its test-only flag
    `--inject-hang-at`.
* *Fleet note.* On the dual-deploy rockets (Swift, Argo), a hung FC also
  means no main chute. The backup device deploys only the drogue, and the
  rocket lands on the drogue at about 20 m/s: the chute is deployed, but the
  landing is hard. Real high-power flyers solve this with a second,
  independent altimeter on the main channel; that is future work here (see
  below).

### REQ-006: Determinism
**Identical input to the FC shall produce identical output.**

* *Rationale.* A failure seen once must be replayable exactly, from its seed,
  to debug and to prove a fix. That's why the FC uses only frame timestamps,
  never the wall clock, and why all randomness is seeded.
* *Verification.*
  * Tests: `test_sil_run_is_reproducible`, `test_faulted_runs_are_deterministic`.
  * C++: `IdenticalInputGivesIdenticalOutput_REQ006`,
    `DeterministicForIdenticalInput_REQ006`.
  * Monte Carlo: serial run == parallel run, and replayed worst cases.

### REQ-007: Bad replies never deploy
**A malformed, garbled or stale (wrong-timestamp) FC reply shall never cause a
deployment, and shall be counted as a protocol error.**

* *Rationale.* A pyro channel must fire only on a valid, explicit command.
* *Verification.*
  * Tests: `test_sim_treats_malformed_fc_replies_as_no_command`,
    `test_parse_reply_rejects_malformed[*]`.

### REQ-008: Protocol robustness
**The FC shall answer every input line with exactly one line, and shall survive
malformed input.**

* *Rationale.* Lockstep must not slip by one line. A bad input frame must
  never crash the FC, which would leave the rocket on its backup charge alone.
* *Verification.*
  * Tests: `test_protocol_round_trip_with_fc_process`,
    `test_fc_rejects_malformed_lines_and_keeps_running`.
  * C++ `Protocol.*` and `Runner.*` tests.

### REQ-009: Timer never pre-empts the detector
**On a nominal flight, the FC's deploy decision shall come from apogee
detection, never from its backup timer.**

* *Rationale.* A backup timer set too short is a *silent, systematic* failure:
  every flight would deploy on the timer, possibly before apogee, and nobody
  would notice from the outcome alone.
* *Verification.*
  * Monte Carlo (all nominal runs), plus `test_kalman_sil_flight_deploys_at_apogee`
    and `test_backup_timer_does_not_preempt_nominal_detection`.
  * `test_requirement_checks_detect_violations` shows the check catches a
    deliberately mis-set timer.

### REQ-010: Landing detection
**On a nominal flight, the FC shall report LANDED within 10 s of touchdown.**

* *Rationale.* On hardware, LANDED triggers the locator beacon, the log flush
  and low power. It isn't safety-critical, but it is functional.
* *Verification.*
  * Monte Carlo (nominal runs with ≥ 10 s of post-landing frames),
    `test_fc_events_track_truth`.
  * Not required after a barometer failure: the FC then has no altitude
    source, and it skips landing detection by design.

### REQ-011: No deployment before the final burnout (multi-burn rockets)
**On a staged rocket the FC shall never command any deployment, drogue or
main, from lift-off until the final motor burnout: not during either boost,
and not in the inter-stage gap, under any single fault.**

* *Rationale.* The inter-stage gap is a coast phase. An FC built for one burn
  could treat it as the coast to apogee, or treat the sustainer's thrust step
  as a sensor outlier. A deployment then would open the chute in the
  sustainer's exhaust at hundreds of m/s.
* *Design.*
  * The FC is configured before flight with the number of burns (`burns`).
    It never commands staging.
  * While burns remain, apogee detection, deployment and innovation gating
    are locked out.
  * Any detected thrust in coast returns the FC to BOOST.
* *Verification.*
  * The per-run check on every Argo Monte Carlo run, nominal and faulted.
  * `test_two_stage_never_deploys_before_final_burnout_under_faults[*]`
    (spikes, a blackout across separation, a dead accelerometer, a stuck
    barometer in the gap).
  * C++: `TwoBurnSequenceNoDeployBeforeFinalBurnout_REQ011_REQ012`,
    `NoDeployInTheGapEvenWithAnApogeeLikeBaro_REQ011`.

### REQ-012: Staging is never mistaken for apogee or landing
**On a staged rocket the FC shall report no APOGEE, DESCENT or LANDED state
before the final motor burnout.**

* *Rationale.* The interface of REQ-011. A state machine that "sees" apogee
  at booster burnout has already lost track of the flight, even if a lockout
  stops the pyro channel firing.
* *Verification.*
  * The per-run check on every Argo run.
  * `test_two_stage_staging_is_handled_explicitly`.
  * C++ `TwoBurnSequence*_REQ012`.

### REQ-013: Main chute at its altitude (dual deploy)
**On a nominal dual-deploy flight the FC shall command the main chute after
the drogue, during descent, within 25 m of the configured main deploy
altitude.**

* *Rationale.*
  * Deploying the main high (at apogee) multiplies drift.
  * Deploying it too low leaves too little time to inflate and slow down.
  * 25 m at about 20 m/s on the drogue is just over 1 s of descent: well
    inside the margin a 300 m main altitude leaves.
* *Verification.*
  * Monte Carlo (nominal Swift and Argo runs).
  * `test_dual_deploy_main_commanded_near_its_altitude[*]`.
  * C++ `MainCommandedBelowItsAltitudeAfterDrogue_REQ013`,
    `FailedBaroCommandsMainWithDrogue_REQ013` (the designed fallback: no
    altitude, so the main goes with the drogue).

---

## Why these requirements, and what's missing

* REQ-001 to REQ-006 come from the SIL project brief. REQ-011 to REQ-013
  were added for the fleet (two burns, dual deploy).
* I added REQ-007 to REQ-010 because each closes a gap the first six leave
  open:
  * bad replies (REQ-007) and bad inputs (REQ-008) are the integration
    boundary itself;
  * a mis-set timer (REQ-009) would pass every accuracy check;
  * landing (REQ-010) is the last state transition.
* **Not covered** (out of scope for a SIL hobby FC):
  * real-time deadlines and CPU load (a HIL concern);
  * power loss and brown-out reboot in flight;
  * multiple simultaneous faults;
  * pyro-channel continuity checks;
  * a pre-launch self-test (accelerometer reads 1 g, barometer steady);
  * a redundant altimeter for the main channel of dual-deploy rockets
    (REQ-005 fleet note);
  * pressure-port errors near Mach 1 (the barometer model has no transonic
    pressure disturbance, so no Mach lockout is modelled).

  The walkthrough discusses each one as future work.
