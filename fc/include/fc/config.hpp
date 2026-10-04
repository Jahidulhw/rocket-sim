#pragma once

// Every flight-computer threshold in one place. Values are tuned for the
// default rocket (Estes C6, ~314 m apogee at ~7.4 s) and the sensor models in
// sim/sensors.py (baro sigma 0.5 m, accel sigma 0.5 m/s^2, 100 Hz).
// docs/fc_walkthrough.md explains each value, and what goes wrong if it is
// too high or too low.
//
// Units: s, m, m/s, m/s^2. "accel" is vertical SPECIFIC FORCE (+g at rest).

#include <string_view>

#include "fc/kalman.hpp"

namespace fc {

enum class ApogeeMode {
  Kalman,    // Kalman velocity estimate crosses zero (default)
  Baseline,  // raw barometer: N samples a margin below the running maximum
};

struct FcConfig {
  ApogeeMode apogee_mode = ApogeeMode::Kalman;

  // Persistence counters ("N consecutive samples") are a minimum-duration
  // filter. A gap between frames longer than this breaks the run: we have no
  // evidence about what happened during the gap. 5 nominal periods.
  double max_frame_gap_s = 0.05;

  // Ground reference, so altitudes are above ground and a constant baro bias
  // cancels. Plain mean over the first second of frames, then an EMA (tau 2 s:
  // residual noise ~0.03 m, follows weather drift easily) that only accepts
  // samples within +-3 m of the current reference. The gate stops the
  // reference creeping up with the rocket during the first metres of a climb
  // (which would delay baro launch detection) and rejects spikes on the pad.
  double ground_init_s = 1.0;
  double ground_tau_s = 2.0;
  double ground_gate_m = 3.0;
  // Accelerometer reference, learned the same way (same init window and tau).
  // On the pad the true acceleration is exactly zero, so the mean reading IS
  // g + bias; the filter uses a = accel - accel_ref. Gate: 4 sigma of noise.
  double accel_ref_gate_mps2 = 2.0;

  // ---- PAD -> BOOST ------------------------------------------------------
  // Primary: accel > 25 m/s^2 (~2.5 g, i.e. 1.5 g above rest) for 5 samples.
  // On the pad the reading is 9.81 + 0.2 bias, sigma 0.5: 25 is 30 sigma
  // away. Boost is above 42 m/s^2 for its whole duration after ~0.05 s.
  // 5 samples (50 ms) rejects single-sample bumps and outliers.
  double launch_accel_mps2 = 25.0;
  int launch_accel_samples = 5;
  // Backup: barometric altitude > 15 m above ground for 10 samples, so a dead
  // or stuck accelerometer cannot keep the FC on the pad for the whole flight.
  // 15 m is 30 sigma of baro noise and still reached ~0.6 s after liftoff.
  double launch_baro_agl_m = 15.0;
  int launch_baro_samples = 10;

  // ---- BOOST -> COAST ----------------------------------------------------
  // Burnout: accel < 5 m/s^2 for 5 samples. While climbing after burnout the
  // reading is negative (drag adds to gravity: about -43 m/s^2 at burnout),
  // while during boost it never drops below ~40. 5 sits in the wide gap.
  double burnout_accel_mps2 = 5.0;
  int burnout_samples = 5;
  // Fallback (stuck accelerometer): leave BOOST anyway once 3 s have passed
  // since launch (C6 burn: 1.86 s) AND the baro confirms real climb. The
  // altitude condition stops a false launch on the pad ever reaching COAST,
  // where deployment is allowed.
  double max_boost_s = 3.0;
  double burnout_fallback_agl_m = 30.0;

  // ---- COAST -> APOGEE (baseline) ----------------------------------------
  // Apogee when raw altitude stays >= 1.5 m below the highest altitude seen
  // since launch for 10 consecutive samples (100 ms). See the walkthrough for
  // why "N strictly decreasing samples" was rejected (noise alone triggers it).
  double apogee_drop_m = 1.5;
  int apogee_samples = 10;

  // ---- COAST -> APOGEE (Kalman) ------------------------------------------
  // Apogee when the estimated vertical velocity has been negative for 5
  // consecutive samples (50 ms). Near apogee v falls at ~9.8 m/s^2, so 50 ms
  // after the true crossing v is already ~-0.5 m/s, several times the
  // estimate's noise; the window adds 40 ms of lag and rejects a single
  // noisy crossing.
  int kalman_apogee_samples = 5;
  KalmanConfig kf{};

  // ---- Backup timer -------------------------------------------------------
  // Deploy if apogee has not been detected 8.5 s after launch: nominal apogee
  // is ~7.4 s after launch with +-0.2 s (1 sigma) dispersion, so 8.5 s never
  // pre-empts a working detector, and the rocket is then only ~10 m/s into
  // its fall.
  double backup_timer_s = 8.5;

  // ---- Fault handling ----------------------------------------------------
  // Stuck sensor: N bit-identical readings in a row. With baro noise 0.5 m and
  // 0.1 m quantization, two live readings share a bin ~6 % of the time, so 10
  // in a row happens by chance ~5e-12 per window; the accelerometer is sent to
  // 1e-4 m/s^2, so 5 identical live readings are essentially impossible.
  // (Readings at accelerometer full scale are exempt: saturation repeats.)
  int stale_baro_samples = 10;
  int stale_accel_samples = 5;
  double accel_full_scale_mps2 = 24.0 * 9.81;
  // Innovation gate (COAST only), in sigmas of the predicted innovation:
  // 5 sigma rejects a good sample with probability ~6e-7, so nominal flights
  // never lose data, while outliers of more than ~2.6 m (baro) or ~2.8 m/s^2
  // (accel) are refused.
  double kf_gate_sigma = 5.0;
  // 25 consecutive rejections (0.25 s) = persistent disagreement, not an
  // outlier: apogee detection is disabled and the backup timer decides.
  int max_consecutive_rejections = 25;
  // No apogee deploy until 2.5 s after launch (C6 burn: 1.86 s), regardless
  // of what the burnout detector concluded.
  double min_deploy_after_launch_s = 2.5;

  // ---- DESCENT -> LANDED ---------------------------------------------------
  // Landed when altitude stays within +-3 m of a reference sample for 5 s.
  // Under the chute the rocket sinks ~3.8 m/s and leaves a 3 m band in <1 s;
  // baro noise exceeds 3 m between two samples (sigma*sqrt(2) = 0.71 m) with
  // probability ~3e-5 per sample.
  double landing_band_m = 3.0;
  double landing_duration_s = 5.0;
  // The band test runs on a low-pass filtered altitude (EMA, tau 0.5 s;
  // residual noise ~0.05 m). On RAW samples, a window restart adopts the
  // out-of-band sample (a noise extreme) as its reference, and on the ground
  // the reference ping-pongs between +-3 sigma extremes, delaying LANDED
  // indefinitely: found by the SIL Monte Carlo (REQ-010). 0.5 s of lag still
  // lets a 1 m/s descent leave the band within a few seconds.
  double landing_filter_tau_s = 0.5;
};

// Override one numeric parameter by name (e.g. "kf.jerk_psd"). Returns false
// for an unknown key or an out-of-range value.
bool set_param(FcConfig& c, std::string_view key, double value);

}  // namespace fc
