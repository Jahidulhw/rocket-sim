#include "fc/state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fc {

bool Persistence::update(bool condition, double t, double max_gap_s) {
  if (last_t_ && t - *last_t_ > max_gap_s) count_ = 0;  // gap: the run is broken
  last_t_ = t;
  if (!condition) {
    count_ = 0;
    return false;
  }
  if (count_ == 0) first_t_ = t;
  if (count_ < required_) ++count_;
  return count_ >= required_;
}

bool StaleDetector::update(double value, bool exempt) {
  if (required_ <= 0) return false;
  if (exempt) {
    run_ = 0;
    return false;
  }
  run_ = (run_ > 0 && value == last_) ? run_ + 1 : 1;  // exact comparison is the point
  last_ = value;
  return run_ == required_;
}

StateMachine::StateMachine(FcConfig cfg)
    : cfg_(cfg),
      kf_(cfg.kf),
      baro_stale_(cfg.stale_baro_samples),
      accel_stale_(cfg.stale_accel_samples),
      launch_accel_(cfg.launch_accel_samples),
      launch_baro_(cfg.launch_baro_samples),
      burnout_(cfg.burnout_samples),
      apogee_baro_(cfg.apogee_samples),
      apogee_kf_(cfg.kalman_apogee_samples) {}

std::vector<std::string> StateMachine::take_diagnostics() {
  std::vector<std::string> out;
  out.swap(diagnostics_);
  return out;
}

void StateMachine::diag(double t, const std::string& msg) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "t=%.3f ", t);
  diagnostics_.push_back(buf + msg);
}

FcOutput StateMachine::update(const SensorFrame& f) {
  const double dt = prev_t_ ? f.t - *prev_t_ : 0.0;
  prev_t_ = f.t;

  check_health(f);

  // Ground and accelerometer references: only learned on the pad, frozen
  // from launch onward. A failed sensor teaches nothing.
  if (state_ == FlightState::Pad) {
    if (!ground_init_) {
      ground_init_ = true;
      ground_start_t_ = f.t;
      ground_n_ = 0;
    }
    if (f.t - ground_start_t_ < cfg_.ground_init_s) {
      ++ground_n_;  // running means
      if (!baro_failed_) ground_m_ += (f.baro_alt_m - ground_m_) / ground_n_;
      if (!accel_failed_) accel_ref_ += (f.accel_mps2 - accel_ref_) / ground_n_;
    } else {
      const double alpha = 1.0 - std::exp(-dt / cfg_.ground_tau_s);  // dt-correct EMA
      if (!baro_failed_ && std::fabs(f.baro_alt_m - ground_m_) <= cfg_.ground_gate_m)
        ground_m_ += alpha * (f.baro_alt_m - ground_m_);
      if (!accel_failed_ && std::fabs(f.accel_mps2 - accel_ref_) <= cfg_.accel_ref_gate_mps2)
        accel_ref_ += alpha * (f.accel_mps2 - accel_ref_);
    }
  }
  const double agl = f.baro_alt_m - ground_m_;
  if (launch_t_ && !baro_failed_) max_agl_ = std::max(max_agl_, agl);

  const bool use_kf = cfg_.apogee_mode == ApogeeMode::Kalman;
  if (use_kf) run_filter(dt, f, agl);

  switch (state_) {
    case FlightState::Pad: on_pad(f, agl); break;
    case FlightState::Boost: on_boost(f, agl); break;
    case FlightState::Coast: on_coast(f, agl); break;
    case FlightState::Apogee: state_ = FlightState::Descent; [[fallthrough]];
    case FlightState::Descent: on_descent(f, agl); break;
    case FlightState::Landed: break;
  }

  FcOutput out;
  out.state = state_;
  out.est_alt_m = use_kf ? kf_.x()[0] : agl;
  out.est_vel_mps = use_kf ? kf_.x()[1] : 0.0;
  // Boost lockout, enforced at the output as well as by construction (only
  // COAST can command a deploy): no code path may fire in PAD or BOOST.
  out.deploy = deploy_ && state_ != FlightState::Pad && state_ != FlightState::Boost;
  return out;
}

void StateMachine::check_health(const SensorFrame& f) {
  if (!baro_failed_ && baro_stale_.update(f.baro_alt_m)) {
    baro_failed_ = true;
    diag(f.t, "HEALTH baro FAILED (stuck: " + std::to_string(cfg_.stale_baro_samples) +
                  " identical readings); excluded from estimation and detection");
  }
  // Saturation legitimately repeats the full-scale value; it is not staleness.
  const bool at_full_scale = std::fabs(f.accel_mps2) >= cfg_.accel_full_scale_mps2 - 1e-6;
  if (!accel_failed_ && accel_stale_.update(f.accel_mps2, at_full_scale)) {
    accel_failed_ = true;
    diag(f.t, "HEALTH accel FAILED (stuck: " + std::to_string(cfg_.stale_accel_samples) +
                  " identical readings); excluded from estimation and detection");
  }
}

void StateMachine::run_filter(double dt, const SensorFrame& f, double agl) {
  if (!kf_.initialized()) {
    kf_.init(agl);
    return;
  }
  kf_.predict(dt);
  // Gate only in COAST. Ignition, burnout and the chute snatch are REAL steps
  // in acceleration that a gate would reject (and the filter would then never
  // follow them); in COAST the true motion is smooth, so a large innovation
  // means a bad measurement.
  const double gate = state_ == FlightState::Coast ? cfg_.kf_gate_sigma : 0.0;
  if (!baro_failed_) {
    const UpdateResult r = kf_.update_baro(agl, gate);
    baro_rejected_run_ = r.accepted ? 0 : baro_rejected_run_ + 1;
    baro_rejected_total_ += r.accepted ? 0 : 1;
  }
  if (!accel_failed_) {
    const UpdateResult r = kf_.update_accel(f.accel_mps2 - accel_ref_, gate);  // kinematic, bias removed
    accel_rejected_run_ = r.accepted ? 0 : accel_rejected_run_ + 1;
    accel_rejected_total_ += r.accepted ? 0 : 1;
  }
  // A sensor that disagrees with the filter for a sustained run is not an
  // outlier: either it or the estimate is wrong, and with two sensors the FC
  // cannot always tell which. Stop trusting the apogee detector; the backup
  // timer still deploys.
  const int limit = cfg_.max_consecutive_rejections;
  if (!inconsistent_ && limit > 0 && (baro_rejected_run_ >= limit || accel_rejected_run_ >= limit)) {
    inconsistent_ = true;
    diag(f.t, std::string("HEALTH estimator INCONSISTENT (") +
                  (baro_rejected_run_ >= limit ? "baro" : "accel") +
                  " rejected " + std::to_string(limit) +
                  " frames in a row); apogee detection disabled, backup timer only");
  }
}

void StateMachine::on_pad(const SensorFrame& f, double agl) {
  const double gap = cfg_.max_frame_gap_s;
  const bool by_accel =
      launch_accel_.update(!accel_failed_ && f.accel_mps2 > cfg_.launch_accel_mps2, f.t, gap);
  const bool by_baro = launch_baro_.update(!baro_failed_ && agl > cfg_.launch_baro_agl_m, f.t, gap);
  if (!by_accel && !by_baro) return;
  // Launch time = start of the confirming run, not the confirmation sample.
  launch_t_ = by_accel ? launch_accel_.first_t() : launch_baro_.first_t();
  max_agl_ = agl;
  state_ = FlightState::Boost;
}

void StateMachine::on_boost(const SensorFrame& f, double agl) {
  const bool burnout = burnout_.update(!accel_failed_ && f.accel_mps2 < cfg_.burnout_accel_mps2, f.t,
                                       cfg_.max_frame_gap_s);
  // Stuck-accelerometer fallback; the altitude condition means a false
  // launch on the pad can never get out of BOOST into a deployable state.
  const bool fallback =
      !baro_failed_ && f.t - *launch_t_ > cfg_.max_boost_s && agl > cfg_.burnout_fallback_agl_m;
  if (burnout || fallback) state_ = FlightState::Coast;
}

void StateMachine::on_coast(const SensorFrame& f, double agl) {
  const double gap = cfg_.max_frame_gap_s;
  const double since_launch = f.t - *launch_t_;
  // Deploy inhibit: no apogee decision until well after the longest expected
  // burn, whatever the burnout detector concluded (defence in depth).
  const bool armed = since_launch >= cfg_.min_deploy_after_launch_s;

  bool apogee = false;
  if (armed && !inconsistent_) {
    if (cfg_.apogee_mode == ApogeeMode::Kalman && !accel_failed_) {
      // Works with the barometer failed too: the filter then runs on the
      // (pad-calibrated) accelerometer alone for the few seconds to apogee.
      apogee = apogee_kf_.update(kf_.x()[1] < 0.0, f.t, gap);
    } else if (!baro_failed_) {
      // Baseline mode, or Kalman mode degraded by a failed accelerometer:
      // the raw-barometer detector never needed the accelerometer.
      apogee = apogee_baro_.update(agl <= max_agl_ - cfg_.apogee_drop_m, f.t, gap);
    }
  }
  if (apogee) {
    state_ = FlightState::Apogee;
    command_deploy(DeployReason::Apogee);
  } else if (since_launch >= cfg_.backup_timer_s) {
    // Apogee never confirmed: deploy anyway. Straight to DESCENT, so the
    // reply stream shows the timer (not the detector) made the decision.
    state_ = FlightState::Descent;
    command_deploy(DeployReason::BackupTimer);
  }
}

void StateMachine::on_descent(const SensorFrame& f, double agl) {
  if (baro_failed_) return;  // no altitude, no landing detection (not safety-relevant)
  // Landed = altitude stays within +-band of a reference sample for the
  // duration. Leaving the band, or a frame gap, restarts the window.
  const bool restart = !land_ref_init_ || std::fabs(agl - land_ref_m_) > cfg_.landing_band_m ||
                       f.t - land_last_t_ > cfg_.max_frame_gap_s;
  if (restart) {
    land_ref_init_ = true;
    land_ref_m_ = agl;
    land_start_t_ = f.t;
  }
  land_last_t_ = f.t;
  if (f.t - land_start_t_ >= cfg_.landing_duration_s) state_ = FlightState::Landed;
}

void StateMachine::command_deploy(DeployReason why) {
  if (deploy_) return;
  deploy_ = true;  // latched: a lost reply cannot "un-command" the chute
  reason_ = why;
}

}  // namespace fc
