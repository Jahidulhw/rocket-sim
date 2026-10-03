#include "fc/state_machine.hpp"

#include <algorithm>
#include <cmath>

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

StateMachine::StateMachine(FcConfig cfg)
    : cfg_(cfg),
      launch_accel_(cfg.launch_accel_samples),
      launch_baro_(cfg.launch_baro_samples),
      burnout_(cfg.burnout_samples),
      apogee_(cfg.apogee_samples) {}

FcOutput StateMachine::update(const SensorFrame& f) {
  const double dt = prev_t_ ? f.t - *prev_t_ : 0.0;
  prev_t_ = f.t;

  // Ground reference: only learned on the pad, frozen from launch onward.
  if (state_ == FlightState::Pad) {
    if (!ground_init_) {
      ground_init_ = true;
      ground_start_t_ = f.t;
      ground_n_ = 0;
      ground_m_ = 0.0;
    }
    if (f.t - ground_start_t_ < cfg_.ground_init_s) {
      ++ground_n_;
      ground_m_ += (f.baro_alt_m - ground_m_) / ground_n_;  // running mean
    } else if (std::fabs(f.baro_alt_m - ground_m_) <= cfg_.ground_gate_m) {
      const double alpha = 1.0 - std::exp(-dt / cfg_.ground_tau_s);  // dt-correct EMA
      ground_m_ += alpha * (f.baro_alt_m - ground_m_);
    }
  }
  const double agl = f.baro_alt_m - ground_m_;
  if (launch_t_) max_agl_ = std::max(max_agl_, agl);

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
  out.est_alt_m = agl;
  out.est_vel_mps = 0.0;
  // Boost lockout, enforced at the output as well as by construction (only
  // COAST can command a deploy): no code path may fire in PAD or BOOST.
  out.deploy = deploy_ && state_ != FlightState::Pad && state_ != FlightState::Boost;
  return out;
}

void StateMachine::on_pad(const SensorFrame& f, double agl) {
  const double gap = cfg_.max_frame_gap_s;
  const bool by_accel = launch_accel_.update(f.accel_mps2 > cfg_.launch_accel_mps2, f.t, gap);
  const bool by_baro = launch_baro_.update(agl > cfg_.launch_baro_agl_m, f.t, gap);
  if (!by_accel && !by_baro) return;
  // Launch time = start of the confirming run, not the confirmation sample.
  launch_t_ = by_accel ? launch_accel_.first_t() : launch_baro_.first_t();
  max_agl_ = agl;
  state_ = FlightState::Boost;
}

void StateMachine::on_boost(const SensorFrame& f, double agl) {
  const bool burnout = burnout_.update(f.accel_mps2 < cfg_.burnout_accel_mps2, f.t, cfg_.max_frame_gap_s);
  // Stuck-accelerometer fallback; the altitude condition means a false
  // launch on the pad can never get out of BOOST into a deployable state.
  const bool fallback = f.t - *launch_t_ > cfg_.max_boost_s && agl > cfg_.burnout_fallback_agl_m;
  if (burnout || fallback) state_ = FlightState::Coast;
}

void StateMachine::on_coast(const SensorFrame& f, double agl) {
  const bool apogee = apogee_.update(agl <= max_agl_ - cfg_.apogee_drop_m, f.t, cfg_.max_frame_gap_s);
  if (apogee) {
    state_ = FlightState::Apogee;
    command_deploy(DeployReason::Apogee);
  } else if (f.t - *launch_t_ >= cfg_.backup_timer_s) {
    // Apogee never confirmed: deploy anyway. Straight to DESCENT, so the
    // reply stream shows the timer (not the detector) made the decision.
    state_ = FlightState::Descent;
    command_deploy(DeployReason::BackupTimer);
  }
}

void StateMachine::on_descent(const SensorFrame& f, double agl) {
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
