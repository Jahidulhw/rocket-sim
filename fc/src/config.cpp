#include "fc/config.hpp"

#include <cmath>

namespace fc {
namespace {

enum class Kind { Positive, NonNegative, Count, CountOrZero };

struct Param {
  const char* name;
  Kind kind;
  double FcConfig::*d = nullptr;     // double fields of FcConfig
  int FcConfig::*i = nullptr;        // int fields of FcConfig
  double KalmanConfig::*k = nullptr; // double fields of FcConfig::kf
};

// Every numeric tuning parameter, by name. Per-rocket configurations
// (configs/rockets/*.json "sil.fc") and tuning studies use these; a typo or
// an out-of-range value is an error, never silently ignored.
const Param kParams[] = {
    {"max_frame_gap_s", Kind::Positive, &FcConfig::max_frame_gap_s},
    {"ground_init_s", Kind::Positive, &FcConfig::ground_init_s},
    {"ground_tau_s", Kind::Positive, &FcConfig::ground_tau_s},
    {"ground_gate_m", Kind::Positive, &FcConfig::ground_gate_m},
    {"accel_ref_gate_mps2", Kind::Positive, &FcConfig::accel_ref_gate_mps2},
    {"launch_accel_mps2", Kind::Positive, &FcConfig::launch_accel_mps2},
    {"launch_accel_samples", Kind::Count, nullptr, &FcConfig::launch_accel_samples},
    {"launch_baro_agl_m", Kind::Positive, &FcConfig::launch_baro_agl_m},
    {"launch_baro_samples", Kind::Count, nullptr, &FcConfig::launch_baro_samples},
    {"burnout_accel_mps2", Kind::Positive, &FcConfig::burnout_accel_mps2},
    {"burnout_samples", Kind::Count, nullptr, &FcConfig::burnout_samples},
    {"max_boost_s", Kind::Positive, &FcConfig::max_boost_s},
    {"burnout_fallback_agl_m", Kind::Positive, &FcConfig::burnout_fallback_agl_m},
    {"apogee_drop_m", Kind::NonNegative, &FcConfig::apogee_drop_m},
    {"apogee_samples", Kind::Count, nullptr, &FcConfig::apogee_samples},
    {"kalman_apogee_samples", Kind::Count, nullptr, &FcConfig::kalman_apogee_samples},
    {"backup_timer_s", Kind::Positive, &FcConfig::backup_timer_s},
    {"stale_baro_samples", Kind::CountOrZero, nullptr, &FcConfig::stale_baro_samples},
    {"stale_accel_samples", Kind::CountOrZero, nullptr, &FcConfig::stale_accel_samples},
    {"accel_full_scale_mps2", Kind::Positive, &FcConfig::accel_full_scale_mps2},
    {"kf_gate_sigma", Kind::NonNegative, &FcConfig::kf_gate_sigma},  // 0 disables gating
    {"max_consecutive_rejections", Kind::CountOrZero, nullptr, &FcConfig::max_consecutive_rejections},
    {"min_deploy_after_launch_s", Kind::NonNegative, &FcConfig::min_deploy_after_launch_s},
    {"powered_jerk_psd", Kind::NonNegative, &FcConfig::powered_jerk_psd},  // 0: same as kf.jerk_psd
    {"tailoff_s", Kind::NonNegative, &FcConfig::tailoff_s},
    {"burns", Kind::Count, nullptr, &FcConfig::burns},
    {"next_ignition_accel_mps2", Kind::Positive, &FcConfig::next_ignition_accel_mps2},
    {"next_ignition_samples", Kind::Count, nullptr, &FcConfig::next_ignition_samples},
    {"stage_ignition_timeout_s", Kind::Positive, &FcConfig::stage_ignition_timeout_s},
    {"main_deploy_altitude_m", Kind::NonNegative, &FcConfig::main_deploy_altitude_m},
    {"main_samples", Kind::Count, nullptr, &FcConfig::main_samples},
    {"landing_band_m", Kind::Positive, &FcConfig::landing_band_m},
    {"landing_duration_s", Kind::Positive, &FcConfig::landing_duration_s},
    {"landing_filter_tau_s", Kind::Positive, &FcConfig::landing_filter_tau_s},
    {"kf.baro_sigma_m", Kind::Positive, nullptr, nullptr, &KalmanConfig::baro_sigma_m},
    {"kf.baro_quant_m", Kind::NonNegative, nullptr, nullptr, &KalmanConfig::baro_quant_m},
    {"kf.accel_sigma_mps2", Kind::Positive, nullptr, nullptr, &KalmanConfig::accel_sigma_mps2},
    {"kf.jerk_psd", Kind::Positive, nullptr, nullptr, &KalmanConfig::jerk_psd},
};

bool valid(Kind kind, double v) {
  if (!std::isfinite(v)) return false;
  switch (kind) {
    case Kind::Positive: return v > 0.0;
    case Kind::NonNegative: return v >= 0.0;
    case Kind::Count: return v >= 1.0 && v <= 100000.0 && v == std::floor(v);
    case Kind::CountOrZero: return v >= 0.0 && v <= 100000.0 && v == std::floor(v);
  }
  return false;
}

}  // namespace

bool set_param(FcConfig& c, std::string_view key, double v) {
  for (const Param& p : kParams) {
    if (key != p.name) continue;
    if (!valid(p.kind, v)) return false;
    if (p.d) c.*(p.d) = v;
    if (p.i) c.*(p.i) = static_cast<int>(v);
    if (p.k) c.kf.*(p.k) = v;
    return true;
  }
  return false;
}

}  // namespace fc
