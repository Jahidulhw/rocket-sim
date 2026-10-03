#include "fc/config.hpp"

#include <cmath>

namespace fc {

// Named overrides for tuning studies (flight_computer --param key=value).
// Only numeric tuning parameters are exposed; a typo is an error, never
// silently ignored.
bool set_param(FcConfig& c, std::string_view key, double v) {
  auto as_count = [&](int& dst) {
    if (v < 1.0 || v != std::floor(v) || v > 100000.0) return false;
    dst = static_cast<int>(v);
    return true;
  };
  auto positive = [&](double& dst) {
    if (!(v > 0.0)) return false;
    dst = v;
    return true;
  };
  auto non_negative = [&](double& dst) {
    if (!(v >= 0.0)) return false;
    dst = v;
    return true;
  };
  if (key == "kf.jerk_psd") return positive(c.kf.jerk_psd);
  if (key == "kf.baro_sigma_m") return positive(c.kf.baro_sigma_m);
  if (key == "kf.baro_quant_m") return non_negative(c.kf.baro_quant_m);
  if (key == "kf.accel_sigma_mps2") return positive(c.kf.accel_sigma_mps2);
  if (key == "kalman_apogee_samples") return as_count(c.kalman_apogee_samples);
  if (key == "apogee_drop_m") return non_negative(c.apogee_drop_m);
  if (key == "apogee_samples") return as_count(c.apogee_samples);
  if (key == "backup_timer_s") return positive(c.backup_timer_s);
  return false;
}

}  // namespace fc
