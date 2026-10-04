#pragma once

// Linear Kalman filter, state x = [h, v, a]:
//   h  altitude above ground (m)
//   v  vertical velocity (m/s, +up)
//   a  vertical KINEMATIC acceleration (m/s^2, +up; -g in free fall)
//
// Model: constant acceleration driven by continuous white-noise jerk.
//   F(dt) = [1 dt dt^2/2; 0 1 dt; 0 0 1]
//   Q(dt) = q * [dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 dt]
// Q is the exact integral of the white-jerk input over the step, so any dt
// (a skipped frame, irregular timing) gets the right amount of process noise.
//
// Measurements (two scalar updates per frame, R diagonal):
//   barometer      z = h + noise,  H = [1 0 0],  R = sigma_b^2 + quant^2/12
//   accelerometer  z = a + noise,  H = [0 0 1],  R = sigma_a^2
// Covariance update in Joseph form, then symmetrised, so P stays symmetric
// positive semidefinite despite rounding.

#include <array>
#include <cstddef>

namespace fc {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

struct KalmanConfig {
  double baro_sigma_m = 0.5;        // sensor model: baro white noise
  double baro_quant_m = 0.1;        // sensor model: baro quantization step
  double accel_sigma_mps2 = 0.5;    // sensor model: accel white noise
  double jerk_psd = 10.0;           // q, m^2/s^5: process noise (tuned by SIL sweep, see walkthrough)
  double init_vel_sigma_mps = 1.0;  // initial uncertainty (filter starts on the pad)
  double init_acc_sigma_mps2 = 1.0;
};

struct UpdateResult {
  double innovation = 0.0;  // z - H x (before the update)
  double s = 0.0;           // innovation variance H P H' + R
  bool accepted = true;     // false: rejected by the innovation gate, state untouched
};

class Kalman3 {
 public:
  explicit Kalman3(KalmanConfig cfg = {});

  void init(double h);
  bool initialized() const { return init_; }

  void predict(double dt);
  // Process noise for subsequent predictions (jerk PSD q, m^2/s^5).
  void set_jerk_psd(double q) { cfg_.jerk_psd = q; }
  double jerk_psd() const { return cfg_.jerk_psd; }
  // gate_sigma > 0 enables innovation gating: the measurement is rejected
  // (no state or covariance change) if |innovation| > gate_sigma * sqrt(S),
  // i.e. a chi-square test with 1 degree of freedom.
  UpdateResult update_baro(double h_meas, double gate_sigma = 0.0);
  UpdateResult update_accel(double a_meas, double gate_sigma = 0.0);

  const Vec3& x() const { return x_; }
  const Mat3& P() const { return P_; }
  double r_baro() const { return r_baro_; }
  double r_accel() const { return r_accel_; }

  static Mat3 transition(double dt);
  static Mat3 process_noise(double dt, double q);

 private:
  UpdateResult update_scalar(std::size_t idx, double z, double r, double gate_sigma);

  KalmanConfig cfg_;
  double r_baro_;
  double r_accel_;
  bool init_ = false;
  Vec3 x_{};
  Mat3 P_{};
};

}  // namespace fc
