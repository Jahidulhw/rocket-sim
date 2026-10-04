#include "fc/kalman.hpp"

#include <cstddef>

namespace fc {
namespace {

Mat3 mul(const Mat3& A, const Mat3& B) {
  Mat3 C{};
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j)
      for (std::size_t k = 0; k < 3; ++k) C[i][j] += A[i][k] * B[k][j];
  return C;
}

Mat3 transpose(const Mat3& A) {
  Mat3 T{};
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) T[i][j] = A[j][i];
  return T;
}

void symmetrise(Mat3& P) {
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = i + 1; j < 3; ++j) P[i][j] = P[j][i] = 0.5 * (P[i][j] + P[j][i]);
}

}  // namespace

Kalman3::Kalman3(KalmanConfig cfg)
    : cfg_(cfg),
      // Quantization to step q adds uniform error of variance q^2/12,
      // independent of the Gaussian noise, so the variances add.
      r_baro_(cfg.baro_sigma_m * cfg.baro_sigma_m + cfg.baro_quant_m * cfg.baro_quant_m / 12.0),
      r_accel_(cfg.accel_sigma_mps2 * cfg.accel_sigma_mps2) {}

void Kalman3::init(double h) {
  x_ = {h, 0.0, 0.0};
  P_ = Mat3{};
  P_[0][0] = r_baro_;
  P_[1][1] = cfg_.init_vel_sigma_mps * cfg_.init_vel_sigma_mps;
  P_[2][2] = cfg_.init_acc_sigma_mps2 * cfg_.init_acc_sigma_mps2;
  init_ = true;
}

Mat3 Kalman3::transition(double dt) {
  return Mat3{{{1.0, dt, 0.5 * dt * dt}, {0.0, 1.0, dt}, {0.0, 0.0, 1.0}}};
}

Mat3 Kalman3::process_noise(double dt, double q) {
  const double d2 = dt * dt, d3 = d2 * dt, d4 = d3 * dt, d5 = d4 * dt;
  return Mat3{{{q * d5 / 20.0, q * d4 / 8.0, q * d3 / 6.0},
               {q * d4 / 8.0, q * d3 / 3.0, q * d2 / 2.0},
               {q * d3 / 6.0, q * d2 / 2.0, q * dt}}};
}

void Kalman3::predict(double dt) {
  if (dt <= 0.0) return;  // the Runner rejects non-increasing stamps; belt and braces
  const Mat3 F = transition(dt);
  const Vec3 x = x_;
  x_[0] = x[0] + dt * x[1] + 0.5 * dt * dt * x[2];
  x_[1] = x[1] + dt * x[2];
  x_[2] = x[2];
  P_ = mul(mul(F, P_), transpose(F));
  const Mat3 Q = process_noise(dt, cfg_.jerk_psd);
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) P_[i][j] += Q[i][j];
  symmetrise(P_);
}

UpdateResult Kalman3::update_baro(double h_meas, double gate_sigma) {
  return update_scalar(0, h_meas, r_baro_, gate_sigma);
}
UpdateResult Kalman3::update_accel(double a_meas, double gate_sigma) {
  return update_scalar(2, a_meas, r_accel_, gate_sigma);
}

// Scalar update for H = e_idx (a unit row vector). With diagonal R, two
// sequential scalar updates equal one joint update, and S is a scalar, so
// there is no matrix inverse anywhere in the filter.
UpdateResult Kalman3::update_scalar(std::size_t idx, double z, double r, double gate_sigma) {
  UpdateResult u;
  u.innovation = z - x_[idx];
  u.s = P_[idx][idx] + r;
  // Innovation gate: S is what the filter EXPECTS the innovation variance to
  // be. A measurement many sigma away is far more likely a sensor fault than
  // a real event, so it is not allowed to move the state.
  if (gate_sigma > 0.0 && u.innovation * u.innovation > gate_sigma * gate_sigma * u.s) {
    u.accepted = false;
    return u;
  }
  Vec3 K{};
  for (std::size_t i = 0; i < 3; ++i) K[i] = P_[i][idx] / u.s;
  for (std::size_t i = 0; i < 3; ++i) x_[i] += K[i] * u.innovation;

  // Joseph form: P = (I - K H) P (I - K H)' + K R K'. Algebraically equal
  // to (I - K H) P, but it stays symmetric PSD under rounding error, where
  // the short form can lose definiteness and make the filter diverge.
  Mat3 A{};
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) A[i][j] = (i == j ? 1.0 : 0.0);
    A[i][idx] -= K[i];
  }
  Mat3 P = mul(mul(A, P_), transpose(A));
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) P[i][j] += K[i] * r * K[j];
  P_ = P;
  symmetrise(P_);
  return u;
}

}  // namespace fc
