#pragma once

// Helpers for synthetic sensor streams in unit tests.

#include <cmath>
#include <cstdint>
#include <random>

#include "fc/protocol.hpp"

namespace fc_test {

inline constexpr double kG = 9.81;

// Portable Gaussian noise. std::normal_distribution's algorithm differs
// between standard libraries (MSVC vs libstdc++), so tests using it would see
// different numbers on Windows and Linux. mt19937 itself is fully specified;
// Box-Muller on top of it is the same everywhere.
class Noise {
 public:
  explicit Noise(std::uint32_t seed) : rng_(seed) {}
  double gauss() {
    if (has_spare_) {
      has_spare_ = false;
      return spare_;
    }
    double u1 = 0.0;
    do {
      u1 = uniform();
    } while (u1 <= 0.0);
    const double u2 = uniform();
    const double r = std::sqrt(-2.0 * std::log(u1));
    spare_ = r * std::sin(2.0 * kPi * u2);
    has_spare_ = true;
    return r * std::cos(2.0 * kPi * u2);
  }

 private:
  static constexpr double kPi = 3.14159265358979323846;
  double uniform() { return static_cast<double>(rng_()) / 4294967296.0; }
  std::mt19937 rng_;
  bool has_spare_ = false;
  double spare_ = 0.0;
};

// Symmetric positive semidefinite (3x3) via leading minors after a tiny shift.
template <class M>
bool IsPsd(const M& P) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] !=
          P[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)])
        return false;
  const double e = 1e-12 * (std::fabs(P[0][0]) + std::fabs(P[1][1]) + std::fabs(P[2][2]));
  const double a = P[0][0] + e, b = P[1][1] + e, c = P[2][2] + e;
  const double m2 = a * b - P[0][1] * P[1][0];
  const double m3 = a * (b * c - P[1][2] * P[2][1]) - P[0][1] * (P[1][0] * c - P[1][2] * P[2][0]) +
                    P[0][2] * (P[1][0] * P[2][1] - b * P[2][0]);
  return a > 0 && m2 > 0 && m3 > 0;
}

inline fc::SensorFrame frame(double t, double alt, double accel) {
  return fc::SensorFrame{t, alt, accel};
}

}  // namespace fc_test
