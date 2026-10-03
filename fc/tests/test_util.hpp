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

inline fc::SensorFrame frame(double t, double alt, double accel) {
  return fc::SensorFrame{t, alt, accel};
}

}  // namespace fc_test
