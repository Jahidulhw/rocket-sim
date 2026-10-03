#pragma once

// Synthetic flights for state-machine and fault tests.

#include <cmath>
#include <vector>

#include "fc/state_machine.hpp"
#include "test_util.hpp"

namespace fc_test {

// Idealised synthetic flight (no drag subtleties, closed form):
//   t < 0          on the pad
//   0 .. 1.5 s     boost, a = 40 m/s^2         (specific force 49.81)
//   1.5 s ..       coast, a = -(g + 2) m/s^2   (specific force -2: "drag")
// Apogee at t = 1.5 + 60 / 11.81 = 6.5804 s, altitude 197.4 m.
struct Profile {
  double boost_a = 40.0, burn_t = 1.5, coast_a = -(kG + 2.0);
  double v_bo() const { return boost_a * burn_t; }
  double z_bo() const { return 0.5 * boost_a * burn_t * burn_t; }
  double apogee_t() const { return burn_t + v_bo() / -coast_a; }
  double apogee_z() const { return z_bo() + v_bo() * v_bo() / (2.0 * -coast_a); }
  double alt(double t) const {
    if (t <= 0.0) return 0.0;
    if (t <= burn_t) return 0.5 * boost_a * t * t;
    const double tc = t - burn_t;
    return z_bo() + v_bo() * tc + 0.5 * coast_a * tc * tc;
  }
  double accel(double t) const {  // specific force
    if (t <= 0.0) return kG;
    return (t <= burn_t ? boost_a : coast_a) + kG;
  }
};

struct Sensors {
  double baro_sigma = 0.0, baro_bias = 0.0, accel_sigma = 0.0, accel_bias = 0.0;
  static Sensors nominal() { return {0.5, 0.5, 0.5, 0.2}; }
};

struct Trace {
  std::vector<double> t;
  std::vector<fc::FcOutput> out;
  // Time of the first output in `s`, or NaN.
  double first(fc::FlightState s) const {
    for (std::size_t i = 0; i < out.size(); ++i)
      if (out[i].state == s) return t[i];
    return std::nan("");
  }
  double first_deploy() const {
    for (std::size_t i = 0; i < out.size(); ++i)
      if (out[i].deploy) return t[i];
    return std::nan("");
  }
};

inline double quantize(double v, double q) { return q * std::round(v / q); }

// Drive the state machine over [t0, t1) at fixed dt with a profile + noise.
// No-op corruption (the default): sensors deliver what the model produced.
struct NoFault {
  void operator()(double, double&, double&) const {}
};

// Drive the state machine over k in [k0, k1) at fixed dt with a truth
// profile + sensor noise. `corrupt(t, baro, accel)` runs AFTER noise and
// quantization, the way a real fault corrupts a real (noisy) reading.
template <class AltFn, class AccFn, class Corrupt = NoFault>
void drive(fc::StateMachine& sm, Trace& tr, int k0, int k1, double dt, AltFn alt, AccFn acc,
           Sensors s = {}, unsigned seed = 1, Corrupt corrupt = {}) {
  Noise n(seed);
  for (int k = k0; k < k1; ++k) {
    const double t = k * dt;
    const double nb = n.gauss(), na = n.gauss();
    double baro = quantize(alt(t) + s.baro_bias + s.baro_sigma * nb, 0.1);
    double accel = acc(t) + s.accel_bias + s.accel_sigma * na;
    corrupt(t, baro, accel);
    tr.t.push_back(t);
    tr.out.push_back(sm.update(fc::SensorFrame{t, baro, accel}));
  }
}

}  // namespace fc_test
