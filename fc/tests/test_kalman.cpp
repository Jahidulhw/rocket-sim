#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "fc/kalman.hpp"
#include "test_util.hpp"

using fc::Kalman3;
using fc::KalmanConfig;
using fc::Mat3;
using fc_test::kG;
using fc_test::Noise;

namespace {

double quantize(double v, double q) { return q * std::round(v / q); }

// Positive semidefinite check for a symmetric 3x3 matrix via its leading
// principal minors after a tiny diagonal shift (Sylvester's criterion is
// for strict definiteness; the shift turns "semi" into "strict").
::testing::AssertionResult IsSymmetricPsd(const Mat3& P) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (P[i][j] != P[j][i]) return ::testing::AssertionFailure() << "asymmetric at " << i << j;
  double scale = 0.0;
  for (int i = 0; i < 3; ++i) scale = std::max(scale, std::fabs(P[i][i]));
  const double eps = 1e-12 * std::max(scale, 1e-30);
  Mat3 A = P;
  for (int i = 0; i < 3; ++i) A[i][i] += eps;
  const double m1 = A[0][0];
  const double m2 = A[0][0] * A[1][1] - A[0][1] * A[1][0];
  const double m3 = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                    A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                    A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (m1 > 0.0 && m2 > 0.0 && m3 > 0.0) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "minors " << m1 << " " << m2 << " " << m3;
}

struct Truth {
  double h0, v0, a;
  double h(double t) const { return h0 + v0 * t + 0.5 * a * t * t; }
  double v(double t) const { return v0 + a * t; }
};

// Run the filter over [0, t_end] at fixed dt on a constant-acceleration truth.
template <class Fn>
void run(Kalman3& kf, const Truth& tr, double dt, double t_end, double baro_sigma, double acc_sigma,
         unsigned seed, Fn&& each) {
  Noise n(seed);
  kf.init(quantize(tr.h(0.0) + baro_sigma * n.gauss(), 0.1));
  const int steps = static_cast<int>(std::lround(t_end / dt));
  for (int k = 1; k <= steps; ++k) {
    const double t = k * dt;
    kf.predict(dt);
    kf.update_baro(quantize(tr.h(t) + baro_sigma * n.gauss(), 0.1));
    kf.update_accel(tr.a + acc_sigma * n.gauss());
    each(t);
  }
}

}  // namespace

TEST(Kalman, NoiseCovariancesComeFromSensorModel) {
  KalmanConfig c;
  c.baro_sigma_m = 0.5;
  c.baro_quant_m = 0.1;
  c.accel_sigma_mps2 = 0.5;
  Kalman3 kf(c);
  EXPECT_DOUBLE_EQ(kf.r_baro(), 0.25 + 0.01 / 12.0);
  EXPECT_DOUBLE_EQ(kf.r_accel(), 0.25);
}

TEST(Kalman, ProcessNoiseIsExactWhiteJerkIntegral) {
  const double q = 3.0, dt = 0.02;
  const Mat3 Q = Kalman3::process_noise(dt, q);
  EXPECT_DOUBLE_EQ(Q[0][0], q * std::pow(dt, 5) / 20.0);
  EXPECT_DOUBLE_EQ(Q[1][1], q * std::pow(dt, 3) / 3.0);
  EXPECT_DOUBLE_EQ(Q[2][2], q * dt);
  EXPECT_DOUBLE_EQ(Q[0][2], Q[2][0]);
  EXPECT_TRUE(IsSymmetricPsd(Q));
  // Consistency across step sizes: two steps of dt must accumulate exactly
  // the noise of one step of 2 dt, F Q(dt) F' + Q(dt) == Q(2 dt). This is what
  // makes skipped frames and irregular dt safe.
  const Mat3 F = Kalman3::transition(dt);
  Mat3 two{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k)
        for (int l = 0; l < 3; ++l) s += F[i][k] * Q[k][l] * F[j][l];
      two[i][j] = s + Q[i][j];
    }
  const Mat3 Q2 = Kalman3::process_noise(2 * dt, q);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) EXPECT_NEAR(two[i][j], Q2[i][j], 1e-15 + 1e-12 * std::fabs(Q2[i][j]));
}

TEST(Kalman, ConvergesOnConstantVelocity) {
  Kalman3 kf;
  const Truth tr{10.0, 20.0, 0.0};
  std::vector<double> verr;
  run(kf, tr, 0.01, 10.0, 0.5, 0.5, 1, [&](double t) {
    if (t > 5.0) verr.push_back(kf.x()[1] - tr.v(t));
  });
  double rms = 0.0;
  for (double e : verr) rms += e * e;
  rms = std::sqrt(rms / static_cast<double>(verr.size()));
  // Starts believing v = 0 (sigma 1 m/s) while the truth is 20 m/s: must
  // pull in and settle.
  EXPECT_LT(rms, 0.3);
  EXPECT_NEAR(kf.x()[0], tr.h(10.0), 0.5);
  EXPECT_NEAR(kf.x()[2], 0.0, 0.5);
}

TEST(Kalman, NoiselessParabolaIsTrackedExactly) {
  // The constant-acceleration model is exact for a parabola, so with perfect
  // measurements the error must converge to (numerically) zero.
  KalmanConfig c;
  c.baro_quant_m = 0.0;
  Kalman3 kf(c);
  // init() starts at v = 0 while the truth climbs at 60 m/s; that transient
  // decays exponentially, so run 60 s and require it to have vanished.
  const Truth tr{100.0, 60.0, -kG};
  kf.init(tr.h(0.0));
  const double dt = 0.01;
  for (int k = 1; k <= 6000; ++k) {
    const double t = k * dt;
    kf.predict(dt);
    kf.update_baro(tr.h(t));
    kf.update_accel(tr.a);
  }
  const double t_end = 60.0;
  EXPECT_NEAR(kf.x()[0], tr.h(t_end), 1e-6 * std::fabs(tr.h(t_end)));
  EXPECT_NEAR(kf.x()[1], tr.v(t_end), 1e-6);
  EXPECT_NEAR(kf.x()[2], tr.a, 1e-9);
}

TEST(Kalman, AccurateVelocityOnNoisyParabola) {
  // Coast-like ballistic arc with sensor-model noise: velocity accuracy is
  // what apogee detection depends on.
  const Truth tr{100.0, 60.0, -kG};
  const double t_apogee = 60.0 / kG;
  for (unsigned seed = 1; seed <= 10; ++seed) {
    Kalman3 kf;
    double sq = 0.0, worst = 0.0, t_cross = std::nan("");
    int n = 0;
    double v_prev = 0.0;
    bool have_prev = false;
    run(kf, tr, 0.01, 9.0, 0.5, 0.5, seed, [&](double t) {
      const double v = kf.x()[1];
      if (t > 2.0) {
        const double e = v - tr.v(t);
        sq += e * e;
        worst = std::max(worst, std::fabs(e));
        ++n;
        if (have_prev && v_prev > 0.0 && v <= 0.0 && std::isnan(t_cross)) t_cross = t;
      }
      v_prev = v;
      have_prev = true;
    });
    EXPECT_LT(std::sqrt(sq / n), 0.15) << "seed " << seed;
    EXPECT_LT(worst, 0.5) << "seed " << seed;
    EXPECT_NEAR(t_cross, t_apogee, 0.05) << "seed " << seed;  // velocity zero crossing = apogee
  }
}

TEST(Kalman, CovarianceStaysPositiveSemidefinite) {
  Kalman3 kf;
  kf.init(0.0);
  Noise n(5);
  double h = 0.0, v = 0.0;
  for (int k = 0; k < 20000; ++k) {
    // Wildly varying dt, from 0.1 ms to 2 s, including long gaps.
    const double u = std::fabs(n.gauss());
    const double dt = (k % 97 == 0) ? 2.0 : 1e-4 + 0.03 * u;
    v += -kG * dt;
    h += v * dt;
    kf.predict(dt);
    ASSERT_TRUE(IsSymmetricPsd(kf.P())) << "after predict, step " << k;
    if (k % 3 != 0) kf.update_baro(h + 0.5 * n.gauss());
    kf.update_accel(-kG + 0.5 * n.gauss());
    ASSERT_TRUE(IsSymmetricPsd(kf.P())) << "after update, step " << k;
    for (int i = 0; i < 3; ++i) ASSERT_GT(kf.P()[i][i], 0.0);
    if (k % 500 == 0) { h = 0.0; v = 0.0; kf.init(0.0); }
  }
}

TEST(Kalman, HandlesSkippedFrames) {
  const Truth tr{50.0, 40.0, -kG};
  Kalman3 kf;
  Noise n(8);
  kf.init(tr.h(0.0));
  const double dt = 0.01;
  int k = 1;
  for (; k <= 300; ++k) {  // 3 s of normal frames
    kf.predict(dt);
    kf.update_baro(tr.h(k * dt) + 0.5 * n.gauss());
    kf.update_accel(tr.a + 0.5 * n.gauss());
  }
  const double p_hh_before = kf.P()[0][0];
  // 0.5 s with no frames, then one predict across the gap.
  const int k_gap = k + 50;
  kf.predict((k_gap - (k - 1)) * dt);
  const double t = k_gap * dt;
  EXPECT_GT(kf.P()[0][0], p_hh_before);              // uncertainty grew over the gap
  EXPECT_NEAR(kf.x()[0], tr.h(t), 1.0);               // prediction across the gap is sound
  EXPECT_NEAR(kf.x()[1], tr.v(t), 0.5);
  for (k = k_gap; k <= k_gap + 200; ++k) {
    if (k > k_gap) kf.predict(dt);
    kf.update_baro(tr.h(k * dt) + 0.5 * n.gauss());
    kf.update_accel(tr.a + 0.5 * n.gauss());
  }
  EXPECT_NEAR(kf.x()[1], tr.v((k - 1) * dt), 0.3);
}

TEST(Kalman, IrregularDtGivesSameAnswerOnExactModel) {
  // Noiseless parabola sampled at 100 Hz vs an irregular grid: the dt-aware
  // filter must agree at the end.
  KalmanConfig c;
  c.baro_quant_m = 0.0;
  const Truth tr{0.0, 80.0, -kG};
  Kalman3 a(c), b(c);
  a.init(0.0);
  b.init(0.0);
  // 60 s: long enough for both filters' start-up transients (v = 0 at init)
  // to vanish, leaving only the exact steady state, which must not depend on dt.
  for (int k = 1; k <= 6000; ++k) {
    a.predict(0.01);
    a.update_baro(tr.h(k * 0.01));
    a.update_accel(tr.a);
  }
  Noise n(3);
  double t = 0.0;
  while (t < 60.0 - 1e-9) {
    const double dt = std::min(0.003 + 0.04 * std::fabs(n.gauss()), 60.0 - t);
    t += dt;
    b.predict(dt);
    b.update_baro(tr.h(t));
    b.update_accel(tr.a);
  }
  EXPECT_NEAR(a.x()[1], b.x()[1], 1e-4);
  EXPECT_NEAR(a.x()[0], b.x()[0], 1e-4);
}

TEST(Kalman, DeterministicForIdenticalInput_REQ006) {
  const Truth tr{0.0, 50.0, -kG};
  Kalman3 a, b;
  std::vector<double> xa, xb;
  run(a, tr, 0.01, 5.0, 0.5, 0.5, 9, [&](double) { xa.push_back(a.x()[1]); });
  run(b, tr, 0.01, 5.0, 0.5, 0.5, 9, [&](double) { xb.push_back(b.x()[1]); });
  EXPECT_EQ(xa, xb);  // bitwise
}
