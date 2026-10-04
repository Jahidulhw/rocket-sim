// Fault detection and tolerance in the flight computer (milestone 4).
// Test names carry the requirement IDs they verify (docs/requirements.md).

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <string>

#include "fc/kalman.hpp"
#include "fc/state_machine.hpp"
#include "flight_profile.hpp"
#include "test_util.hpp"

using fc::DeployReason;
using fc::FcConfig;
using fc::FlightState;
using fc::Kalman3;
using fc::StateMachine;
using fc_test::drive;
using fc_test::kG;
using fc_test::Noise;
using fc_test::Profile;
using fc_test::Sensors;
using fc_test::Trace;

namespace {

constexpr double kDt = 0.01;
constexpr int kPadStart = -500;  // 5 s pad sit
constexpr int kEnd = 1000;       // to t = 10 s (apogee of the profile ~6.58 s)

FcConfig baseline_cfg() {
  FcConfig c;
  c.apogee_mode = fc::ApogeeMode::Baseline;
  return c;
}

// Holds the first value seen inside [t0, t1) and returns it unchanged
// thereafter: a frozen register, the realistic "stuck sensor".
struct Freeze {
  explicit Freeze(double start, double end = 1e9) : t0(start), t1(end) {}
  double t0, t1;
  std::optional<double> held;
  double apply(double t, double v) {
    if (t < t0 || t >= t1) return v;
    if (!held) held = v;
    return *held;
  }
};

}  // namespace

// ------------------------------------------------------------- gating --

TEST(Kalman, GateRejectsOutlierWithoutTouchingState_REQ004) {
  Kalman3 kf;
  kf.init(0.0);
  Noise n(1);
  for (int k = 1; k <= 300; ++k) {
    kf.predict(kDt);
    kf.update_baro(0.5 * n.gauss(), 5.0);
    kf.update_accel(0.5 * n.gauss(), 5.0);
  }
  const auto x = kf.x();
  const auto P = kf.P();
  const auto r = kf.update_baro(50.0, 5.0);  // 50 m spike
  EXPECT_FALSE(r.accepted);
  EXPECT_GT(r.innovation * r.innovation, 25.0 * r.s);
  EXPECT_EQ(kf.x(), x);  // bitwise unchanged
  EXPECT_EQ(kf.P(), P);
  EXPECT_TRUE(kf.update_baro(0.3, 5.0).accepted);
}

TEST(Kalman, GateDisabledAcceptsEverything) {
  Kalman3 kf;
  kf.init(0.0);
  EXPECT_TRUE(kf.update_baro(1e4, 0.0).accepted);
}

// ------------------------------------------------------------- spikes --

TEST(Faults, BaroSpikesInCoastAreRejected_REQ004) {
  Profile p;
  StateMachine sm;
  Trace tr;
  // Every 7th barometer sample from 2 s on is +-60 m off (alternating sign).
  auto corrupt = [](double t, double& baro, double&) {
    const long k = std::lround(t / kDt);
    if (t >= 2.0 && k % 7 == 0) baro += (k % 14 == 0) ? 60.0 : -60.0;
  };
  drive(sm, tr, kPadStart, kEnd, kDt, [&](double t) { return p.alt(t); },
        [&](double t) { return p.accel(t); }, Sensors::nominal(), 3, corrupt);
  const double err = tr.first(FlightState::Apogee) - p.apogee_t();
  EXPECT_GT(err, -0.05);
  EXPECT_LT(err, 0.15);
  EXPECT_GT(sm.baro_rejections_total(), 50);  // the spikes were seen and refused
  EXPECT_FALSE(sm.estimator_inconsistent());
  EXPECT_FALSE(sm.baro_failed());
}

TEST(Faults, BaselineIsFooledByBaroSpikes) {
  // Documents WHY gating matters: the raw-barometer detector has no defence,
  // a single +60 m spike inflates its running maximum and it fires early.
  Profile p;
  StateMachine sm(baseline_cfg());
  Trace tr;
  auto corrupt = [](double t, double& baro, double&) {
    if (std::fabs(t - 3.0) < 1e-9) baro += 60.0;
  };
  drive(sm, tr, kPadStart, kEnd, kDt, [&](double t) { return p.alt(t); },
        [&](double t) { return p.accel(t); }, Sensors::nominal(), 3, corrupt);
  EXPECT_LT(tr.first(FlightState::Apogee), p.apogee_t() - 2.0);
}

TEST(Faults, AccelSpikesInCoastAreRejected_REQ004) {
  Profile p;
  StateMachine sm;
  Trace tr;
  auto corrupt = [](double t, double&, double& accel) {
    const long k = std::lround(t / kDt);
    if (t >= 2.0 && k % 5 == 0) accel += (k % 10 == 0) ? 80.0 : -80.0;
  };
  drive(sm, tr, kPadStart, kEnd, kDt, [&](double t) { return p.alt(t); },
        [&](double t) { return p.accel(t); }, Sensors::nominal(), 4, corrupt);
  const double err = tr.first(FlightState::Apogee) - p.apogee_t();
  EXPECT_GT(err, -0.05);
  EXPECT_LT(err, 0.15);
  EXPECT_GT(sm.accel_rejections_total(), 50);
  EXPECT_FALSE(sm.estimator_inconsistent());
}

// ------------------------------------------------------------ stuck --

TEST(Faults, StuckBaroInCoastIsIsolatedAndFilterRunsOnAccel_REQ004) {
  Profile p;
  StateMachine sm;
  Trace tr;
  Freeze fz{3.0};
  auto corrupt = [&](double t, double& baro, double&) { baro = fz.apply(t, baro); };
  drive(sm, tr, kPadStart, kEnd, kDt, [&](double t) { return p.alt(t); },
        [&](double t) { return p.accel(t); }, Sensors::nominal(), 5, corrupt);
  EXPECT_TRUE(sm.baro_failed());
  EXPECT_FALSE(sm.accel_failed());
  const double err = tr.first(FlightState::Apogee) - p.apogee_t();
  // 3.6 s of accelerometer-only dead reckoning (bias calibrated on the pad).
  EXPECT_GT(err, -0.2);
  EXPECT_LT(err, 0.3);
  EXPECT_EQ(sm.deploy_reason(), DeployReason::Apogee);
}

TEST(Faults, StuckAccelInBoostIsIsolatedAndBaselineTakesOver_REQ004) {
  Profile p;
  FcConfig cfg;
  StateMachine sm(cfg);
  Trace tr;
  Freeze fz{0.5};
  auto corrupt = [&](double t, double&, double& accel) { accel = fz.apply(t, accel); };
  drive(sm, tr, kPadStart, kEnd, kDt, [&](double t) { return p.alt(t); },
        [&](double t) { return p.accel(t); }, Sensors::nominal(), 6, corrupt);
  EXPECT_TRUE(sm.accel_failed());
  // Burnout never seen by the dead accelerometer: altitude-confirmed fallback.
  EXPECT_NEAR(tr.first(FlightState::Coast), *sm.launch_time() + cfg.max_boost_s, kDt + 1e-9);
  // Apogee from the raw-barometer detector instead of the filter.
  const double err = tr.first(FlightState::Apogee) - p.apogee_t();
  EXPECT_GT(err, 0.0);
  EXPECT_LT(err, 1.2);
  EXPECT_EQ(sm.deploy_reason(), DeployReason::Apogee);
}

TEST(Faults, StuckAccelOnPadStillLaunchesByBaro_REQ004) {
  Profile p;
  StateMachine sm;
  Trace tr;
  Freeze fz{-3.0};
  auto corrupt = [&](double t, double&, double& accel) { accel = fz.apply(t, accel); };
  drive(sm, tr, kPadStart, kEnd, kDt, [&](double t) { return p.alt(t); },
        [&](double t) { return p.accel(t); }, Sensors::nominal(), 7, corrupt);
  EXPECT_TRUE(sm.accel_failed());
  EXPECT_GT(tr.first(FlightState::Boost), 0.8);  // baro path: ~15 m AGL
  EXPECT_LT(tr.first(FlightState::Boost), 1.2);
  EXPECT_FALSE(std::isnan(tr.first_deploy()));
}

TEST(Faults, SaturatedAccelIsNotMistakenForStuck) {
  FcConfig cfg;
  StateMachine sm(cfg);
  double t = 0.0;
  Noise n(2);
  for (int i = 0; i < 300; ++i) sm.update({t += kDt, 0.5 * n.gauss(), kG + 0.5 * n.gauss()});
  for (int i = 0; i < 50; ++i) sm.update({t += kDt, 0.5 * n.gauss(), cfg.accel_full_scale_mps2});
  EXPECT_FALSE(sm.accel_failed());
}

TEST(Faults, LiveNoisySensorsAreNeverDeclaredFailed) {
  Profile p;
  for (unsigned seed = 1; seed <= 30; ++seed) {
    StateMachine sm;
    Trace tr;
    drive(sm, tr, -3000, 1200, kDt, [&](double t) { return p.alt(t); },
          [&](double t) { return p.accel(t); }, Sensors::nominal(), seed);
    ASSERT_FALSE(sm.baro_failed()) << seed;
    ASSERT_FALSE(sm.accel_failed()) << seed;
    ASSERT_FALSE(sm.estimator_inconsistent()) << seed;
    ASSERT_EQ(sm.deploy_reason(), DeployReason::Apogee) << seed;
  }
}

// ---------------------------------------------- unisolated disagreement --

TEST(Faults, PersistentDisagreementFallsBackToBackupTimer_REQ004) {
  // A +40 m barometer offset that stays noisy (so it is NOT stale): the FC
  // cannot tell which sensor is wrong, so it must not trust its apogee
  // detector, and the backup timer deploys.
  Profile p;
  FcConfig cfg;
  StateMachine sm(cfg);
  Trace tr;
  auto corrupt = [](double t, double& baro, double&) {
    if (t >= 3.0) baro += 40.0;
  };
  drive(sm, tr, kPadStart, 1200, kDt, [&](double t) { return p.alt(t); },
        [&](double t) { return p.accel(t); }, Sensors::nominal(), 8, corrupt);
  EXPECT_TRUE(sm.estimator_inconsistent());
  EXPECT_TRUE(std::isnan(tr.first(FlightState::Apogee)));
  EXPECT_EQ(sm.deploy_reason(), DeployReason::BackupTimer);
  EXPECT_NEAR(tr.first_deploy(), *sm.launch_time() + cfg.backup_timer_s, kDt + 1e-9);
}

// --------------------------------------------------- lockout / inhibit --

TEST(Faults, DeployInhibitBlocksApogeeSoonAfterLaunch_REQ003) {
  // Hostile input: launch, then the accelerometer immediately reads "coast"
  // (false burnout at ~0.1 s) and the barometer reads a falling altitude
  // (apogee signature). Without the inhibit the FC would deploy within
  // the first second, possibly under thrust.
  FcConfig cfg = baseline_cfg();
  StateMachine sm(cfg);
  Noise n(9);
  double t = -3.0, first_deploy = std::nan("");
  while (t < 6.0) {
    t += kDt;
    const double accel = t < 0.0 ? kG : (t < 0.05 ? 60.0 : -2.0);
    const double baro = (t < 0.5 ? 20.0 * std::max(t, 0.0) : 10.0 - 10.0 * (t - 0.5)) + 0.5 * n.gauss();
    if (sm.update({t, baro, accel + 0.5 * n.gauss()}).deploy && std::isnan(first_deploy)) first_deploy = t;
  }
  ASSERT_TRUE(sm.launch_time().has_value());
  ASSERT_FALSE(std::isnan(first_deploy));
  EXPECT_GE(first_deploy, *sm.launch_time() + cfg.min_deploy_after_launch_s);
}

// ------------------------------------------------------ diagnostics --

TEST(Faults, HealthEventsAreReportedOnce) {
  Profile p;
  StateMachine sm;
  Freeze fz{3.0};
  Noise n(10);
  int baro_msgs = 0;
  for (int k = -300; k < 800; ++k) {
    const double t = k * kDt;
    const double baro = fz.apply(t, fc_test::quantize(p.alt(t) + 0.5 * n.gauss(), 0.1));
    sm.update({t, baro, p.accel(t) + 0.5 * n.gauss()});
    for (const auto& d : sm.take_diagnostics())
      if (d.find("baro FAILED") != std::string::npos) ++baro_msgs;
  }
  EXPECT_EQ(baro_msgs, 1);
}

// ------------------------------------------------- REQ-002 pad sits --

TEST(Faults, NoFalseLaunchIn1000SixtySecondPadSits_REQ002) {
  // Sensor model as in sim/sensors.py: baro sigma 0.5 m + 0.5 m bias,
  // 0.1 m quantization; accel sigma 0.5 + 0.2 bias. 1000 seeds x 6000 frames.
  int false_launches = 0, false_failures = 0;
  for (unsigned seed = 1; seed <= 1000; ++seed) {
    StateMachine sm;
    Noise n(seed * 7919u);
    for (int k = 0; k < 6000; ++k) {
      const double t = k * kDt;
      const double baro = fc_test::quantize(0.5 + 0.5 * n.gauss(), 0.1);
      const double accel = kG + 0.2 + 0.5 * n.gauss();
      const auto o = sm.update({t, baro, accel});
      if (o.state != FlightState::Pad || o.deploy) {
        ++false_launches;
        break;
      }
    }
    if (sm.baro_failed() || sm.accel_failed()) ++false_failures;
  }
  EXPECT_EQ(false_launches, 0);
  EXPECT_EQ(false_failures, 0);
}

// ------------------------------------------------------ REQ-010 landing --

TEST(Faults, LandingNotDefeatedByNoiseExtremes_REQ010) {
  // Regression test for a flaw found by the SIL Monte Carlo: when the
  // landing window restarted, it took the out-of-band sample (a noise
  // extreme) as its new reference, so on the ground the reference ping-ponged
  // between +-1.6 m extremes and LANDED could be delayed indefinitely.
  Profile p;
  const double t_ap = p.apogee_t(), z_ap = p.apogee_z();
  const double t_land = t_ap + z_ap / 3.8;
  auto alt = [&](double t) { return t <= t_ap ? p.alt(t) : std::max(0.0, z_ap - 3.8 * (t - t_ap)); };
  auto acc = [&](double t) { return t <= t_ap ? p.accel(t) : kG; };
  int late = 0;
  for (unsigned seed = 1; seed <= 60; ++seed) {
    StateMachine sm;
    Trace tr;
    drive(sm, tr, -300, static_cast<int>((t_land + 30.0) / kDt), kDt, alt, acc, Sensors::nominal(), seed);
    const double t_landed = tr.first(FlightState::Landed);
    if (std::isnan(t_landed) || t_landed - t_land > 10.0) ++late;
    else EXPECT_GT(t_landed - t_land, 3.0) << seed;   // never "landed" while still well above ground
  }
  EXPECT_EQ(late, 0);
}
