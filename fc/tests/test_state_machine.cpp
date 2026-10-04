#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "fc/state_machine.hpp"
#include "flight_profile.hpp"
#include "test_util.hpp"

using fc::DeployReason;
using fc::FcConfig;
using fc::FcOutput;
using fc::FlightState;
using fc::StateMachine;
using fc_test::frame;
using fc_test::kG;
using fc_test::Noise;

namespace {

using fc_test::drive;
using fc_test::Profile;
using fc_test::Sensors;
using fc_test::Trace;

constexpr double kDt = 0.01;

FcConfig baseline_cfg() {
  FcConfig c;
  c.apogee_mode = fc::ApogeeMode::Baseline;
  return c;
}

// For tests driven by perfectly noiseless synthetic signals. No live sensor
// produces bit-identical readings, so the stuck-sensor detector would
// (correctly) declare these sensors failed; these tests target other logic,
// so it is switched off. Stuck-sensor handling is tested in test_faults.cpp
// with realistic noise on the healthy sensor.
FcConfig noiseless(FcConfig c = {}) {
  c.stale_baro_samples = 0;
  c.stale_accel_samples = 0;
  return c;
}

}  // namespace

// ------------------------------------------------------------------ PAD --

TEST(StateMachine, NoLaunchOnSixtySecondsOfPadNoise_REQ002) {
  for (unsigned seed = 1; seed <= 20; ++seed) {
    StateMachine sm;
    Trace tr;
    drive(sm, tr, 0, 6000, kDt, [](double) { return 0.0; }, [](double) { return kG; },
          Sensors::nominal(), seed);
    for (const auto& o : tr.out) {
      ASSERT_EQ(o.state, FlightState::Pad) << "seed " << seed;
      ASSERT_FALSE(o.deploy);
    }
  }
}

TEST(StateMachine, GroundReferenceRemovesBaroBias) {
  StateMachine sm;
  Trace tr;
  // Barometer reads 120 m on the pad (field elevation / reference error).
  drive(sm, tr, 0, 1000, kDt, [](double) { return 120.0; }, [](double) { return kG; },
        Sensors::nominal(), 3);
  EXPECT_NEAR(tr.out.back().est_alt_m, 0.0, 2.0);  // single noisy sample, sigma 0.5
  EXPECT_NEAR(sm.ground_ref_m(), 120.5, 0.15);     // bias included, noise averaged
}

TEST(StateMachine, LaunchNeedsNConsecutiveAccelSamples) {
  FcConfig cfg = noiseless();
  StateMachine sm(cfg);
  double t = 0.0;
  auto feed = [&](double accel) { t += kDt; return sm.update(frame(t, 0.0, accel)); };
  for (int i = 0; i < 100; ++i) feed(kG);
  for (int i = 0; i < cfg.launch_accel_samples - 1; ++i) feed(60.0);
  EXPECT_EQ(feed(kG).state, FlightState::Pad);  // run broken one sample short
  const double t_first = t + kDt;
  for (int i = 0; i < cfg.launch_accel_samples - 1; ++i) {
    EXPECT_EQ(feed(60.0).state, FlightState::Pad);
  }
  EXPECT_EQ(feed(60.0).state, FlightState::Boost);
  ASSERT_TRUE(sm.launch_time().has_value());
  EXPECT_DOUBLE_EQ(*sm.launch_time(), t_first);  // launch time = start of the confirming run
}

TEST(StateMachine, IsolatedAccelSpikesDoNotLaunch) {
  StateMachine sm;
  for (int k = 0; k < 3000; ++k) {
    const double accel = (k % 10 == 0) ? 200.0 : kG;
    ASSERT_EQ(sm.update(frame(k * kDt, 0.0, accel)).state, FlightState::Pad) << k;
  }
}

TEST(StateMachine, FrameGapBreaksPersistence) {
  FcConfig cfg = noiseless();
  StateMachine sm(cfg);
  double t = 0.0;
  for (int i = 0; i < 100; ++i) sm.update(frame(t += kDt, 0.0, kG));
  for (int i = 0; i < cfg.launch_accel_samples - 1; ++i) sm.update(frame(t += kDt, 0.0, 60.0));
  t += 0.2;  // 200 ms with no frames: no evidence the acceleration persisted
  EXPECT_EQ(sm.update(frame(t, 0.0, 60.0)).state, FlightState::Pad);
  for (int i = 0; i < cfg.launch_accel_samples - 2; ++i) sm.update(frame(t += kDt, 0.0, 60.0));
  EXPECT_EQ(sm.update(frame(t += kDt, 0.0, 60.0)).state, FlightState::Boost);
}

TEST(StateMachine, BaroBackupDetectsLaunchWithDeadAccelerometer) {
  Profile p;
  StateMachine sm(noiseless());
  Trace tr;
  drive(sm, tr, -200, 300, kDt, [&](double t) { return p.alt(t); }, [](double) { return kG; });
  const double t_boost = tr.first(FlightState::Boost);
  // Altitude passes 15 m at sqrt(2*15/40) = 0.866 s; 10 samples to confirm.
  EXPECT_NEAR(t_boost, 0.87 + 0.09, 0.015);
  EXPECT_NEAR(*sm.launch_time(), 0.87, 0.015);
}

// ---------------------------------------------------------------- BOOST --

TEST(StateMachine, BurnoutWhenAccelDrops) {
  Profile p;
  StateMachine sm(noiseless());
  Trace tr;
  drive(sm, tr, -200, 400, kDt, [&](double t) { return p.alt(t); }, [&](double t) { return p.accel(t); });
  // t = 0 is still a pad sample; boost samples 0.01..0.05 confirm launch.
  EXPECT_NEAR(tr.first(FlightState::Boost), 0.05, 1e-9);
  EXPECT_NEAR(*sm.launch_time(), 0.01, 1e-9);
  // First coast sample is 1.51 (or 1.50 if 150*0.01 rounds above 1.5); 5 to confirm.
  EXPECT_NEAR(tr.first(FlightState::Coast), 1.55, kDt + 1e-9);
}

TEST(StateMachine, BurnoutFallbackWithStuckAccelerometer) {
  Profile p;
  FcConfig cfg = noiseless();
  StateMachine sm(cfg);
  Trace tr;
  // Accelerometer frozen at its boost reading from t = 0.5 s onward.
  drive(sm, tr, -200, 600, kDt, [&](double t) { return p.alt(t); },
        [&](double t) { return t < 0.5 ? p.accel(t) : p.accel(0.5); });
  EXPECT_NEAR(tr.first(FlightState::Coast), *sm.launch_time() + cfg.max_boost_s, kDt + 1e-9);
}

TEST(StateMachine, FalseLaunchOnPadNeverReachesDeployableState_REQ003) {
  // Accelerometer fails high on the pad: the FC believes it launched, but the
  // barometer never confirms a climb, so it must stay in BOOST (deploy locked
  // out) for ever, including long past the backup timer.
  StateMachine sm;
  Trace tr;
  drive(sm, tr, 0, 6000, kDt, [](double) { return 0.0; },
        [](double t) { return t < 5.0 ? kG : 40.0; }, Sensors::nominal(), 9);
  EXPECT_FALSE(std::isnan(tr.first(FlightState::Boost)));
  for (const auto& o : tr.out) {
    ASSERT_NE(o.state, FlightState::Coast);
    ASSERT_FALSE(o.deploy);
  }
}

TEST(StateMachine, NoDeployInPadOrBoostEvenIfBaroFalls_REQ003) {
  // A falling baro is the apogee signature; in PAD and BOOST it must be ignored.
  StateMachine sm;
  double t = 0.0;
  for (int i = 0; i < 500; ++i) {
    const auto o = sm.update(frame(t += kDt, 50.0 - 0.2 * i, kG));
    ASSERT_EQ(o.state, FlightState::Pad);
    ASSERT_FALSE(o.deploy);
  }
  for (int i = 0; i < 1500; ++i) {  // boost-level accel, baro falling for 15 s
    const auto o = sm.update(frame(t += kDt, -50.0 - 0.5 * i, 60.0));
    ASSERT_TRUE(o.state == FlightState::Pad || o.state == FlightState::Boost);
    ASSERT_FALSE(o.deploy) << "t=" << t;
  }
}

// --------------------------------------------------------------- APOGEE --

TEST(StateMachine, BaselineApogeeOnNoiselessParabola) {
  Profile p;
  FcConfig cfg = noiseless(baseline_cfg());
  StateMachine sm(cfg);
  Trace tr;
  drive(sm, tr, -200, 800, kDt, [&](double t) { return p.alt(t); }, [&](double t) { return p.accel(t); });
  // Altitude first sits >= 1.5 m below the peak at t_ap + sqrt(2*1.5/11.81),
  // then 9 more samples confirm.
  const double t_cond = p.apogee_t() + std::sqrt(2.0 * cfg.apogee_drop_m / -p.coast_a);
  const double t_detect = tr.first(FlightState::Apogee);
  // +-3 samples: 0.1 m baro quantization moves the threshold crossing.
  EXPECT_GE(t_detect, t_cond + (cfg.apogee_samples - 1) * kDt - 3 * kDt);
  EXPECT_LE(t_detect, t_cond + cfg.apogee_samples * kDt + 3 * kDt);
  EXPECT_DOUBLE_EQ(tr.first_deploy(), t_detect);
  EXPECT_EQ(sm.deploy_reason(), DeployReason::Apogee);
  // APOGEE is a one-sample event state; the next frame is DESCENT.
  const auto it = std::find(tr.t.begin(), tr.t.end(), t_detect);
  const std::size_t i = static_cast<std::size_t>(it - tr.t.begin());
  ASSERT_LT(i + 1, tr.out.size());
  EXPECT_EQ(tr.out[i + 1].state, FlightState::Descent);
}

TEST(StateMachine, BaselineApogeeWithNoiseIsLateButBounded) {
  Profile p;
  for (unsigned seed = 1; seed <= 20; ++seed) {
    StateMachine sm(baseline_cfg());
    Trace tr;
    drive(sm, tr, -500, 900, kDt, [&](double t) { return p.alt(t); },
          [&](double t) { return p.accel(t); }, Sensors::nominal(), seed);
    const double err = tr.first(FlightState::Apogee) - p.apogee_t();
    EXPECT_GT(err, 0.0) << "seed " << seed;   // never before true apogee
    EXPECT_LT(err, 1.2) << "seed " << seed;
    EXPECT_EQ(sm.deploy_reason(), DeployReason::Apogee);
  }
}

TEST(StateMachine, KalmanApogeeOnNoisyParabola) {
  Profile p;
  FcConfig cfg;  // default mode: Kalman
  ASSERT_EQ(cfg.apogee_mode, fc::ApogeeMode::Kalman);
  for (unsigned seed = 1; seed <= 20; ++seed) {
    StateMachine sm(cfg);
    Trace tr;
    drive(sm, tr, -500, 900, kDt, [&](double t) { return p.alt(t); },
          [&](double t) { return p.accel(t); }, Sensors::nominal(), seed);
    const double err = tr.first(FlightState::Apogee) - p.apogee_t();
    // v < 0 for 5 samples: about 40 ms of confirmation lag plus filter error.
    EXPECT_GT(err, -0.05) << "seed " << seed;
    EXPECT_LT(err, 0.15) << "seed " << seed;
    EXPECT_EQ(sm.deploy_reason(), DeployReason::Apogee);
  }
}

TEST(StateMachine, KalmanBeatsBaselineOnIdenticalInput) {
  Profile p;
  for (unsigned seed = 1; seed <= 10; ++seed) {
    StateMachine kal, base(baseline_cfg());
    Trace tk, tb;
    auto alt = [&](double t) { return p.alt(t); };
    auto acc = [&](double t) { return p.accel(t); };
    drive(kal, tk, -500, 900, kDt, alt, acc, Sensors::nominal(), seed);
    drive(base, tb, -500, 900, kDt, alt, acc, Sensors::nominal(), seed);
    EXPECT_LT(std::fabs(tk.first(FlightState::Apogee) - p.apogee_t()),
              std::fabs(tb.first(FlightState::Apogee) - p.apogee_t())) << "seed " << seed;
  }
}

TEST(StateMachine, KalmanReportsVelocityAndAltitude) {
  Profile p;
  StateMachine sm;
  Trace tr;
  drive(sm, tr, -500, 650, kDt, [&](double t) { return p.alt(t); }, [&](double t) { return p.accel(t); },
        Sensors::nominal(), 6);
  // Mid-coast (3..6 s): estimates track truth closely; truth v = 60 - 11.81 (t - 1.5).
  for (std::size_t i = 0; i < tr.t.size(); ++i) {
    const double t = tr.t[i];
    if (t < 3.0 || t > 6.0) continue;
    const double v_true = p.v_bo() + p.coast_a * (t - p.burn_t);
    ASSERT_NEAR(tr.out[i].est_vel_mps, v_true, 0.5) << t;
    ASSERT_NEAR(tr.out[i].est_alt_m, p.alt(t), 0.5) << t;
  }
}

TEST(StateMachine, AccelReferenceLearnsGravityPlusBiasOnPad) {
  StateMachine sm;
  Trace tr;
  drive(sm, tr, 0, 1000, kDt, [](double) { return 0.0; }, [](double) { return kG; }, Sensors::nominal(), 2);
  EXPECT_NEAR(sm.accel_ref_mps2(), kG + 0.2, 0.05);
  EXPECT_NEAR(tr.out.back().est_vel_mps, 0.0, 0.1);
}

// --------------------------------------------------------- BACKUP TIMER --

TEST(StateMachine, BackupTimerFiresWhenApogeeNeverSeen) {
  // Baseline detector + frozen barometer: apogee is never seen, so this
  // exercises the timer path. (Kalman behaviour under a stuck barometer is a
  // milestone-4 fault case.)
  Profile p;
  FcConfig cfg = noiseless(baseline_cfg());
  StateMachine sm(cfg);
  Trace tr;
  // Barometer freezes at 3 s (stuck sensor): the baseline can never see a drop.
  drive(sm, tr, -200, 1500, kDt, [&](double t) { return p.alt(std::min(t, 3.0)); },
        [&](double t) { return p.accel(t); });
  EXPECT_TRUE(std::isnan(tr.first(FlightState::Apogee)));
  const double t_dep = tr.first_deploy();
  EXPECT_NEAR(t_dep, *sm.launch_time() + cfg.backup_timer_s, kDt + 1e-9);
  EXPECT_EQ(sm.deploy_reason(), DeployReason::BackupTimer);
  EXPECT_DOUBLE_EQ(tr.first(FlightState::Descent), t_dep);  // timer: COAST -> DESCENT directly
}

TEST(StateMachine, BackupTimerUsesTimestampsWithIrregularFrames) {
  Profile p;
  FcConfig cfg = noiseless(baseline_cfg());
  StateMachine sm(cfg);
  Noise jitter(42);
  double t = -2.0, t_dep = std::nan("");
  while (t < 15.0) {
    // dt between 5 and 45 ms: below the persistence gap limit, but nowhere
    // near a fixed 100 Hz, so sample counting would get the timer wrong.
    t += 0.005 + 0.04 * std::fabs(std::sin(jitter.gauss()));
    const auto o = sm.update(frame(t, p.alt(std::min(t, 3.0)), p.accel(t)));
    if (o.deploy && std::isnan(t_dep)) t_dep = t;
  }
  ASSERT_TRUE(sm.launch_time().has_value());
  EXPECT_GE(t_dep, *sm.launch_time() + cfg.backup_timer_s);
  EXPECT_LT(t_dep, *sm.launch_time() + cfg.backup_timer_s + 0.046);
}

TEST(StateMachine, TimeGapAcrossApogeeStillDetects) {
  // 300 ms of lost frames right at apogee: detection resumes on new data.
  for (const auto mode : {fc::ApogeeMode::Baseline, fc::ApogeeMode::Kalman}) {
  Profile p;
  FcConfig cfg = noiseless();
  cfg.apogee_mode = mode;
  StateMachine sm(cfg);
  Trace tr;
  drive(sm, tr, -200, 655, kDt, [&](double t) { return p.alt(t); }, [&](double t) { return p.accel(t); });
  drive(sm, tr, 686, 900, kDt, [&](double t) { return p.alt(t); }, [&](double t) { return p.accel(t); });
  const double err = tr.first(FlightState::Apogee) - p.apogee_t();
  EXPECT_GT(err, 0.0);   // frames resume after apogee, so detection is after it
  EXPECT_LT(err, 1.0);
  }
}

// ------------------------------------------------------ DESCENT / LANDED --

TEST(StateMachine, DeployIsLatchedThroughLanding) {
  Profile p;
  StateMachine sm;
  Trace tr;
  const double t_ap = p.apogee_t(), z_ap = p.apogee_z();
  const double t_land = t_ap + z_ap / 3.8;
  auto alt = [&](double t) {
    if (t <= t_ap) return p.alt(t);
    return std::max(0.0, z_ap - 3.8 * (t - t_ap));  // under chute
  };
  auto acc = [&](double t) { return t <= t_ap ? p.accel(t) : kG; };
  drive(sm, tr, -500, static_cast<int>((t_land + 20.0) / kDt), kDt, alt, acc, Sensors::nominal(), 4);
  const double t_dep = tr.first_deploy();
  for (std::size_t i = 0; i < tr.t.size(); ++i) {
    if (tr.t[i] >= t_dep) {
      ASSERT_TRUE(tr.out[i].deploy) << tr.t[i];
    }
  }
  const double t_landed = tr.first(FlightState::Landed);
  EXPECT_GT(t_landed, t_land + 5.0 - 0.8);  // ref sample may be set just before touchdown
  EXPECT_LT(t_landed, t_land + 6.5);
  EXPECT_EQ(tr.out.back().state, FlightState::Landed);
}

TEST(StateMachine, NoLandingWhileDescendingUnderChute) {
  Profile p;
  StateMachine sm;
  Trace tr;
  const double t_ap = p.apogee_t(), z_ap = p.apogee_z();
  // Descent at 1.5 m/s (2.5x slower than the real ~3.8 m/s) for 150 s, never reaching ground.
  auto alt = [&](double t) { return t <= t_ap ? p.alt(t) : z_ap - 1.5 * (t - t_ap); };
  auto acc = [&](double t) { return t <= t_ap ? p.accel(t) : kG; };
  drive(sm, tr, -200, static_cast<int>((t_ap + 150.0) / kDt), kDt, alt, acc, Sensors::nominal(), 5);
  EXPECT_TRUE(std::isnan(tr.first(FlightState::Landed)));
  EXPECT_EQ(tr.out.back().state, FlightState::Descent);
}

// ------------------------------------------------------------- general --

TEST(StateMachine, StatesNeverGoBackwards) {
  Profile p;
  for (unsigned seed = 1; seed <= 5; ++seed) {
    StateMachine sm;
    Trace tr;
    const double t_ap = p.apogee_t(), z_ap = p.apogee_z();
    auto alt = [&](double t) { return t <= t_ap ? p.alt(t) : std::max(0.0, z_ap - 3.8 * (t - t_ap)); };
    auto acc = [&](double t) { return t <= t_ap ? p.accel(t) : kG; };
    drive(sm, tr, -500, 9000, kDt, alt, acc, Sensors::nominal(), seed);
    for (std::size_t i = 1; i < tr.out.size(); ++i) {
      ASSERT_GE(static_cast<int>(tr.out[i].state), static_cast<int>(tr.out[i - 1].state));
    }
    EXPECT_EQ(tr.out.back().state, FlightState::Landed);
  }
}

TEST(StateMachine, IdenticalInputGivesIdenticalOutput_REQ006) {
  Profile p;
  Trace a, b;
  StateMachine sa, sb;
  drive(sa, a, -300, 1500, kDt, [&](double t) { return p.alt(t); }, [&](double t) { return p.accel(t); },
        Sensors::nominal(), 77);
  drive(sb, b, -300, 1500, kDt, [&](double t) { return p.alt(t); }, [&](double t) { return p.accel(t); },
        Sensors::nominal(), 77);
  ASSERT_EQ(a.out.size(), b.out.size());
  for (std::size_t i = 0; i < a.out.size(); ++i) {
    ASSERT_EQ(a.out[i].state, b.out[i].state);
    ASSERT_EQ(a.out[i].est_alt_m, b.out[i].est_alt_m);  // bitwise
    ASSERT_EQ(a.out[i].est_vel_mps, b.out[i].est_vel_mps);
    ASSERT_EQ(a.out[i].deploy, b.out[i].deploy);
  }
}
