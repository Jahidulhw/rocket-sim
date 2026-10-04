// Two burns (passive staging) and dual deploy in the flight computer (Phase A5).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "fc/config.hpp"
#include "fc/protocol.hpp"
#include "fc/state_machine.hpp"
#include "flight_profile.hpp"
#include "test_util.hpp"

using fc::DeployReason;
using fc::FcConfig;
using fc::FlightState;
using fc::StateMachine;
using fc_test::drive;
using fc_test::kG;
using fc_test::Sensors;
using fc_test::Trace;

namespace {

constexpr double kDt = 0.01;

// Piecewise-constant kinematic acceleration from t = 0 (pad before that).
// Specific force = a + g. Closed form within each segment.
class Segments {
 public:
  explicit Segments(std::vector<std::pair<double, double>> segs) : segs_(std::move(segs)) {
    double t = 0.0, z = 0.0, v = 0.0;
    for (const auto& [dur, a] : segs_) {
      starts_.push_back({t, z, v});
      z += v * dur + 0.5 * a * dur * dur;
      v += a * dur;
      t += dur;
    }
  }
  double alt(double t) const { return state(t).first; }
  double vel(double t) const { return state(t).second; }
  double accel(double t) const {  // specific force
    if (t <= 0.0) return kG;
    return seg(t).second + kG;
  }
  double end(std::size_t i) const { return starts_[i][0] + segs_[i].first; }
  // Time where velocity crosses zero in the segment that decelerates through it.
  double apogee_t() const {
    for (std::size_t i = 0; i < segs_.size(); ++i) {
      const double v0 = starts_[i][2], a = segs_[i].second;
      if (v0 > 0 && a < 0 && v0 / -a <= segs_[i].first) return starts_[i][0] + v0 / -a;
    }
    return std::nan("");
  }

 private:
  std::pair<double, double> seg(double t) const {
    for (std::size_t i = 0; i < segs_.size(); ++i)
      if (t <= end(i)) return {static_cast<double>(i), segs_[i].second};
    return {static_cast<double>(segs_.size() - 1), segs_.back().second};
  }
  std::pair<double, double> state(double t) const {
    if (t <= 0.0) return {0.0, 0.0};
    std::size_t i = static_cast<std::size_t>(seg(t).first);
    const double tau = t - starts_[i][0], a = segs_[i].second;
    return {starts_[i][1] + starts_[i][2] * tau + 0.5 * a * tau * tau, starts_[i][2] + a * tau};
  }
  std::vector<std::pair<double, double>> segs_;
  std::vector<std::array<double, 3>> starts_;
};

// Booster 1.5 s at 40 m/s^2, 1.0 s gap (drag + gravity), sustainer 1.5 s at
// 50 m/s^2, then coast to apogee (drag + gravity).
Segments two_burn() { return Segments({{1.5, 40.0}, {1.0, -(kG + 5.0)}, {1.5, 50.0}, {30.0, -(kG + 2.0)}}); }
// Same booster, sustainer never lights.
Segments dud() { return Segments({{1.5, 40.0}, {30.0, -(kG + 3.0)}}); }

FcConfig two_burn_cfg() {
  FcConfig c;
  c.burns = 2;
  c.min_deploy_after_launch_s = 5.0;
  c.backup_timer_s = 40.0;
  return c;
}

std::vector<FlightState> sequence(const Trace& tr) {
  std::vector<FlightState> s;
  for (const auto& o : tr.out)
    if (s.empty() || s.back() != o.state) s.push_back(o.state);
  return s;
}

}  // namespace

TEST(MultiBurn, TwoBurnSequenceNoDeployBeforeFinalBurnout_REQ011_REQ012) {
  const Segments p = two_burn();
  const double final_burnout = p.end(2);
  for (unsigned seed = 1; seed <= 10; ++seed) {
    StateMachine sm(two_burn_cfg());
    Trace tr;
    drive(sm, tr, -500, 1500, kDt, [&](double t) { return p.alt(t); }, [&](double t) { return p.accel(t); },
          Sensors::nominal(), seed);
    const auto seq = sequence(tr);
    const std::vector<FlightState> want = {FlightState::Pad, FlightState::Boost, FlightState::Coast,
                                           FlightState::Boost, FlightState::Coast, FlightState::Apogee,
                                           FlightState::Descent};
    ASSERT_GE(seq.size(), want.size()) << seed;
    EXPECT_TRUE(std::equal(want.begin(), want.end(), seq.begin())) << seed;
    for (std::size_t i = 0; i < tr.t.size(); ++i) {
      if (tr.t[i] < final_burnout) {
        ASSERT_FALSE(tr.out[i].deploy) << "deploy during boost/gap at t=" << tr.t[i];
        ASSERT_NE(tr.out[i].state, FlightState::Apogee) << tr.t[i];    // staging is not apogee
        ASSERT_NE(tr.out[i].state, FlightState::Descent) << tr.t[i];
        ASSERT_NE(tr.out[i].state, FlightState::Landed) << tr.t[i];
      }
    }
    const double err = tr.first(FlightState::Apogee) - p.apogee_t();
    EXPECT_GT(err, -0.05) << seed;
    EXPECT_LT(err, 0.15) << seed;
    EXPECT_EQ(sm.deploy_reason(), DeployReason::Apogee);
    EXPECT_FALSE(sm.estimator_inconsistent()) << "sustainer ignition must not trip the gate";
  }
}

TEST(MultiBurn, NoDeployInTheGapEvenWithAnApogeeLikeBaro_REQ011) {
  // Hostile input in the gap: barometer reads falling altitude (apogee-like).
  const Segments p = two_burn();
  StateMachine sm(two_burn_cfg());
  Trace tr;
  auto alt = [&](double t) { return (t > 1.6 && t < 2.5) ? p.alt(1.6) - 30.0 * (t - 1.6) : p.alt(t); };
  drive(sm, tr, -300, 400, kDt, alt, [&](double t) { return p.accel(t); }, Sensors::nominal(), 3);
  for (std::size_t i = 0; i < tr.t.size(); ++i) {
    ASSERT_FALSE(tr.out[i].deploy) << tr.t[i];
  }
  EXPECT_TRUE(sm.awaiting_ignition() || sm.state() == FlightState::Boost || sm.state() == FlightState::Coast);
}

TEST(MultiBurn, DudSustainerTimesOutAndStillDeploysAtApogee_REQ004) {
  const Segments p = dud();
  FcConfig cfg = two_burn_cfg();
  StateMachine sm(cfg);
  Trace tr;
  bool timeout_msg = false;
  fc_test::Noise n(4);
  for (int k = -500; k < 1200; ++k) {
    const double t = k * kDt;
    const double baro = fc_test::quantize(p.alt(t) + 0.5 + 0.5 * n.gauss(), 0.1);
    tr.t.push_back(t);
    tr.out.push_back(sm.update({t, baro, p.accel(t) + 0.2 + 0.5 * n.gauss()}));
    for (const auto& d : sm.take_diagnostics()) timeout_msg |= d.find("no ignition") != std::string::npos;
  }
  EXPECT_TRUE(timeout_msg);
  const double err = tr.first(FlightState::Apogee) - p.apogee_t();
  EXPECT_GT(err, -0.05);
  EXPECT_LT(err, 0.15);
}

TEST(MultiBurn, UnexpectedSecondBurnReturnsToBoost_REQ003) {
  // Misconfigured as single-burn: the sustainer still cannot get a deploy under thrust.
  const Segments p = two_burn();
  FcConfig cfg;   // burns = 1
  cfg.backup_timer_s = 40.0;
  StateMachine sm(cfg);
  Trace tr;
  drive(sm, tr, -500, 1500, kDt, [&](double t) { return p.alt(t); }, [&](double t) { return p.accel(t); },
        Sensors::nominal(), 5);
  const auto seq = sequence(tr);
  EXPECT_GE(std::count(seq.begin(), seq.end(), FlightState::Boost), 2);
  for (std::size_t i = 0; i < tr.t.size(); ++i) {
    if (tr.t[i] < p.end(2)) {
      ASSERT_FALSE(tr.out[i].deploy) << tr.t[i];
    }
  }
  EXPECT_LT(std::fabs(tr.first(FlightState::Apogee) - p.apogee_t()), 0.3);
}

// ------------------------------------------------------------- dual deploy --

TEST(DualDeploy, MainCommandedBelowItsAltitudeAfterDrogue_REQ013) {
  // Single burn, then after apogee a drogue descent at 20 m/s.
  fc_test::Profile prof;
  const double t_ap = prof.apogee_t(), z_ap = prof.apogee_z();
  auto alt = [&](double t) { return t <= t_ap ? prof.alt(t) : std::max(0.0, z_ap - 20.0 * (t - t_ap)); };
  auto acc = [&](double t) { return t <= t_ap ? prof.accel(t) : kG; };
  FcConfig cfg;
  cfg.main_deploy_altitude_m = 100.0;
  StateMachine sm(cfg);
  Trace tr;
  drive(sm, tr, -500, 2000, kDt, alt, acc, Sensors::nominal(), 6);
  std::optional<double> t_main;
  for (std::size_t i = 0; i < tr.t.size(); ++i) {
    if (tr.out[i].deploy_main && !t_main) t_main = tr.t[i];
    if (tr.out[i].deploy_main) {
      ASSERT_TRUE(tr.out[i].deploy) << "main before drogue";
    }
  }
  ASSERT_TRUE(t_main.has_value());
  EXPECT_LE(alt(*t_main), 100.0);
  EXPECT_GE(alt(*t_main), 100.0 - 6.0);   // 5 samples at 20 m/s + estimate noise
}

TEST(DualDeploy, SingleDeployRocketNeverCommandsMain) {
  fc_test::Profile prof;
  StateMachine sm;   // main_deploy_altitude_m = 0
  Trace tr;
  drive(sm, tr, -500, 1500, kDt, [&](double t) { return prof.alt(t); }, [&](double t) { return prof.accel(t); },
        Sensors::nominal(), 7);
  for (const auto& o : tr.out) {
    ASSERT_FALSE(o.deploy_main);
  }
}

TEST(DualDeploy, FailedBaroCommandsMainWithDrogue_REQ013) {
  fc_test::Profile prof;
  FcConfig cfg;
  cfg.main_deploy_altitude_m = 100.0;
  StateMachine sm(cfg);
  Trace tr;
  std::optional<double> held;
  auto corrupt = [&](double t, double& baro, double&) {
    if (t >= 3.0) {
      if (!held) held = baro;
      baro = *held;
    }
  };
  drive(sm, tr, -500, 1200, kDt, [&](double t) { return prof.alt(t); }, [&](double t) { return prof.accel(t); },
        Sensors::nominal(), 8, corrupt);
  ASSERT_TRUE(sm.baro_failed());
  const double t_drogue = tr.first_deploy();
  ASSERT_FALSE(std::isnan(t_drogue));
  for (std::size_t i = 0; i < tr.t.size(); ++i) {
    if (tr.t[i] > t_drogue + 0.02) {
      ASSERT_TRUE(tr.out[i].deploy_main) << tr.t[i];
    }
  }
}

TEST(DualDeploy, ReplyCarriesDeployBitmask) {
  fc::Reply r{1.0, FlightState::Descent, 90.0, -20.0, true, true};
  EXPECT_EQ(fc::format_reply(r), "R 1.000000 DESCENT 90.000 -20.000 3");
  r.deploy_main = false;
  EXPECT_EQ(fc::format_reply(r), "R 1.000000 DESCENT 90.000 -20.000 1");
}

TEST(Config, SetParamCoversEveryTuningField) {
  FcConfig c;
  const std::pair<const char*, double> ok[] = {
      {"burns", 2}, {"stage_ignition_timeout_s", 2.5}, {"main_deploy_altitude_m", 300},
      {"launch_accel_mps2", 30}, {"max_boost_s", 4.5}, {"accel_full_scale_mps2", 1962},
      {"accel_ref_gate_mps2", 4}, {"kf.accel_sigma_mps2", 1.0}, {"next_ignition_accel_mps2", 25}};
  for (const auto& [k, v] : ok) {
    EXPECT_TRUE(fc::set_param(c, k, v)) << k;
  }
  EXPECT_EQ(c.burns, 2);
  EXPECT_DOUBLE_EQ(c.main_deploy_altitude_m, 300.0);
  EXPECT_DOUBLE_EQ(c.kf.accel_sigma_mps2, 1.0);
  EXPECT_FALSE(fc::set_param(c, "burns", 0));            // at least one burn
  EXPECT_FALSE(fc::set_param(c, "burns", 1.5));
  EXPECT_FALSE(fc::set_param(c, "max_boost_s", -1));
  EXPECT_FALSE(fc::set_param(c, "max_boost_s", std::nan("")));
  EXPECT_FALSE(fc::set_param(c, "stage_charge_s", 1));    // unknown: the FC has no staging command
}
