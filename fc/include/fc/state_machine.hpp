#pragma once

// Flight event detection: PAD -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED.
// The ONLY actuator output is the parachute deploy command. There is no
// guidance, steering or attitude control of any kind.

#include <optional>

#include "fc/config.hpp"
#include "fc/flight_state.hpp"
#include "fc/kalman.hpp"
#include "fc/protocol.hpp"

namespace fc {

enum class DeployReason { None, Apogee, BackupTimer };

struct FcOutput {
  FlightState state = FlightState::Pad;
  double est_alt_m = 0.0;    // above the ground reference
  double est_vel_mps = 0.0;  // Kalman estimate; baseline mode has none and reports 0
  bool deploy = false;       // latched: stays true once commanded
};

// "Condition held for N consecutive samples", where a frame gap longer than
// max_gap_s breaks the run.
class Persistence {
 public:
  explicit Persistence(int required) : required_(required) {}
  bool update(bool condition, double t, double max_gap_s);
  void reset() { count_ = 0; }
  int count() const { return count_; }
  double first_t() const { return first_t_; }  // time the current run started

 private:
  int required_;
  int count_ = 0;
  double first_t_ = 0.0;
  std::optional<double> last_t_;
};

class StateMachine {
 public:
  explicit StateMachine(FcConfig cfg = {});

  // One sensor frame in, one decision out. Frames must have strictly
  // increasing t (the Runner enforces this); t is the only clock.
  FcOutput update(const SensorFrame& f);

  FlightState state() const { return state_; }
  DeployReason deploy_reason() const { return reason_; }
  std::optional<double> launch_time() const { return launch_t_; }
  double ground_ref_m() const { return ground_m_; }
  double accel_ref_mps2() const { return accel_ref_; }
  const Kalman3& filter() const { return kf_; }

 private:
  void on_pad(const SensorFrame& f, double agl);
  void on_boost(const SensorFrame& f, double agl);
  void on_coast(const SensorFrame& f, double agl);
  void on_descent(const SensorFrame& f, double agl);
  void command_deploy(DeployReason why);

  FcConfig cfg_;
  FlightState state_ = FlightState::Pad;
  DeployReason reason_ = DeployReason::None;
  bool deploy_ = false;

  std::optional<double> prev_t_;
  bool ground_init_ = false;
  double ground_start_t_ = 0.0;
  int ground_n_ = 0;
  double ground_m_ = 0.0;
  double accel_ref_ = 0.0;

  Kalman3 kf_;

  Persistence launch_accel_;
  Persistence launch_baro_;
  Persistence burnout_;
  Persistence apogee_baro_;
  Persistence apogee_kf_;
  std::optional<double> launch_t_;
  double max_agl_ = 0.0;

  bool land_ref_init_ = false;
  double land_ref_m_ = 0.0;
  double land_start_t_ = 0.0;
  double land_last_t_ = 0.0;
};

}  // namespace fc
