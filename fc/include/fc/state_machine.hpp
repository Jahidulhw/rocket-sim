#pragma once

// Flight event detection: PAD -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED.
// The ONLY actuator output is the parachute deploy command. There is no
// guidance, steering or attitude control of any kind.

#include <optional>
#include <string>
#include <vector>

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

// Stuck-sensor detector: a live sensor with real noise essentially never
// repeats a bit-identical reading, so N identical readings in a row means the
// value is frozen (stuck register, dead bus returning stale data).
class StaleDetector {
 public:
  explicit StaleDetector(int required) : required_(required) {}
  // Returns true when the run of identical values reaches `required`.
  // `exempt` samples (e.g. accelerometer at full scale) never count.
  bool update(double value, bool exempt = false);

 private:
  int required_;  // 0 disables
  int run_ = 0;
  double last_ = 0.0;
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

  // Health (latched once set).
  bool baro_failed() const { return baro_failed_; }
  bool accel_failed() const { return accel_failed_; }
  bool estimator_inconsistent() const { return inconsistent_; }
  int baro_rejections_total() const { return baro_rejected_total_; }
  int accel_rejections_total() const { return accel_rejected_total_; }

  // Human-readable health events since the last call (main.cpp prints them
  // to stderr). Kept out of the reply so the protocol stays fixed.
  std::vector<std::string> take_diagnostics();

 private:
  void check_health(const SensorFrame& f);
  void run_filter(double dt, const SensorFrame& f, double agl);
  void on_pad(const SensorFrame& f, double agl);
  void on_boost(const SensorFrame& f, double agl);
  void on_coast(const SensorFrame& f, double agl);
  void on_descent(const SensorFrame& f, double agl);
  void command_deploy(DeployReason why);
  void diag(double t, const std::string& msg);

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

  StaleDetector baro_stale_;
  StaleDetector accel_stale_;
  bool baro_failed_ = false;
  bool accel_failed_ = false;
  bool inconsistent_ = false;
  int baro_rejected_run_ = 0;
  int accel_rejected_run_ = 0;
  int baro_rejected_total_ = 0;
  int accel_rejected_total_ = 0;

  Persistence launch_accel_;
  Persistence launch_baro_;
  Persistence burnout_;
  Persistence apogee_baro_;
  Persistence apogee_kf_;
  std::optional<double> launch_t_;
  double max_agl_ = 0.0;

  bool land_filt_init_ = false;
  double land_filt_m_ = 0.0;  // low-pass AGL for landing detection
  bool land_ref_init_ = false;
  double land_ref_m_ = 0.0;
  double land_start_t_ = 0.0;
  double land_last_t_ = 0.0;

  std::vector<std::string> diagnostics_;
};

}  // namespace fc
