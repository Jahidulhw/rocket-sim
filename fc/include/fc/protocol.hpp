#pragma once

// Line protocol between the simulator and the flight computer (docs/protocol.md).
//
//   Sim -> FC:  "S <t> <baro_alt_m> <accel_mps2>"   one per sensor tick
//               "END"                               orderly shutdown
//   FC -> Sim:  "R <t> <state> <est_alt> <est_vel> <deploy 0-3>"
//               deploy is a bitmask: 1 = drogue/primary chute, 2 = main chute
//               (dual deploy); a single-deploy rocket only ever sends 0 or 1
//               "E <reason>"                        input line rejected
//
// Parsing is strict on purpose: single-space separators, exact field count,
// every number fully consumed and finite. Anything else is rejected rather
// than "best-effort" interpreted, because a misparsed altitude is worse than
// a missing one: a missing frame is visible, a wrong one is silent.

#include <cstddef>
#include <string>
#include <string_view>

#include "fc/flight_state.hpp"

namespace fc {

inline constexpr std::size_t kMaxLineLength = 256;

struct SensorFrame {
  double t = 0.0;           // s, simulator time stamp (the FC's only clock)
  double baro_alt_m = 0.0;  // m, barometric altitude
  double accel_mps2 = 0.0;  // m/s^2, vertical specific force, +up (reads +g at rest)
};

enum class LineKind { Sensor, End, Invalid };

struct ParsedLine {
  LineKind kind = LineKind::Invalid;
  SensorFrame frame{};      // valid only when kind == Sensor
  std::string error;        // short machine-readable reason when kind == Invalid
};

ParsedLine parse_line(std::string_view line);

struct Reply {
  double t = 0.0;
  FlightState state = FlightState::Pad;
  double est_alt_m = 0.0;
  double est_vel_mps = 0.0;
  bool deploy = false;       // primary (drogue) chute commanded
  bool deploy_main = false;  // main chute commanded (dual deploy)
};

// Both return the line WITHOUT the trailing newline.
std::string format_reply(const Reply& r);
std::string format_error(std::string_view reason);

}  // namespace fc
