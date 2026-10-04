#pragma once

#include <string_view>

namespace fc {

// Flight-computer view of the flight. These are the FC's *beliefs*, derived
// only from sensor frames; they are deliberately distinct from the
// simulator's ground-truth phases (PAD, RAIL, BOOST, COAST, DESCENT, LANDED).
enum class FlightState { Pad, Boost, Coast, Apogee, Descent, Landed };

constexpr std::string_view to_string(FlightState s) {
  switch (s) {
    case FlightState::Pad: return "PAD";
    case FlightState::Boost: return "BOOST";
    case FlightState::Coast: return "COAST";
    case FlightState::Apogee: return "APOGEE";
    case FlightState::Descent: return "DESCENT";
    case FlightState::Landed: return "LANDED";
  }
  return "UNKNOWN";
}

}  // namespace fc
