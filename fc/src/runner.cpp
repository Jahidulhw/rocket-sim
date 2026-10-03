#include "fc/runner.hpp"

namespace fc {

Runner::Runner(RunnerOptions opts) : opts_(opts) {}

Response Runner::handle_line(std::string_view line) {
  const ParsedLine p = parse_line(line);
  switch (p.kind) {
    case LineKind::End:
      return {Action::Exit, {}};
    case LineKind::Invalid:
      return {Action::Reply, format_error(p.error)};
    case LineKind::Sensor:
      break;
  }

  const SensorFrame& f = p.frame;
  // dt is derived from these stamps, so time must strictly increase: a
  // repeated or backwards stamp would give dt <= 0 and corrupt any filter.
  if (last_t_ && !(f.t > *last_t_)) return {Action::Reply, format_error("non_increasing_time")};
  if (opts_.hang_at_s && f.t >= *opts_.hang_at_s) return {Action::Hang, {}};
  last_t_ = f.t;

  // Milestone 1 stub: report PAD, pass the barometer through, never deploy.
  Reply r{f.t, FlightState::Pad, f.baro_alt_m, 0.0, false};
  return {Action::Reply, format_reply(r)};
}

}  // namespace fc
