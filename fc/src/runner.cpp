#include "fc/runner.hpp"

namespace fc {

Runner::Runner(RunnerOptions opts) : opts_(opts), sm_(opts.config) {}

Response Runner::handle_line(std::string_view line) {
  const ParsedLine p = parse_line(line);
  switch (p.kind) {
    case LineKind::End:
      return {Action::Exit, {}, {}};
    case LineKind::Invalid:
      return {Action::Reply, format_error(p.error), {}};
    case LineKind::Sensor:
      break;
  }

  const SensorFrame& f = p.frame;
  // dt is derived from these stamps, so time must strictly increase: a
  // repeated or backwards stamp would give dt <= 0 and corrupt any filter.
  if (last_t_ && !(f.t > *last_t_)) return {Action::Reply, format_error("non_increasing_time"), {}};
  if (opts_.hang_at_s && f.t >= *opts_.hang_at_s) return {Action::Hang, {}, {}};
  last_t_ = f.t;

  if (opts_.mode == Mode::Stub) {
    return {Action::Reply, format_reply(Reply{f.t, FlightState::Pad, f.baro_alt_m, 0.0, false, false}), {}};
  }
  const FcOutput o = sm_.update(f);
  return {Action::Reply, format_reply(Reply{f.t, o.state, o.est_alt_m, o.est_vel_mps, o.deploy, o.deploy_main}),
          sm_.take_diagnostics()};
}

}  // namespace fc
