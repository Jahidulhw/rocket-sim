#pragma once

// Session logic between the I/O loop (main.cpp) and the flight logic: turns
// one input line into one action. Kept free of I/O so it is unit-testable.

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "fc/config.hpp"
#include "fc/protocol.hpp"
#include "fc/state_machine.hpp"

namespace fc {

enum class Action {
  Reply,  // write `line` and flush
  Exit,   // END received: exit cleanly
  Hang,   // test-only fault: stop responding forever
};

struct Response {
  Action action = Action::Reply;
  std::string line;
  std::vector<std::string> diagnostics;  // health events, for stderr only
};

enum class Mode {
  Flight,  // state machine (event detection + deploy decision)
  Stub,    // plumbing test double: echo baro, PAD, never deploy
};

struct RunnerOptions {
  Mode mode = Mode::Flight;
  FcConfig config{};

  // Test-only fault injection: on the first frame with t >= hang_at_s the FC
  // stops replying (genuinely, the process blocks), so the simulator's
  // watchdog is exercised against a real hung process.
  std::optional<double> hang_at_s;
};

class Runner {
 public:
  explicit Runner(RunnerOptions opts = {});

  Response handle_line(std::string_view line);

 private:
  RunnerOptions opts_;
  StateMachine sm_;
  std::optional<double> last_t_;  // last ACCEPTED frame time (rejects time going backwards)
};

}  // namespace fc
