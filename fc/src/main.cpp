// Flight computer process: one sensor frame in on stdin, one reply out on
// stdout, flushed every time (strict lockstep, see docs/protocol.md).
// Diagnostics go to stderr only, so stdout carries nothing but protocol.
//
// Time comes exclusively from frame stamps; the wall clock is never read.
// That is what makes a run reproducible bit-for-bit from its input.

#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>

#include "fc/config.hpp"
#include "fc/protocol.hpp"
#include "fc/runner.hpp"

namespace {

void usage() {
  std::cerr << "usage: flight_computer [--mode kalman|baseline|stub] [--inject-hang-at <t>]\n"
               "  --mode kalman         apogee from Kalman velocity estimate (default)\n"
               "  --mode baseline       apogee from raw barometer\n"
               "  --mode stub           TEST ONLY: echo the barometer, report PAD, never deploy\n"
               "  --param key=value     override a tuning parameter (see fc/src/config.cpp)\n"
               "  --inject-hang-at <t>  TEST ONLY: stop responding at the first frame with time >= t\n";
}

bool parse_args(int argc, char** argv, fc::RunnerOptions& opts) {
  for (int i = 1; i < argc; ++i) {
    const std::string_view a = argv[i];
    if (a == "--inject-hang-at" && i + 1 < argc) {
      // Reuse the strict protocol number parser for the argument.
      const std::string probe = std::string("S ") + argv[++i] + " 0 0";
      const fc::ParsedLine p = fc::parse_line(probe);
      if (p.kind != fc::LineKind::Sensor) {
        std::cerr << "invalid --inject-hang-at value\n";
        return false;
      }
      opts.hang_at_s = p.frame.t;
    } else if (a == "--mode" && i + 1 < argc) {
      const std::string_view m = argv[++i];
      if (m == "kalman") {
        opts.mode = fc::Mode::Flight;
        opts.config.apogee_mode = fc::ApogeeMode::Kalman;
      } else if (m == "baseline") {
        opts.mode = fc::Mode::Flight;
        opts.config.apogee_mode = fc::ApogeeMode::Baseline;
      } else if (m == "stub") {
        opts.mode = fc::Mode::Stub;
      } else {
        std::cerr << "unknown mode: " << m << "\n";
        return false;
      }
    } else if (a == "--param" && i + 1 < argc) {
      // --param key=value: tuning studies only; defaults are the flight values.
      const std::string_view kv = argv[++i];
      const std::size_t eq = kv.find('=');
      const fc::ParsedLine p = eq == std::string_view::npos
                                   ? fc::ParsedLine{}
                                   : fc::parse_line("S " + std::string(kv.substr(eq + 1)) + " 0 0");
      if (p.kind != fc::LineKind::Sensor || !fc::set_param(opts.config, kv.substr(0, eq), p.frame.t)) {
        std::cerr << "invalid --param: " << kv << "\n";
        return false;
      }
    } else if (a == "--help" || a == "-h") {
      usage();
      return false;
    } else {
      std::cerr << "unknown argument: " << a << "\n";
      usage();
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  fc::RunnerOptions opts;
  if (!parse_args(argc, argv, opts)) return 2;

  std::ios::sync_with_stdio(false);
  fc::Runner runner(opts);
  std::string line;
  while (std::getline(std::cin, line)) {
    fc::Response r = runner.handle_line(line);
    switch (r.action) {
      case fc::Action::Reply:
        std::cout << r.line << '\n';
        std::cout.flush();  // the sim is blocked waiting for this line
        break;
      case fc::Action::Exit:
        return 0;
      case fc::Action::Hang:
        std::cerr << "flight_computer: injected hang, no further replies\n";
        for (;;) std::this_thread::sleep_for(std::chrono::hours(1));
    }
  }
  std::cerr << "flight_computer: stdin closed without END\n";
  return 1;
}
