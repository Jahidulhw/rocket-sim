#include "fc/protocol.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <system_error>

namespace fc {
namespace {

// std::from_chars rather than strtod/stod: it is locale-independent (a
// German locale would make strtod expect "1,5"), never allocates, never
// throws, and reports exactly how many characters it consumed, which is what
// makes "the whole token must be a number" easy to enforce.
bool parse_double(std::string_view tok, double& out) {
  if (tok.empty()) return false;
  const char* first = tok.data();
  const char* last = tok.data() + tok.size();
  auto [ptr, ec] = std::from_chars(first, last, out, std::chars_format::general);
  return ec == std::errc{} && ptr == last && std::isfinite(out);
}

ParsedLine invalid(const char* reason) {
  ParsedLine p;
  p.kind = LineKind::Invalid;
  p.error = reason;
  return p;
}

std::string printf_string(const char* fmt, double t, const char* state, double a, double v,
                          int deploy) {
  std::array<char, 128> buf{};
  int n = std::snprintf(buf.data(), buf.size(), fmt, t, state, a, v, deploy);
  if (n < 0) return "E format_failure";
  if (static_cast<std::size_t>(n) < buf.size()) return std::string(buf.data(), static_cast<std::size_t>(n));
  // Absurdly large magnitudes print long under %f; size the buffer exactly.
  std::string s(static_cast<std::size_t>(n) + 1, '\0');
  std::snprintf(s.data(), s.size(), fmt, t, state, a, v, deploy);
  s.resize(static_cast<std::size_t>(n));
  return s;
}

}  // namespace

ParsedLine parse_line(std::string_view line) {
  if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
  if (line.size() > kMaxLineLength) return invalid("line_too_long");
  if (line == "END") return ParsedLine{LineKind::End, {}, {}};
  if (line.empty()) return invalid("empty_line");

  // Split on single spaces; an empty token (double/leading/trailing space)
  // is an error rather than something to skip over.
  std::array<std::string_view, 5> tok{};
  std::size_t n = 0;
  std::size_t start = 0;
  while (true) {
    std::size_t sp = line.find(' ', start);
    std::string_view t = line.substr(start, sp == std::string_view::npos ? std::string_view::npos : sp - start);
    if (t.empty()) return invalid("empty_field");
    if (n == tok.size()) return invalid("bad_field_count");
    tok[n++] = t;
    if (sp == std::string_view::npos) break;
    start = sp + 1;
  }

  if (tok[0] != "S") return invalid("unknown_tag");
  if (n != 4) return invalid("bad_field_count");

  SensorFrame f;
  if (!parse_double(tok[1], f.t) || !parse_double(tok[2], f.baro_alt_m) ||
      !parse_double(tok[3], f.accel_mps2)) {
    return invalid("bad_number");
  }
  return ParsedLine{LineKind::Sensor, f, {}};
}

std::string format_reply(const Reply& r) {
  // t keeps microsecond resolution (the sim sends %.6f and checks the echo);
  // estimates are printed to 1 mm / 1 mm/s, far below sensor noise.
  const std::string_view s = to_string(r.state);
  const std::string state(s);
  return printf_string("R %.6f %s %.3f %.3f %d", r.t, state.c_str(), r.est_alt_m, r.est_vel_mps,
                       (r.deploy ? 1 : 0) | (r.deploy_main ? 2 : 0));
}

std::string format_error(std::string_view reason) {
  return "E " + std::string(reason);
}

}  // namespace fc
