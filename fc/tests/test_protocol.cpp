#include <gtest/gtest.h>

#include <string>

#include "fc/protocol.hpp"

using fc::FlightState;
using fc::LineKind;
using fc::parse_line;

TEST(Protocol, ParsesValidSensorFrame) {
  auto p = parse_line("S 1.250000 312.400 -9.7500");
  ASSERT_EQ(p.kind, LineKind::Sensor) << p.error;
  EXPECT_DOUBLE_EQ(p.frame.t, 1.25);
  EXPECT_DOUBLE_EQ(p.frame.baro_alt_m, 312.4);
  EXPECT_DOUBLE_EQ(p.frame.accel_mps2, -9.75);
}

TEST(Protocol, AcceptsNegativeTimeAndExponents) {
  // Pre-ignition pad frames carry negative time stamps.
  auto p = parse_line("S -10.000000 1e-3 9.81E0");
  ASSERT_EQ(p.kind, LineKind::Sensor) << p.error;
  EXPECT_DOUBLE_EQ(p.frame.t, -10.0);
  EXPECT_DOUBLE_EQ(p.frame.baro_alt_m, 0.001);
  EXPECT_DOUBLE_EQ(p.frame.accel_mps2, 9.81);
}

TEST(Protocol, StripsSingleTrailingCarriageReturn) {
  // Windows text-mode pipes may deliver CRLF.
  EXPECT_EQ(parse_line("S 0 0 0\r").kind, LineKind::Sensor);
  EXPECT_EQ(parse_line("END\r").kind, LineKind::End);
}

TEST(Protocol, RecognisesEnd) {
  EXPECT_EQ(parse_line("END").kind, LineKind::End);
  EXPECT_EQ(parse_line("END ").kind, LineKind::Invalid);
  EXPECT_EQ(parse_line("end").kind, LineKind::Invalid);
}

TEST(Protocol, RejectsMalformedLines) {
  const char* bad[] = {
      "",                          // empty
      "S",                         // no fields
      "S 1 2",                     // too few fields
      "S 1 2 3 4",                 // too many fields
      "X 1 2 3",                   // unknown tag
      "s 1 2 3",                   // tags are case sensitive
      "S 1 2 abc",                 // non-numeric
      "S 1 2 3abc",                // trailing garbage inside a field
      "S 1  2 3",                  // double space (empty token)
      " S 1 2 3",                  // leading space
      "S 1 2 3 ",                  // trailing space
      "S\t1\t2\t3",                // tabs are not separators
      "S nan 2 3",                 // NaN
      "S 1 inf 3",                 // infinity
      "S 1 2 -inf",
      "S 1 2 1e999",               // overflows to infinity
      "S 0x10 2 3",                // hex is not a decimal number
      "S +1 2 3",                  // from_chars rejects a leading '+'; so do we
      "R 1 PAD 0 0 0",             // a reply echoed back is not a sensor frame
  };
  for (const char* line : bad) {
    auto p = parse_line(line);
    EXPECT_EQ(p.kind, LineKind::Invalid) << "accepted: '" << line << "'";
    if (p.kind == LineKind::Invalid) {
      EXPECT_FALSE(p.error.empty()) << line;
      EXPECT_EQ(p.error.find(' '), std::string::npos) << "reason must be one token: " << p.error;
    }
  }
}

TEST(Protocol, RejectsOverlongLine) {
  std::string line = "S 1 2 " + std::string(fc::kMaxLineLength, '1');
  auto p = parse_line(line);
  EXPECT_EQ(p.kind, LineKind::Invalid);
  EXPECT_EQ(p.error, "line_too_long");
}

TEST(Protocol, RejectsEmbeddedNul) {
  std::string line = "S 1 2 3";
  line[3] = '\0';
  EXPECT_EQ(parse_line(line).kind, LineKind::Invalid);
}

TEST(Protocol, FormatsReplyExactly) {
  fc::Reply r{2.5, FlightState::Coast, 123.4564, -0.0004, true};
  EXPECT_EQ(fc::format_reply(r), "R 2.500000 COAST 123.456 -0.000 1");
  r.deploy = false;
  r.state = FlightState::Pad;
  r.t = -0.01;
  EXPECT_EQ(fc::format_reply(r), "R -0.010000 PAD 123.456 -0.000 0");
}

TEST(Protocol, FormatsEveryStateName) {
  const std::pair<FlightState, const char*> names[] = {
      {FlightState::Pad, "PAD"},         {FlightState::Boost, "BOOST"},
      {FlightState::Coast, "COAST"},     {FlightState::Apogee, "APOGEE"},
      {FlightState::Descent, "DESCENT"}, {FlightState::Landed, "LANDED"},
  };
  for (const auto& [state, name] : names) {
    EXPECT_EQ(fc::to_string(state), name);
  }
}

TEST(Protocol, ReplyTimeRoundTripsThroughParser) {
  // The sim checks the echoed time; formatting must not lose the 1 us
  // resolution the sim sends.
  auto p = parse_line("S 12.340000 0 0");
  ASSERT_EQ(p.kind, LineKind::Sensor);
  fc::Reply r{p.frame.t, FlightState::Pad, 0.0, 0.0, false};
  EXPECT_EQ(fc::format_reply(r).substr(0, 12), "R 12.340000 ");
}

TEST(Protocol, FormatsErrorAsSingleToken) {
  EXPECT_EQ(fc::format_error("bad_number"), "E bad_number");
}
