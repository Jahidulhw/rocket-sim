#include <gtest/gtest.h>

#include "fc/runner.hpp"

using fc::Action;
using fc::Runner;

TEST(Runner, RepliesToEveryValidFrameEchoingTime) {
  Runner r;
  auto a = r.handle_line("S 0.000000 1.500 9.8100");
  ASSERT_EQ(a.action, Action::Reply);
  EXPECT_EQ(a.line.substr(0, 11), "R 0.000000 ");
  auto b = r.handle_line("S 0.010000 1.600 9.8100");
  ASSERT_EQ(b.action, Action::Reply);
  EXPECT_EQ(b.line.substr(0, 11), "R 0.010000 ");
}

TEST(Runner, MalformedLineGetsErrorReplyAndSessionContinues) {
  // Strict lockstep: every input line gets exactly one reply, even garbage,
  // so the sim never waits on a line the FC silently dropped.
  Runner r;
  auto bad = r.handle_line("S 1 2");
  ASSERT_EQ(bad.action, Action::Reply);
  EXPECT_EQ(bad.line.rfind("E ", 0), 0u) << bad.line;
  auto good = r.handle_line("S 0.000000 0 9.81");
  ASSERT_EQ(good.action, Action::Reply);
  EXPECT_EQ(good.line.rfind("R ", 0), 0u) << good.line;
}

TEST(Runner, RejectsNonIncreasingTimestamps) {
  Runner r;
  ASSERT_EQ(r.handle_line("S 1.000000 0 9.81").line.rfind("R ", 0), 0u);
  EXPECT_EQ(r.handle_line("S 1.000000 0 9.81").line, "E non_increasing_time");
  EXPECT_EQ(r.handle_line("S 0.500000 0 9.81").line, "E non_increasing_time");
  // A rejected frame does not advance the clock: the next good frame works.
  EXPECT_EQ(r.handle_line("S 1.010000 0 9.81").line.rfind("R ", 0), 0u);
}

TEST(Runner, RejectedFrameDoesNotAdvanceClock) {
  Runner r;
  ASSERT_EQ(r.handle_line("S 1.000000 0 9.81").action, Action::Reply);
  EXPECT_EQ(r.handle_line("S 5.000000 0 abc").line.rfind("E ", 0), 0u);
  EXPECT_EQ(r.handle_line("S 2.000000 0 9.81").line.rfind("R ", 0), 0u);
}

TEST(Runner, EndRequestsExit) {
  Runner r;
  EXPECT_EQ(r.handle_line("END").action, Action::Exit);
}

TEST(Runner, StubReportsPadNoDeploy) {
  Runner r;
  auto a = r.handle_line("S 3.000000 42.000 -9.8100");
  ASSERT_EQ(a.action, Action::Reply);
  EXPECT_NE(a.line.find(" PAD "), std::string::npos) << a.line;
  EXPECT_EQ(a.line.back(), '0') << a.line;
}

TEST(Runner, InjectedHangTriggersAtConfiguredTime) {
  fc::RunnerOptions opts;
  opts.hang_at_s = 0.5;
  Runner r(opts);
  EXPECT_EQ(r.handle_line("S 0.490000 0 9.81").action, Action::Reply);
  EXPECT_EQ(r.handle_line("S 0.500000 0 9.81").action, Action::Hang);
}

TEST(Runner, NoHangWithoutOption) {
  Runner r;
  for (int k = 0; k < 1000; ++k) {
    std::string line = "S " + std::to_string(k * 0.01) + " 0 9.81";
    ASSERT_EQ(r.handle_line(line).action, Action::Reply) << line;
  }
}
