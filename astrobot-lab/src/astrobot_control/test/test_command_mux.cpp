// Copyright 2026 Jacob Lambert
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "astrobot_control/command_mux.hpp"

namespace astrobot_control
{

namespace
{

CommandMux localized_mux()
{
  MuxConfig config;
  config.require_localization = true;
  CommandMux mux(config);
  mux.localization(true, 0.0);
  mux.localization(true, 0.05);  // a fresh permission
  mux.nav({0.1, 0.0}, 0.05);
  return mux;
}

}  // namespace

TEST(CommandMux, LocalizationLossInhibitsNavAndTeleop) {
  auto mux = localized_mux();
  EXPECT_EQ(mux.select(0.06, nullptr).linear, 0.1);
  EXPECT_EQ(mux.select(1.0, nullptr).linear, 0.0);  // permission heartbeat stale
  mux.localization(false, 0.07);
  mux.nav({0.1, 0.0}, 0.07);
  EXPECT_EQ(mux.select(0.08, nullptr).linear, 0.0);
}

TEST(CommandMux, NonfiniteCommandIsZero) {
  CommandMux mux;
  for (double value : {std::nan(""), std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()})
  {
    EXPECT_EQ(mux.limit({value, 0.0}).linear, 0.0);
  }
}

TEST(CommandMux, ReenableRequiresANewCommand) {
  MuxConfig config;
  config.require_localization = true;
  CommandMux mux(config);
  mux.nav({0.1, 0.0}, 0.0);
  mux.localization(true, 0.01);  // previously not allowed: cached commands are dropped
  std::string source;
  EXPECT_EQ(mux.select(0.02, &source).linear, 0.0);
  EXPECT_EQ(source, "hold");
}

TEST(CommandMux, ReenableAfterHeartbeatGapDiscardsCachedCommands) {
  auto mux = localized_mux();
  mux.localization(true, 1.0);  // 0.95 s since the last permission
  std::string source;
  EXPECT_EQ(mux.select(1.01, &source).linear, 0.0);
  EXPECT_EQ(source, "hold");
}

TEST(CommandMux, TeleopOverridesNavInAutoAndLimitsApply) {
  CommandMux mux;
  mux.nav({0.2, 0.0}, 0.0);
  mux.teleop({5.0, -9.0}, 0.0);
  std::string source;
  const auto out = mux.select(0.1, &source);
  EXPECT_EQ(source, "teleop");
  EXPECT_EQ(out.linear, 0.5);
  EXPECT_EQ(out.angular, -1.2);
  mux.set_mode("nav");
  EXPECT_EQ(mux.select(0.1, &source).linear, 0.2);
  mux.set_estop(true);
  EXPECT_EQ(mux.select(0.1, &source).linear, 0.0);
}

}  // namespace astrobot_control
