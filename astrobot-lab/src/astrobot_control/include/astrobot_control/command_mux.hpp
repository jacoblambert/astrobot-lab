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

#pragma once
#include <optional>
#include <string>

namespace astrobot_control
{

struct Command
{
  double linear{0}, angular{0};
};

struct MuxConfig
{
  double nav_timeout_s{0.5}, teleop_timeout_s{0.5};
  double max_linear{0.5}, max_angular{1.2};
  double linear_scale{1.0}, angular_scale{1.0};
  bool require_localization{false};
  double localization_timeout_s{0.1};
};

// Velocity command arbitration: teleop overrides navigation (auto), or one source only (nav /
// teleop), or hold. Stale sources, e-stop and (optionally) a missing or denied localization
// permission all command zero. A re-granted permission requires fresh commands. Acceleration
// shaping belongs upstream (nav2_velocity_smoother). Times are monotonic seconds.
class CommandMux {
public:
  explicit CommandMux(MuxConfig config = {})
  : config_(config) {}
  static bool valid_mode(const std::string & mode);
  void set_mode(const std::string & mode);
  void set_estop(bool estop) {estop_ = estop;}
  void nav(const Command & command, double now);
  void teleop(const Command & command, double now);
  void localization(bool motion_permitted, double now);
  // Selected output and its source: "nav", "teleop" or "hold".
  Command step(double now, std::string * source = nullptr);
  Command select(double now, std::string * source) const;
  Command limit(const Command & command) const;

  const std::string & mode() const {return mode_;}
  bool estop() const {return estop_;}
  const Command & last_output() const {return last_output_;}
  const std::string & last_source() const {return last_source_;}
  std::optional<double> nav_age(double now) const {return age(nav_stamp_, now);}
  std::optional<double> teleop_age(double now) const {return age(teleop_stamp_, now);}

private:
  static std::optional<double> age(const std::optional<double> & stamp, double now);
  bool fresh(const std::optional<double> & stamp, double timeout, double now) const;
  MuxConfig config_;
  std::string mode_{"auto"};
  bool estop_{false};
  bool localization_allowed_{false};
  std::optional<double> localization_stamp_, nav_stamp_, teleop_stamp_;
  std::optional<Command> nav_, teleop_;
  Command last_output_;
  std::string last_source_{"hold"};
};

}  // namespace astrobot_control
