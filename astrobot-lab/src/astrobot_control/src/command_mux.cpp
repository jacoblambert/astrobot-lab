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

#include "astrobot_control/command_mux.hpp"

#include <algorithm>
#include <cmath>

namespace astrobot_control
{
namespace
{

double clamp(double value, double lo, double hi) {return std::min(std::max(value, lo), hi);}

}  // namespace

bool CommandMux::valid_mode(const std::string & mode)
{
  return mode == "auto" || mode == "nav" || mode == "teleop" || mode == "hold";
}

void CommandMux::set_mode(const std::string & mode) {mode_ = valid_mode(mode) ? mode : mode_;}

std::optional<double> CommandMux::age(const std::optional<double> & stamp, double now)
{
  if (!stamp) {return std::nullopt;}
  return std::max(now - *stamp, 0.0);
}

bool CommandMux::fresh(const std::optional<double> & stamp, double timeout, double now) const
{
  const auto a = age(stamp, now);
  return a && *a <= timeout;
}

Command CommandMux::limit(const Command & command) const
{
  if (!std::isfinite(command.linear) || !std::isfinite(command.angular)) {return Command{};}
  return {clamp(command.linear * config_.linear_scale, -config_.max_linear, config_.max_linear),
    clamp(command.angular * config_.angular_scale, -config_.max_angular, config_.max_angular)};
}

void CommandMux::nav(const Command & command, double now)
{
  nav_ = limit(command);
  nav_stamp_ = now;
}

void CommandMux::teleop(const Command & command, double now)
{
  teleop_ = limit(command);
  teleop_stamp_ = now;
}

void CommandMux::localization(bool motion_permitted, double now)
{
  const bool was_allowed = localization_allowed_ && fresh(localization_stamp_,
      config_.localization_timeout_s, now);
  localization_allowed_ = motion_permitted;
  localization_stamp_ = now;
  if (!motion_permitted || !was_allowed) {  // a commanded motion never survives a hold
    nav_.reset();
    teleop_.reset();
    nav_stamp_.reset();
    teleop_stamp_.reset();
  }
}

Command CommandMux::select(double now, std::string * source) const
{
  auto out = [&](const std::optional<Command> & command, const char * name) {
      if (source) {*source = command ? name : "hold";}
      return command ? *command : Command{};
    };
  if (estop_ || mode_ == "hold") {return out(std::nullopt, "hold");}
  if (config_.require_localization &&
    (!localization_allowed_ || !fresh(localization_stamp_, config_.localization_timeout_s, now)))
  {
    return out(std::nullopt, "hold");
  }
  const bool has_nav = nav_ && fresh(nav_stamp_, config_.nav_timeout_s, now);
  const bool has_teleop = teleop_ && fresh(teleop_stamp_, config_.teleop_timeout_s, now);
  if (mode_ == "teleop") {return out(has_teleop ? teleop_ : std::nullopt, "teleop");}
  if (mode_ == "nav") {return out(has_nav ? nav_ : std::nullopt, "nav");}
  if (has_teleop) {return out(teleop_, "teleop");}
  return out(has_nav ? nav_ : std::nullopt, "nav");
}

Command CommandMux::step(double now, std::string * source)
{
  std::string selected;
  const Command target = select(now, &selected);
  last_output_ = target;
  last_source_ = selected;
  if (source) {*source = selected;}
  return target;
}

}  // namespace astrobot_control
