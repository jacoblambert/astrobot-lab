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

#include "astrobot_localization/child_config.hpp"

#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace astrobot_localization
{
namespace fs = std::filesystem;
using nlohmann::json;

namespace
{

json read(const fs::path & path)
{
  std::ifstream stream(path);
  if (!stream) {throw std::runtime_error("cannot read " + path.string());}
  return json::parse(stream);
}

void write(const fs::path & path, const json & value)
{
  std::ofstream stream(path);
  stream << value.dump(2);
  if (!stream) {throw std::runtime_error("cannot write " + path.string());}
}

}  // namespace

ConfigCheck check_revision_config(const fs::path & config)
{
  try {
    const json global = read(config / "config.json").at("global");
    for (const char * section : {"sub_mapping", "global_mapping"}) {
      const auto file = global.at(std::string("config_") + section).get<std::string>();
      const json settings = read(config / file).at(section);
      const auto found = settings.find("enable_optimization");
      if (found == settings.end() || !found->is_boolean() || found->get<bool>()) {
        return ConfigCheck::REVISION_CONSUMER_REQUIRED;
      }
    }
    return ConfigCheck::OK;
  } catch (const std::exception &) {
    return ConfigCheck::INVALID;
  }
}

fs::path prepare_child_config(
  const fs::path & config, const fs::path & run_dir, uint64_t epoch,
  const std::array<double, 7> & pose, const std::string & absolute_pose_topic)
{
  const fs::path directory = run_dir / std::to_string(epoch);
  fs::copy(config, directory, fs::copy_options::recursive);
  const fs::path odometry = directory / "config_odometry_cpu.json";
  json cfg = read(odometry);
  cfg.at("odometry_estimation")["init_T_world_imu"] = std::vector<double>(pose.begin(), pose.end());
  cfg.at("odometry_estimation")["init_v_world_imu"] = std::vector<double>{0.0, 0.0, 0.0};
  write(odometry, cfg);
  const fs::path ros = directory / "config_ros.json";
  json ros_cfg = read(ros);
  ros_cfg.at("abs_constraint")["rtk_topic"] = absolute_pose_topic;
  ros_cfg["revision_export"] = {{"localization_epoch", epoch}};
  write(ros, ros_cfg);
  return directory;
}

}  // namespace astrobot_localization
