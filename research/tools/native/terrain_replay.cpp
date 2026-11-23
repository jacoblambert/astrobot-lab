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

//
// Exact receipt-order replay of a recorded terrain run (events.jsonl + native.bin, the raw
// archive layout of the live publisher) through the nav TerrainPipeline, without ROS.
// Writes every published planner window and the final mosaics for comparison.
//
//   terrain_replay <run_dir> <out_dir> <robot.json>
//
// robot.json: {"base_from_imu": [16 row-major], "base_from_lidar_translation": [3],
//              "chassis_aabb": [6], "footprint_half_length", "footprint_half_width",
//              "footprint_padding", "mosaic_half_extent_m"}
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "astrobot_terrain/pipeline.hpp"

using astrobot_terrain::ClassGrid;
using astrobot_terrain::NativeChunk;
using astrobot_terrain::Pose;
using astrobot_terrain::Revision;
using nlohmann::json;

namespace
{

Pose matrix(const json & rows)
{
  Pose out;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out(r, c) = rows.at(r).at(c).get<double>();
    }
  }
  return out;
}

Revision revision(const json & record)
{
  Revision rev;
  rev.session = record.at("session").get<std::string>();
  rev.epoch = record.at("epoch").get<uint64_t>();
  rev.revision = record.at("revision").get<uint64_t>();
  rev.parent = record.at("parent").get<uint64_t>();
  rev.snapshot = record.at("snapshot").get<bool>();
  rev.kind = record.at("kind").get<uint8_t>();
  for (const auto & f : record.at("frames")) {
    astrobot_terrain::FramePose frame;
    frame.frame_id = f.at("frame_id").get<uint64_t>();
    frame.stamp_ns = f.at("stamp_ns").get<int64_t>();
    frame.map_from_imu = matrix(f.at("map_from_imu"));
    frame.imu_from_lidar = matrix(f.at("imu_from_lidar"));
    frame.deskew_recomputed = f.value("deskew_recomputed", false);
    rev.frames.push_back(frame);
  }
  return rev;
}

NativeChunk chunk(const json & payload, std::ifstream & raw)
{
  const auto offset = payload.at("offset").get<int64_t>();
  const auto size = payload.at("size").get<size_t>();
  if (size == 0 || size % 40) {throw std::runtime_error("raw native extent mismatch");}
  std::vector<uint8_t> data(size);
  raw.seekg(offset);
  raw.read(reinterpret_cast<char *>(data.data()), static_cast<std::streamsize>(size));
  if (!raw) {throw std::runtime_error("raw native payload truncated");}
  NativeChunk out;
  out.capture = payload.at("capture").get<std::string>();
  out.sequence = payload.at("sequence").get<uint64_t>();
  out.stamp_ns = payload.at("stamp_ns").get<int64_t>();
  const size_t n = size / 40;
  out.xyz.resize(3 * n);
  out.intensity.resize(n);
  for (size_t i = 0; i < n; ++i) {
    std::memcpy(&out.xyz[3 * i], &data[40 * i], 12);
    out.intensity[i] = data[40 * i + 16];
  }
  return out;
}

void write_grid(std::ofstream & stream, const ClassGrid & grid)
{
  stream.write(reinterpret_cast<const char *>(grid.data.data()),
      static_cast<std::streamsize>(grid.data.size()));
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "usage: terrain_replay <run_dir> <out_dir> <robot.json>\n";
    return 2;
  }
  const std::string run = argv[1], out = argv[2];
  const json robot = json::parse(std::ifstream(argv[3]));
  astrobot_terrain::PipelineConfig config;
  const auto base = robot.at("base_from_imu").get<std::vector<double>>();
  for (int i = 0; i < 16; ++i) {
    config.robot.base_from_imu(i / 4, i % 4) = base.at(i);
  }
  const auto lidar = robot.at("base_from_lidar_translation").get<std::vector<double>>();
  const auto box = robot.at("chassis_aabb").get<std::vector<double>>();
  for (int i = 0; i < 3; ++i) {
    config.robot.base_from_lidar_translation[i] = lidar.at(i);
  }
  for (int i = 0; i < 6; ++i) {
    config.robot.chassis_aabb[i] = box.at(i);
  }
  const double padding = robot.value("footprint_padding", 0.10);
  config.robot.body = {robot.value("footprint_half_length", 0.595) - padding,
    robot.value("footprint_half_width", 0.435) - padding};
  config.mosaic_half_extent_m = robot.value("mosaic_half_extent_m", 30.0);
  astrobot_terrain::TerrainPipeline pipeline(config);

  std::ifstream events(run + "/events.jsonl");
  std::ifstream raw(run + "/native.bin", std::ios::binary);
  if (!events || !raw) {
    std::cerr << "missing events.jsonl / native.bin in " << run << "\n";
    return 2;
  }
  std::ofstream windows(out + "/windows.bin", std::ios::binary);
  std::ofstream index(out + "/windows.jsonl");
  size_t lines = 0, published = 0;
  double compute_s = 0, worst_s = 0;
  std::string line;
  while (std::getline(events, line)) {
    ++lines;
    const json event = json::parse(line);
    const auto kind = event.at("kind").get<std::string>();
    const json & payload = event.at("payload");
    if (kind == "reset") {
      std::string reason;
      if (!pipeline.reset(payload.at("epoch").get<uint64_t>(),
        payload.at("minimum_ns").get<int64_t>(), &reason))
      {
        std::cerr << "reset refused: " << reason << "\n";
      }
    } else if (kind == "revision") {
      pipeline.revision(revision(payload));
    } else if (kind == "chunk") {
      pipeline.chunk(chunk(payload, raw));
    } else if (kind == "clock") {
      const auto start = std::chrono::steady_clock::now();
      auto window = pipeline.clock(payload.get<int64_t>());
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() -
        start).count();
      compute_s += elapsed;
      if (window) {
        worst_s = std::max(worst_s, elapsed);
        write_grid(windows, window->costs);
        index << json{{"stamp_ns", window->stamp_ns},
          {"origin", {window->map_from_grid(0, 3), window->map_from_grid(1, 3)}},
          {"compute_ms", elapsed * 1e3}}
          .dump()
              << "\n";
        ++published;
      }
    }
  }
  const auto status = pipeline.core().status();
  json summary = {{"events", lines},
    {"published", published},
    {"valid", status.valid},
    {"failure", status.failure},
    {"received", status.received},
    {"projected", status.projected},
    {"skipped_sequences", status.skipped_sequences},
    {"late_chunks", status.late_chunks},
    {"compute_s", compute_s},
    {"worst_projection_ms", worst_s * 1e3}};
  for (const auto & [name, mosaic] : {std::pair{"cost", pipeline.cost_mosaic()},
      std::pair{"explore", pipeline.explore_mosaic()}})
  {
    if (!mosaic) {continue;}
    std::ofstream grid(out + "/" + name + "_mosaic.bin", std::ios::binary);
    write_grid(grid, mosaic->grid());
    summary[std::string(name) + "_mosaic"] = {{"origin", {mosaic->origin_x(), mosaic->origin_y()}},
      {"width", mosaic->grid().width},
      {"height", mosaic->grid().height}};
  }
  std::ofstream(out + "/summary.json") << summary.dump(2) << "\n";
  std::cout << summary.dump() << "\n";
  return status.failure.empty() ? 0 : 1;
}
