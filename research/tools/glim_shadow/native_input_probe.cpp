// Installed GLIM ROS conversion/time/preprocessing integration, NOT odometry.
#define GLIM_ROS2
#include <filesystem>
#include <fstream>
#include <cstring>
#include <nlohmann/json.hpp>
#include <glim/util/config.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>

int main(int argc, char** argv) {
  try {
    if (argc != 5 || std::filesystem::exists(argv[4])) throw std::runtime_error("usage: config jobs mode output");
    glim::GlobalConfig::instance(argv[1]);
    glim::CloudPreprocessorParams params;
    if (std::string(argv[3]) == "global-shutter-negative-control") params.global_shutter = true;
    else if (std::string(argv[3]) != "timed" || params.global_shutter) throw std::runtime_error("timed profile required");
    glim::CloudPreprocessor preprocess(params);
    std::ifstream input(argv[2]);
    nlohmann::json jobs; input >> jobs;
    std::ofstream output(argv[4]);
    for (const auto& job : jobs) {
      // Independent settled windows; do not present teleported views as motion.
      glim::TimeKeeper time;
      sensor_msgs::msg::PointCloud2 msg;
      const auto stamp = job.at("stamp_ns").get<int64_t>();
      msg.header.stamp.sec = stamp / 1000000000;
      msg.header.stamp.nanosec = stamp % 1000000000;
      msg.header.frame_id = "vlp16";
      msg.height = 1; msg.width = job.at("points").get<uint32_t>();
      msg.point_step = 32; msg.row_step = msg.width * 32;
      msg.is_dense = true; msg.is_bigendian = false;
      for (const auto& spec : jobs.at(0).at("fields")) {
        sensor_msgs::msg::PointField field;
        field.name = spec[0].get<std::string>(); field.offset = spec[1];
        field.datatype = spec[2]; field.count = spec[3]; msg.fields.push_back(field);
      }
      if (!msg.width || msg.width > 100000) throw std::runtime_error("unbounded cloud");
      std::ifstream data(job.at("file").get<std::string>(), std::ios::binary | std::ios::ate);
      if (!data || data.tellg() != msg.row_step) throw std::runtime_error("bad cloud bytes");
      data.seekg(0); msg.data.resize(msg.row_step);
      data.read(reinterpret_cast<char*>(msg.data.data()), msg.data.size());
      auto raw = glim::extract_raw_points(msg, "intensity", "ring");
      if (!raw || raw->size() != msg.width || raw->times.size() != msg.width) throw std::runtime_error("converter lost data/times");
      const auto before_stamp = raw->stamp;
      const auto before_times = raw->times;
      for (size_t i = 0; i < raw->size(); ++i) {
        float xyz[3]; double t;
        std::memcpy(xyz, msg.data.data() + 32*i, 12);
        std::memcpy(&t, msg.data.data() + 32*i + 16, 8);
        if ((raw->points[i].head<3>() - Eigen::Vector3d(xyz[0], xyz[1], xyz[2])).norm() != 0 || raw->times[i] != t)
          throw std::runtime_error("converter changed sensor-local geometry/time");
      }
      if (!time.process(raw) || raw->stamp != before_stamp || raw->times != before_times)
        throw std::runtime_error("TimeKeeper changed native acquisition times");
      auto frame = preprocess.preprocess(raw);
      const double expected_end = job.at("end_ns").get<int64_t>() / 1e9;
      if (frame->points.empty() || std::abs(frame->scan_end_time - expected_end) > 1e-8 || frame->scan_end_time <= raw->stamp)
        throw std::runtime_error("preprocessing erased/changed native acquisition extent");
      output << nlohmann::json({{"file", job.at("file")}, {"input_points", raw->size()},
        {"output_points", frame->points.size()}, {"stamp", frame->stamp},
        {"scan_end", frame->scan_end_time}, {"passed", true}}).dump() << '\n';
    }
    if (!output) throw std::runtime_error("output write failed");
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
