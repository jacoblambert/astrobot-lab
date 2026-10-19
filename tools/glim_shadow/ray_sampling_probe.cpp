// Exact installed GLIM random-grid sampling stage, not odometry/registration.
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("usage: ray_sampling_probe jobs.txt");
    std::ifstream jobs(argv[1]);
    if (!jobs) throw std::runtime_error("cannot read jobs");
    std::string input, output;
    unsigned seed;
    double resolution;
    int target, threads;
    while (jobs >> input >> output >> seed >> resolution >> target >> threads) {
      std::ifstream in(input, std::ios::binary | std::ios::ate);
      if (!in || in.tellg() <= 0 || in.tellg() % (3 * sizeof(double))) throw std::runtime_error("bad XYZ input");
      const auto n = static_cast<size_t>(in.tellg()) / (3 * sizeof(double));
      if (n > 100000 || resolution <= 0 || target <= 0 || threads != 2) throw std::runtime_error("unbounded job");
      in.seekg(0);
      std::vector<Eigen::Vector4d> points(n);
      for (auto& point : points) {
        in.read(reinterpret_cast<char*>(point.data()), 3 * sizeof(double));
        point.w() = 1.0;
        if (!point.allFinite()) throw std::runtime_error("nonfinite input");
      }
      auto frame = std::make_shared<gtsam_points::PointCloudCPU>();
      frame->add_points(points);
      std::mt19937 mt(seed);
      const auto sampled = gtsam_points::randomgrid_sampling(frame, resolution, double(target) / n, mt, threads);
      std::ofstream out(output, std::ios::binary);
      size_t kept = 0;
      for (size_t i = 0; i < sampled->size(); ++i) {
        const auto& point = sampled->points[i];
        const double squared = point.head<3>().squaredNorm();
        if (point.allFinite() && squared > 0.5 * 0.5 && squared < 100.0 * 100.0) {
          out.write(reinterpret_cast<const char*>(point.data()), 3 * sizeof(double));
          ++kept;
        }
      }
      if (!out) throw std::runtime_error("output write failed");
      std::cout << input << " " << seed << " " << n << " " << kept << std::endl;
    }
    if (!jobs.eof()) throw std::runtime_error("invalid job list");
  } catch (const std::exception& e) { std::cerr << e.what() << std::endl; return 1; }
}
