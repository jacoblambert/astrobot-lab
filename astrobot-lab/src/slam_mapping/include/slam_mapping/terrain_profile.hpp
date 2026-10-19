#pragma once

// Explicit diagnostic builds only. Stage timings are wall time (including
// OpenMP barriers), never resource acceptance. No state or output in normal
// builds; the mapper's geometry and ordered evidence operations are unchanged.
#ifdef TERRAIN_PROFILE
#include <chrono>
#include <cstdio>
#include <utility>
#include <vector>
namespace slam_mapping {
class TerrainProjectionProfile {
 public:
  explicit TerrainProjectionProfile(double stamp): stamp_(stamp), last_(Clock::now()) {}
  void stage(const char* name) {
    const auto now=Clock::now();
    stages_.emplace_back(name,std::chrono::duration<double,std::milli>(now-last_).count());
    last_=now;
  }
  ~TerrainProjectionProfile() {
    stage("clearance_finish");
    std::fprintf(stderr,"{\"terrain_projection_profile\":%.17g,\"milliseconds\":{",stamp_);
    for (size_t i=0;i<stages_.size();++i)
      std::fprintf(stderr,"%s\"%s\":%.9g",i ? "," : "",stages_[i].first,stages_[i].second);
    std::fprintf(stderr,"}}\n");
  }
 private:
  using Clock=std::chrono::steady_clock;
  double stamp_;
  Clock::time_point last_;
  std::vector<std::pair<const char*,double>> stages_;
};
}
#define TERRAIN_PROFILE_BEGIN(stamp) TerrainProjectionProfile terrain_profile(stamp)
#define TERRAIN_PROFILE_STAGE(name) terrain_profile.stage(name)
#else
#define TERRAIN_PROFILE_BEGIN(stamp) ((void)0)
#define TERRAIN_PROFILE_STAGE(name) ((void)0)
#endif
