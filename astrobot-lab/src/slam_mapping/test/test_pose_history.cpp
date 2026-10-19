#include "slam_mapping/pose_history.hpp"
#include <stdexcept>
void check(bool ok) { if (!ok) throw std::runtime_error("pose history contract failed"); }
int main() {
  slam_mapping::PoseHistory h;
  h.insert(1, {0, 0, 3.13}); h.insert(1.1, {.02, 0, -3.13});
  check(!h.at(.9)); check(!h.at(1.2));
  auto mid = h.at(1.05);
  check(mid && std::abs(mid->x - .01) < 1e-10);
  check(std::abs(std::abs(mid->yaw) - 3.141592653589793) < 1e-10);
  h.insert(2, {.2, 0, 0}); check(!h.at(1.5));
  h.clear(); check(!h.at(1));
  h.insert(1, {0, 0, 0}); h.insert(1.1, {.02, 0, 0});
  double correction = 2.01 - h.at(1.05)->x;
  check(std::abs(correction + h.at(1.1)->x - 2.02) < 1e-10);
}
