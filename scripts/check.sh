#!/bin/bash
# Build and test the release packages, then lint them (inside astrobot-dev):
#   unit tests (gtest / pytest) through colcon test;
#   C++: ament_cpplint, ament_uncrustify (ROS 2 Jazzy configs), ament_copyright, ament_xmllint,
#        ament_lint_cmake, and clang-tidy (Space ROS ament_clang_tidy, default ament config);
#   Python: ament_flake8 / ament_pep257 on the launch files (enforced) and the sim/tooling
#           packages (reported only).
# The Space ROS linter suite of the base image (SARIF output, Cobra, IKOS and its Humble-era
# cpplint/uncrustify configs) is not usable against this Jazzy toolchain yet: release 0.2.
#
# Usage: scripts/check.sh            (from the host: runs inside the astrobot-dev container)
# NOTE: no `set -u` (ROS setup scripts reference unbound variables).
set -o pipefail
cd "$(dirname "$0")/.." || exit 2

if [ ! -f /etc/ros_setup.sh ]; then
  exec docker exec astrobot-dev bash -lc "cd /workspace/astrobot-lab && scripts/check.sh"
fi
CORE_CPP=(astrobot_terrain astrobot_localization astrobot_glim_extensions astrobot_control
  astrobot_exploration)
CORE=(astrobot_interfaces "${CORE_CPP[@]}" astrobot_bringup)
TOOLING=(astrobot_sim astrobot_eval astrobot_ui_backend)

# shellcheck disable=SC1091
source /etc/ros_setup.sh >/dev/null 2>&1
cd astrobot-lab || exit 2
colcon build --symlink-install --packages-select "${CORE[@]}" "${TOOLING[@]}" \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON || exit 1
# shellcheck disable=SC1091
source install/setup.bash

status=0
step() {
  local name=$1; shift
  if "$@" >/tmp/check_step.log 2>&1; then
    echo "  ok    $name"
  else
    echo "  FAIL  $name"; sed 's/^/        /' /tmp/check_step.log | tail -40; status=1
  fi
}

echo "== unit tests"
for package in "${CORE[@]}" "${TOOLING[@]}"; do
  rm -rf "build/$package/test_results" "build/$package/pytest.xml" "build/$package/Testing"
done
colcon test --packages-select "${CORE[@]}" "${TOOLING[@]}" >/dev/null 2>&1
for package in "${CORE[@]}" "${TOOLING[@]}"; do
  # gtest/pytest write test_results/ or pytest.xml; plain CTest tests write Testing/.
  [ -d "build/$package/test_results" ] || [ -f "build/$package/pytest.xml" ] \
    || [ -d "build/$package/Testing" ] || continue
  step "$package" colcon test-result --test-result-base "build/$package"
done

echo "== C++ lint"
for package in "${CORE_CPP[@]}"; do
  step "$package cpplint" ament_cpplint "src/$package"
  step "$package uncrustify" ament_uncrustify "src/$package"
  step "$package copyright" ament_copyright "src/$package"
  step "$package lint_cmake" ament_lint_cmake "src/$package/CMakeLists.txt"
  step "$package xmllint" ament_xmllint "src/$package"
  step "$package clang-tidy" /opt/ros/spaceros/bin/ament_clang_tidy \
    "$PWD/build/$package/compile_commands.json"
done
step "astrobot_interfaces xmllint" ament_xmllint src/astrobot_interfaces
step "astrobot_bringup xmllint" ament_xmllint src/astrobot_bringup

echo "== Python lint"
step "astrobot_bringup flake8" ament_flake8 src/astrobot_bringup/launch
step "astrobot_bringup pep257" ament_pep257 src/astrobot_bringup/launch
step "astrobot_bringup copyright" ament_copyright src/astrobot_bringup
for package in "${TOOLING[@]}"; do
  count=$(ament_flake8 "src/$package" 2>/dev/null | grep -cE ': [A-Z][0-9]+ ')
  echo "  info  $package flake8: $count findings (not enforced)"
done

[ "$status" = 0 ] && echo "CHECK PASSED" || echo "CHECK FAILED"
exit $status
