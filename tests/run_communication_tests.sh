#!/usr/bin/env bash
# Host-only regressions for ESP32 C++ code. No ROS/Pi runtime is needed.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
test_build_dir=$(mktemp -d "${TMPDIR:-/tmp}/esp32-communication.XXXXXX")
trap 'rm -rf -- "$test_build_dir"' EXIT
cd -- "$repo_dir"
sources=(
  src/app/*.cpp src/communication/*.cpp src/control/src/*.cpp
  src/drivers/src/encoder.cpp src/drivers/src/motor_driver.cpp
  src/drivers/src/servo_driver.cpp tests/fakes/hardware.cpp tests/fakes/imu.cpp
)
for name in serial_protocol serial_telemetry serial_console serial_console_telemetry robot_controller_safety lift_control safety; do
  "${CXX:-g++}" -std=c++11 -Wall -Wextra -Werror -Itests/fakes \
    "tests/${name}_test.cpp" "${sources[@]}" -o "$test_build_dir/$name"
  "$test_build_dir/$name"
  echo "$name: PASS"
done
