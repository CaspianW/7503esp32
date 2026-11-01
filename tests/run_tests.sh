#!/usr/bin/env bash
# Host regressions only; build products live in a temporary directory.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
test_build_dir=$(mktemp -d "${TMPDIR:-/tmp}/esp32-tests.XXXXXX")
trap 'rm -rf -- "$test_build_dir"' EXIT
cd -- "$repo_dir"
case "${1:-all}" in
  all) groups=(communication control drivers app) ;;
  communication|control|drivers|app) groups=("$1") ;;
  *) echo "Usage: bash tests/run_tests.sh [all|communication|control|drivers|app]" >&2; exit 2 ;;
esac
compiler=${CXX:-g++}
flags=(-std=c++11 -Wall -Wextra -Werror -Itests -Itests/fakes)
sources=(
  src/app/*.cpp src/communication/*.cpp src/control/src/*.cpp
  src/drivers/src/encoder.cpp src/drivers/src/motor_driver.cpp
  src/drivers/src/servo_driver.cpp tests/fakes/hardware.cpp
)
objects=()
for source in "${sources[@]}"; do
  object="$test_build_dir/${source//\//_}.o"
  "$compiler" "${flags[@]}" -c "$source" -o "$object"
  objects+=("$object")
done
# Most integration tests use a stationary fake IMU. Its driver test uses the real
# driver with only the device registers replaced, so driver regressions are caught.
"$compiler" "${flags[@]}" -c tests/fakes/imu.cpp -o "$test_build_dir/imu_fake.o"
"$compiler" "${flags[@]}" -c src/drivers/src/imu.cpp -o "$test_build_dir/imu_driver.o"
passed=0
for group in "${groups[@]}"; do
  for source in tests/"$group"/*_test.cpp; do
    name=${source##*/}
    binary="$test_build_dir/${name%.cpp}"
    imu_object="$test_build_dir/imu_fake.o"
    if [[ "$source" == tests/drivers/imu_driver_test.cpp ]]; then
      imu_object="$test_build_dir/imu_driver.o"
    fi
    "$compiler" "${flags[@]}" "$source" "${objects[@]}" "$imu_object" -o "$binary"
    "$binary"
    echo "$group/${name%_test.cpp}: PASS"
    passed=$((passed + 1))
  done
done
"$compiler" "${flags[@]}" -x c++ -c 7503esp32.ino -o "$test_build_dir/sketch.o"
echo "sketch entry (host compile): PASS"
echo "$passed test programs passed."
