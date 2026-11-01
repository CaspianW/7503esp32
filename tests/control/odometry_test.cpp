/**
 * @file odometry_test.cpp
 * @brief Host tests for chassis odometry integration.
 * @author Caspian Wu
 * @date 2026-10-19
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../../src/control/include/odometry.h"

#include <cassert>
#include <cmath>
#include <limits>

void assertSameState(const robot::OdometryState &actual,
                     const robot::OdometryState &expected) {
  assert(actual.x == expected.x);
  assert(actual.y == expected.y);
  assert(actual.yaw == expected.yaw);
  assert(actual.vx == expected.vx);
  assert(actual.vy == expected.vy);
  assert(actual.wz == expected.wz);
  assert(actual.timestamp == expected.timestamp);
}

int main() {
  robot::Odometry odometry;
  const robot::OdometryState initial{};
  assertSameState(odometry.state(), initial);
  odometry.reset();
  const robot::Twist2D body{1.0f, 0.0f, 0.0f};
  odometry.update(body, 0.0f, 2.0f, 2000);

  const robot::OdometryState &state = odometry.state();
  assert(std::fabs(state.x - 2.0f) < 0.0001f);
  assert(std::fabs(state.y) < 0.0001f);
  assert(std::fabs(state.yaw) < 0.0001f);
  assert(std::fabs(state.vx - 1.0f) < 0.0001f);
  assert(state.timestamp == 2000);

  // 转弯时用时间段中点朝向旋转车体速度，角速度以 IMU 为准。
  odometry.reset();
  const robot::Twist2D turning{1.0f, 0.0f, -3.0f};
  odometry.update(turning, 1.57079632679f, 1.0f, 3000);
  assert(std::fabs(odometry.state().x - 0.70710678f) < 0.0001f);
  assert(std::fabs(odometry.state().y - 0.70710678f) < 0.0001f);
  assert(std::fabs(odometry.state().yaw - 1.57079632679f) < 0.0001f);
  assert(std::fabs(odometry.state().wz - 1.57079632679f) < 0.0001f);

  // 不合法的周期和传感器读数不能改变位置、速度或时间戳。
  const robot::OdometryState before = odometry.state();
  odometry.update(body, 0.0f, 0.0f, 4000);
  assertSameState(odometry.state(), before);
  odometry.update(body, 0.0f, -1.0f, 4000);
  assertSameState(odometry.state(), before);
  odometry.update(body, std::numeric_limits<float>::quiet_NaN(),
                  1.0f, 4000);
  assertSameState(odometry.state(), before);
  odometry.update({std::numeric_limits<float>::infinity(), 0.0f, 0.0f},
                  0.0f, 1.0f, 4000);
  assertSameState(odometry.state(), before);
  odometry.update({0.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f},
                  0.0f, 1.0f, 4000);
  assertSameState(odometry.state(), before);
  odometry.update({0.0f, 0.0f, std::numeric_limits<float>::infinity()},
                  0.0f, 1.0f, 4000);
  assertSameState(odometry.state(), before);
  odometry.update(body, 0.0f, std::numeric_limits<float>::infinity(), 4000);
  assertSameState(odometry.state(), before);
  odometry.update(body, 0.0f, std::numeric_limits<float>::quiet_NaN(), 4000);
  assertSameState(odometry.state(), before);

  // 航向跨过 +π 后映射到 [-π, π)，不改变旋转方向。
  odometry.reset(0.0f, 0.0f, 3.0f);
  odometry.update({0.0f, 0.0f, 0.0f}, 1.0f, 1.0f, 5000);
  assert(std::fabs(odometry.state().yaw + 2.2831853f) < 0.0001f);

  // 外部设置初始位姿时也保持相同的航向范围，并清除旧速度。
  odometry.reset(2.0f, -3.0f, 4.0f);
  assert(odometry.state().x == 2.0f);
  assert(odometry.state().y == -3.0f);
  assert(std::fabs(odometry.state().yaw + 2.2831853f) < 0.0001f);
  assert(odometry.state().vx == 0.0f);
  assert(odometry.state().vy == 0.0f);
  assert(odometry.state().wz == 0.0f);
  assert(odometry.state().timestamp == 0);

  // 车体向左速度在航向为 +90° 时对应里程计 X 轴负方向。
  odometry.reset(0.0f, 0.0f, 1.57079632679f);
  odometry.update({0.0f, 1.0f, 0.0f}, 0.0f, 2.0f, 6000);
  assert(std::fabs(odometry.state().x + 2.0f) < 0.0001f);
  assert(std::fabs(odometry.state().y) < 0.0001f);
  assert(odometry.state().vy == 1.0f);

  // 有限但会导致积分溢出的读数也不能污染已有位姿。
  const robot::OdometryState beforeOverflow = odometry.state();
  const float max = std::numeric_limits<float>::max();
  odometry.update({max, 0.0f, 0.0f}, 0.0f, max, 7000);
  assertSameState(odometry.state(), beforeOverflow);
  odometry.update({0.0f, 0.0f, 0.0f}, max, max, 7000);
  assertSameState(odometry.state(), beforeOverflow);
  odometry.update({0.0f, max, 0.0f}, 0.0f, max, 7000);
  assertSameState(odometry.state(), beforeOverflow);

  // 非有限初始位姿被拒绝，后续调用仍可继续工作。
  odometry.reset(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
  assertSameState(odometry.state(), beforeOverflow);
  odometry.update({1.0f, 0.0f, 0.0f}, 0.0f, 1.0f, 8000);
  assert(std::fabs(odometry.state().x + 2.0f) < 0.0001f);
  assert(std::fabs(odometry.state().y - 1.0f) < 0.0001f);
  assert(odometry.state().timestamp == 8000);

  // +π 的边界约定为 -π。
  odometry.reset(0.0f, 0.0f, 3.14159265358979323846f);
  assert(std::fabs(odometry.state().yaw + 3.14159265358979323846f) < 0.0001f);
}
