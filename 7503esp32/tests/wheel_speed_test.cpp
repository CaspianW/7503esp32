/**
 * @file wheel_speed_test.cpp
 * @brief Host tests for encoder-count to wheel-speed conversion.
 * @author Caspian Wu
 * @date 2026-10-20
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../src/control/include/wheel_speed.h"

#include <cassert>
#include <cmath>
#include <limits>

int main() {
  robot::WheelSpeed wheelSpeed;
  assert(wheelSpeed.configure(1000.0f, 0.1f));

  int64_t ticks[robot::wheelCount] = {0, 0, 0, 0};
  robot::WheelValues speed{{9.0f, 9.0f, 9.0f, 9.0f}};
  // 第一帧只有累计值，没有可计算的时间段。
  assert(!wheelSpeed.update(ticks, 0, speed));
  for (float value : speed.value) assert(value == 0.0f);

  ticks[0] = 1000;
  assert(wheelSpeed.update(ticks, 1000000, speed));
  assert(std::fabs(speed.value[0] - 0.62831853f) < 0.00001f);
  for (uint8_t i = 1; i < robot::wheelCount; ++i) {
    assert(speed.value[i] == 0.0f);
  }

  // 同一时刻重复采样无速度值，也不能推进计数基准。
  ticks[0] = 1100;
  assert(!wheelSpeed.update(ticks, 1000000, speed));
  for (float value : speed.value) assert(value == 0.0f);
  ticks[0] = 1500;
  ticks[1] = -500;
  ticks[2] = 250;
  ticks[3] = -1000;
  assert(wheelSpeed.update(ticks, 2000000, speed));
  assert(std::fabs(speed.value[0] - 0.31415927f) < 0.00001f);
  assert(std::fabs(speed.value[1] + 0.31415927f) < 0.00001f);
  assert(std::fabs(speed.value[2] - 0.15707963f) < 0.00001f);
  assert(std::fabs(speed.value[3] + 0.62831853f) < 0.00001f);

  // 编码器累计计数清零时，测速器也必须丢弃旧基准。
  wheelSpeed.reset();
  for (int64_t &value : ticks) value = 0;
  assert(!wheelSpeed.update(ticks, 3000000, speed));
  for (float value : speed.value) assert(value == 0.0f);
  ticks[0] = 100;
  assert(wheelSpeed.update(ticks, 4000000, speed));
  assert(std::fabs(speed.value[0] - 0.062831853f) < 0.00001f);

  // 用累计绝对计数的差值计算相对位移；1040 count 对应车轮一整圈。
  robot::WheelSpeed oneTurn;
  assert(oneTurn.configure(1040.0f, 0.0325f));
  int64_t turnTicks[robot::wheelCount] = {0, 0, 0, 0};
  assert(!oneTurn.update(turnTicks, 0, speed));
  turnTicks[0] = 1040;
  assert(oneTurn.update(turnTicks, 1000000, speed));
  const float wheelCircumference = 2.0f * 3.14159265358979323846f * 0.0325f;
  assert(std::fabs(speed.value[0] - wheelCircumference) < 0.00001f);
  turnTicks[0] = 2080;
  assert(oneTurn.update(turnTicks, 2000000, speed));
  assert(std::fabs(speed.value[0] - wheelCircumference) < 0.00001f);

  // micros() 的无符号时间戳回绕不改变实际的采样间隔。
  robot::WheelSpeed wrapped;
  assert(wrapped.configure(1000.0f, 0.1f));
  int64_t wrappedTicks[robot::wheelCount] = {0, 0, 0, 0};
  assert(!wrapped.update(wrappedTicks, 0xFFFFFF00U, speed));
  wrappedTicks[0] = 100;
  assert(wrapped.update(wrappedTicks, 999744U, speed));
  assert(std::fabs(speed.value[0] - 0.062831853f) < 0.00001f);

  // 中间计算溢出时四路输出均为零，且下次仍从上次有效基准计算。
  robot::WheelSpeed overflow;
  assert(overflow.configure(1.0f, 1.0e23f));
  int64_t overflowTicks[robot::wheelCount] = {0, 0, 0, 0};
  assert(!overflow.update(overflowTicks, 0, speed));
  overflowTicks[0] = 2147483647;
  assert(!overflow.update(overflowTicks, 1, speed));
  for (float value : speed.value) assert(value == 0.0f);
  overflowTicks[0] = 100;
  assert(overflow.update(overflowTicks, 1000000, speed));
  assert(std::fabs(speed.value[0] / 6.2831853e25f - 1.0f) < 0.0001f);

  // 单次增量超出 32 位范围时拒绝采样，之后仍以原有效帧为基准。
  robot::WheelSpeed hugeStep;
  assert(hugeStep.configure(1000.0f, 0.1f));
  int64_t hugeTicks[robot::wheelCount] = {0, 0, 0, 0};
  assert(!hugeStep.update(hugeTicks, 0, speed));
  hugeTicks[0] = static_cast<int64_t>(INT32_MAX) + 1;
  assert(!hugeStep.update(hugeTicks, 1000000, speed));
  for (float value : speed.value) assert(value == 0.0f);
  hugeTicks[0] = 100;
  assert(hugeStep.update(hugeTicks, 2000000, speed));
  assert(std::fabs(speed.value[0] - 0.031415927f) < 0.00001f);

  // 累计计数跨过旧 32 位边界后，仍按真实的正反向增量计算。
  robot::WheelSpeed boundary;
  assert(boundary.configure(1040.0f, 0.0325f));
  const int64_t oldMax = std::numeric_limits<int32_t>::max();
  int64_t boundaryTicks[robot::wheelCount] = {oldMax - 5, 0, 0, 0};
  assert(!boundary.update(boundaryTicks, 0, speed));
  boundaryTicks[0] = oldMax + 5;
  assert(boundary.update(boundaryTicks, 10000, speed));
  assert(std::fabs(speed.value[0] - 0.19634954f) < 0.00001f);
  boundaryTicks[0] = oldMax - 5;
  assert(boundary.update(boundaryTicks, 20000, speed));
  assert(std::fabs(speed.value[0] + 0.19634954f) < 0.00001f);

  // 极端异常输入不能使 64 位减法溢出，也不推进测速基准。
  robot::WheelSpeed extreme;
  assert(extreme.configure(1000.0f, 0.1f));
  int64_t extremeTicks[robot::wheelCount] = {
      std::numeric_limits<int64_t>::min(), 0, 0, 0};
  assert(!extreme.update(extremeTicks, 0, speed));
  extremeTicks[0] = std::numeric_limits<int64_t>::max();
  assert(!extreme.update(extremeTicks, 10000, speed));
  for (float value : speed.value) assert(value == 0.0f);
  extremeTicks[0] = std::numeric_limits<int64_t>::min() + 10;
  assert(extreme.update(extremeTicks, 20000, speed));
  assert(std::fabs(speed.value[0] - 0.31415927f) < 0.00001f);

  // 零、负数或非有限的每转计数与轮半径都不能成为配置。
  robot::WheelSpeed invalid;
  assert(!invalid.configure(0.0f, 0.1f));
  assert(!invalid.configure(-1.0f, 0.1f));
  assert(!invalid.configure(1000.0f, 0.0f));
  assert(!invalid.configure(1000.0f, -0.1f));
  assert(!invalid.configure(std::numeric_limits<float>::infinity(), 0.1f));
  assert(!invalid.configure(1000.0f,
                            std::numeric_limits<float>::quiet_NaN()));
  int64_t invalidTicks[robot::wheelCount] = {0, 0, 0, 0};
  robot::WheelValues invalidSpeed{{9.0f, 9.0f, 9.0f, 9.0f}};
  assert(!invalid.update(invalidTicks, 0, invalidSpeed));
  for (float value : invalidSpeed.value) assert(value == 0.0f);
  invalidTicks[0] = 100;
  assert(!invalid.update(invalidTicks, 1000000, invalidSpeed));
  for (float value : invalidSpeed.value) assert(value == 0.0f);
}
