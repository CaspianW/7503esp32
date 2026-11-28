/**
 * @file chassis_control_test.cpp
 * @brief Chassis command limits, slew rate, immediate zero and invalid-feedback tests.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../../src/control/include/chassis_control.h"
#include <cassert>
#include <math.h>
int main() {
  robot::ChassisControl c;
  robot::ChassisParam p{0.25f, 0.2f, 0.2f, 0.6f, 0.3f, 0.5f,
                       0.05f, 0.1f, 0.15f,
                       robot::PidParam(2, 0.1f, 0, 0.2f, 1, true)};
  robot::WheelValues measured{}, duty{};
  assert(c.configure(p));
  assert(c.update({2, 3, 4}, measured, 0.01f, duty));
  for (auto speed : c.target().value) assert(fabsf(speed) <= 0.00501f);
  for (int i = 0; i < 200; ++i) assert(c.update({2, 3, 4}, measured, 0.01f, duty));
  for (auto speed : c.target().value) assert(fabsf(speed) <= 0.30001f);
  for (auto value : duty.value) assert(fabsf(value) <= 0.20001f);
  // 非零目标加入同方向前馈，但总输出仍受PID输出上限约束。
  assert(fabsf(duty.value[robot::frontLeft]) > 0.05f);
  for (uint8_t i = 0; i < robot::wheelCount; ++i) {
    if (c.target().value[i] != 0.0f)
      assert(fabsf(duty.value[i]) >= 0.15f);
  }
  robot::WheelValues solved{};
  assert(c.solveWheelTargets({0.2121f, 0.2121f, 0}, solved));
  assert(solved.value[robot::frontLeft] == 0.0f && solved.value[robot::rearRight] == 0.0f);
  assert(solved.value[robot::rearLeft] > 0.28f && solved.value[robot::rearLeft] < 0.29f);
  assert(solved.value[robot::frontRight] > 0.28f && solved.value[robot::frontRight] < 0.29f);
  assert(c.solveWheelTargets({0.212105f, 0.2121f, 0}, solved));
  assert(solved.value[robot::frontLeft] == 0.0f && solved.value[robot::rearRight] == 0.0f);
  assert(!c.solveWheelTargets({NAN, 0, 0}, solved));
  assert(c.update({0, 0, 0}, measured, 0.01f, duty));
  for (auto value : duty.value) assert(value == 0);
  assert(!c.update({NAN, 0, 0}, measured, 0.01f, duty));
  assert(!c.update({0.1f, 0, 0}, measured, 0.051f, duty));
  measured.value[2] = INFINITY;
  assert(!c.update({0.1f, 0, 0}, measured, 0.01f, duty));

  robot::ChassisControl purePi;
  robot::ChassisParam piParam{0.140f, 0.255f, 0.5f, 2.0f, 0.7f, 50.0f,
                             0.0f, 0.0f, 0.0f,
                             robot::PidParam(0.8f, 6.0f, 0.0f, 1.0f, 0.25f, true)};
  assert(purePi.configure(piParam));
  robot::WheelValues fastMeasured{{0.6f, 0.6f, 0.6f, 0.6f}};
  assert(purePi.update({0.4f, 0, 0}, fastMeasured, 0.01f, duty));
  for (auto value : duty.value) assert(value == 0.0f);
  robot::WheelValues slowMeasured{{0.2f, 0.2f, 0.2f, 0.2f}};
  assert(purePi.update({0.4f, 0, 0}, slowMeasured, 0.01f, duty));
  for (auto value : duty.value) assert(value > 0.17f);
  assert(purePi.update({0.2828f, 0.2828f, 0}, slowMeasured, 0.01f, duty));
  assert(duty.value[robot::frontLeft] == 0.0f && duty.value[robot::rearRight] == 0.0f);
  assert(duty.value[robot::rearLeft] > 0.0f && duty.value[robot::frontRight] > 0.0f);

  p.pid.kp = 0; assert(!c.configure(p));
}
