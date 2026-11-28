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
                       robot::PidParam(2, 0.1f, 0, 0.2f, 1, true),
                       0.0f, 0U, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U};
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
                             robot::PidParam(0.8f, 6.0f, 0.0f, 1.0f, 0.25f, true),
                             0.0f, 0U, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U};
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

  // 静止起转脉冲 + 零速主动反接制动测试（低速/原地旋转秒起，零速立刻反接刹停无回弹）。
  robot::ChassisControl snappy;
  robot::ChassisParam sp{0.140f, 0.255f, 0.5f, 2.0f, 0.7f, 50.0f,
                         0.40f, 0.0f, 0.0f,
                         robot::PidParam(0.6f, 5.0f, 0.0f, 1.0f, 0.25f, true),
                         0.72f, 80U, 0.03f, 0.25f, 1.4f, 0.65f, 0.03f, 250U};
  assert(snappy.configure(sp));
  robot::WheelValues zeroMeas{}, moveMeas{{0.05f, 0.05f, 0.05f, 0.05f}};
  // 1) 低速原地旋转静止起步 -> 立刻触发 0.72 脉冲，一旦测到转动立刻回落到纯前馈+PI。
  assert(snappy.update({0.0f, 0.0f, 0.70f}, zeroMeas, 0.01f, duty));
  assert(fabsf(duty.value[robot::frontLeft] + 0.72f) < 1e-4f);
  assert(fabsf(duty.value[robot::frontRight] - 0.72f) < 1e-4f);
  robot::WheelValues spinMeas{{-0.05f, -0.05f, 0.05f, 0.05f}};
  assert(snappy.update({0.0f, 0.0f, 0.70f}, spinMeas, 0.01f, duty));
  assert(fabsf(duty.value[robot::frontRight]) < 0.55f && duty.value[robot::frontRight] > 0.40f);
  // 2) 未检测到转动时脉冲最多持续 80 ms（8 周期），随后回落到正常前馈+PI。
  snappy.reset();
  for (int i = 0; i < 8; ++i) {
    assert(snappy.update({0.10f, 0.0f, 0.0f}, zeroMeas, 0.01f, duty));
    for (auto v : duty.value) assert(fabsf(v - 0.72f) < 1e-4f);
  }
  assert(snappy.update({0.10f, 0.0f, 0.0f}, zeroMeas, 0.01f, duty));
  for (auto v : duty.value) assert(v < 0.55f && v > 0.40f);
  // 3) 滚动中收到零速 -> 立即施加反方向主动制动，降速到 <= 0.03 或过零即切断为 0 无回弹。
  robot::WheelValues rollMeas{{0.40f, 0.40f, 0.40f, 0.40f}};
  assert(snappy.update({0.0f, 0.0f, 0.0f}, rollMeas, 0.01f, duty));
  for (auto v : duty.value) assert(v <= -0.64f && v >= -0.651f);
  robot::WheelValues slowRoll{{0.10f, 0.10f, 0.10f, 0.10f}};
  assert(snappy.update({0.0f, 0.0f, 0.0f}, slowRoll, 0.01f, duty));
  for (auto v : duty.value) assert(fabsf(v + 0.39f) < 1e-3f);
  robot::WheelValues stoppedRoll{{0.02f, 0.02f, -0.01f, 0.0f}};
  assert(snappy.update({0.0f, 0.0f, 0.0f}, stoppedRoll, 0.01f, duty));
  for (auto v : duty.value) assert(v == 0.0f);
  // 刹停锁存后即使出现噪声抖动也不再反向输出。
  assert(snappy.update({0.0f, 0.0f, 0.0f}, moveMeas, 0.01f, duty));
  for (auto v : duty.value) assert(v == 0.0f);
  // 4) 制动最长不超过 250 ms（25 周期）。
  assert(snappy.update({0.40f, 0.0f, 0.0f}, rollMeas, 0.01f, duty));
  for (int i = 0; i < 25; ++i) {
    assert(snappy.update({0.0f, 0.0f, 0.0f}, rollMeas, 0.01f, duty));
    for (auto v : duty.value) assert(v < -0.24f);
  }
  assert(snappy.update({0.0f, 0.0f, 0.0f}, rollMeas, 0.01f, duty));
  for (auto v : duty.value) assert(v == 0.0f);

  p.pid.kp = 0; assert(!c.configure(p));
}
