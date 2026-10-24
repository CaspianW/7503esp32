/**
 * @file chassis_control_test.cpp
 * @brief Chassis command limits, slew rate, immediate zero and invalid-feedback tests.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../src/control/include/chassis_control.h"
#include <cassert>
#include <math.h>
int main() {
  robot::ChassisControl c;
  robot::ChassisParam p{0.25f, 0.2f, 0.2f, 0.6f, 0.3f, 0.5f,
                       robot::PidParam(2, 0.1f, 0, 0.2f, 1, true)};
  robot::WheelValues measured{}, duty{};
  assert(c.configure(p));
  assert(c.update({2, 3, 4}, measured, 0.01f, duty));
  for (auto speed : c.target().value) assert(fabsf(speed) <= 0.00501f);
  for (int i = 0; i < 200; ++i) assert(c.update({2, 3, 4}, measured, 0.01f, duty));
  for (auto speed : c.target().value) assert(fabsf(speed) <= 0.30001f);
  for (auto value : duty.value) assert(fabsf(value) <= 0.20001f);
  assert(c.update({0, 0, 0}, measured, 0.01f, duty));
  for (auto value : duty.value) assert(value == 0);
  assert(!c.update({NAN, 0, 0}, measured, 0.01f, duty));
  assert(!c.update({0.1f, 0, 0}, measured, 0.051f, duty));
  measured.value[2] = INFINITY;
  assert(!c.update({0.1f, 0, 0}, measured, 0.01f, duty));
  p.pid.kp = 0; assert(!c.configure(p));
}
