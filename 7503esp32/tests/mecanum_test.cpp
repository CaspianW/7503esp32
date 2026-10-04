/**
 * @file mecanum_test.cpp
 * @brief Host tests for mecanum forward and inverse kinematics.
 * @author Caspian Wu
 * @date 2026-10-04
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../src/control/include/mecanum.h"

#include <cassert>
#include <cmath>

int main() {
  robot::MecanumKinematics kinematics;
  assert(kinematics.configure(0.4f, 0.3f));

  robot::Twist2D command{};
  command.vx = 1.0f;
  command.vy = 0.2f;
  command.wz = 1.0f;
  robot::WheelValues wheel{};
  assert(kinematics.inverse(command, wheel));
  assert(std::fabs(wheel.value[robot::frontLeft] - 0.45f) < 0.0001f);
  assert(std::fabs(wheel.value[robot::rearLeft] - 0.85f) < 0.0001f);
  assert(std::fabs(wheel.value[robot::frontRight] - 1.55f) < 0.0001f);
  assert(std::fabs(wheel.value[robot::rearRight] - 1.15f) < 0.0001f);

  robot::Twist2D recovered{};
  assert(kinematics.forward(wheel, recovered));
  assert(std::fabs(recovered.vx - 1.0f) < 0.0001f);
  assert(std::fabs(recovered.vy - 0.2f) < 0.0001f);
  assert(std::fabs(recovered.wz - 1.0f) < 0.0001f);
}
