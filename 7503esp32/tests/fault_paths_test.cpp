/**
 * @file fault_paths_test.cpp
 * @brief Physical stop, encoder failure, wheel mapping and reversed feedback integration tests.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include <driver/pcnt.h>
#include "../src/app/robot_controller.h"
#include "../src/config/robot_config.h"
#include <cassert>

static void tick(robot::RobotController &c) { fakeNow += 10000; c.update(); }
static void stopped() {
  for (uint8_t i = 0; i < robot::wheelCount; ++i) assert(fakePwm[i] == 0);
}
int main() {
  robot::RobotController c;
  assert(c.begin()); tick(c); assert(c.arm());
  assert(c.commandVelocity({0.1f, 0, 0})); tick(c);
  if (robot::config::emergencyStopPin >= 0) {
    // 常闭急停断线或触发后必须锁存，恢复引脚不会自动解除。
    fakePin[robot::config::emergencyStopPin] =
      robot::config::emergencyStopActiveHigh ? HIGH : LOW;
    tick(c); stopped(); assert(c.emergencyStopped());
    c.setEmergencyStop(false); assert(c.emergencyStopped());
    fakePin[robot::config::emergencyStopPin] =
      robot::config::emergencyStopActiveHigh ? LOW : HIGH;
    tick(c); assert(c.emergencyStopped());
  } else {
    c.setEmergencyStop(true); stopped(); assert(c.emergencyStopped());
  }
  c.setEmergencyStop(false); assert(!c.emergencyStopped());
  assert(!c.commandVelocity({0.1f, 0, 0}));
  assert(c.arm()); assert(c.commandVelocity({0.1f, 0, 0})); tick(c);
  fakePcntFailure = 1; tick(c); stopped();
  assert(c.fault() == robot::RobotFault::encoder);
  fakePcntFailure = 0; assert(!c.arm());

  // 当前映射中电气M3对应逻辑RL，编码器方向修正后正转速度应为正。
  robot::RobotController mapped;
  assert(mapped.begin()); fakePcnt[2] = -10; tick(mapped);
  assert(mapped.ticks()[2] == 10);
  assert(mapped.wheelSpeed().value[robot::rearLeft] > 0);
  assert(mapped.wheelSpeed().value[robot::frontLeft] == 0);

  // 反向编码器反馈不可在持续发送新命令时无限增大输出。
  robot::RobotController reversed;
  assert(reversed.begin()); tick(reversed); assert(reversed.arm());
  for (int i = 0; i < 80 && reversed.fault() == robot::RobotFault::none; ++i) {
    if (i % 5 == 0) assert(reversed.commandVelocity({0.1f, 0, 0}));
    for (auto &count : fakePcnt) count = -1;
    tick(reversed);
  }
  assert(reversed.fault() == robot::RobotFault::feedback); stopped();
}
