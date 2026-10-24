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
#include <cassert>

static void tick(robot::RobotController &c) { fakeNow += 10000; c.update(); }
static void stopped() { for (auto duty : fakePwm) assert(duty == 0); }
int main() {
  robot::RobotController c;
  assert(c.begin()); tick(c); assert(c.arm());
  assert(c.commandVelocity({0.1f, 0, 0})); tick(c);
  // 常闭急停断线或触发后的 HIGH 必须锁存。
  fakePin[17] = HIGH; tick(c); stopped();
  assert(c.emergencyStopped());
  c.setEmergencyStop(false); assert(c.emergencyStopped());
  fakePin[17] = LOW; tick(c); assert(c.emergencyStopped());
  c.setEmergencyStop(false); assert(!c.emergencyStopped());
  assert(!c.commandVelocity({0.1f, 0, 0}));
  assert(c.arm()); assert(c.commandVelocity({0.1f, 0, 0})); tick(c);
  fakePcntFailure = 1; tick(c); stopped();
  assert(c.fault() == robot::RobotFault::encoder);
  fakePcntFailure = 0; assert(!c.arm());

  // 使用非恒等映射，电气 M3 的计数应出现在逻辑 FL。
  robot::RobotController mapped;
  assert(mapped.begin()); fakePcnt[2] = 10; tick(mapped);
  assert(mapped.ticks()[2] == 10);
  assert(mapped.wheelSpeed().value[robot::frontLeft] > 0);
  assert(mapped.wheelSpeed().value[robot::rearLeft] == 0);

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
