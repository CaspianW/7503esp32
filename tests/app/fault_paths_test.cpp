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
#include "../../src/app/robot_controller.h"
#include "../../src/config/robot_config.h"
#include <cassert>

static void tick(robot::RobotController &c) { fakeNow += 10000; c.update(); }
static void stopped() {
  for (uint8_t i = 0; i < robot::wheelCount; ++i) assert(fakePwm[i] == 0);
}
int main() {
  robot::RobotController c;
  assert(c.begin()); tick(c); assert(c.arm());
  assert(c.commandVelocity({0.45f, 0, 0})); tick(c);
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
  assert(!c.commandVelocity({0.45f, 0, 0}));
  assert(c.arm()); assert(c.commandVelocity({0.45f, 0, 0})); tick(c);
  const uint32_t liftHolding = fakePwm[4];
  if (robot::config::liftOutputsEnabled) assert(c.liftActive() && liftHolding != 0);
  fakePcntFailure = 1; tick(c); stopped();
  assert(c.fault() == robot::RobotFault::encoder);
  assert(!c.armed());
  // 底盘编码器故障不得停止升降，且故障期间升降命令仍可用。
  if (robot::config::liftOutputsEnabled) {
    assert(c.liftActive() && fakePwm[4] == liftHolding);
    assert(c.commandLift(robot::LiftPosition::raised));
  }
  // 编码器仍异常时拒绝 arm；恢复后 arm 应清除运行时故障并重新使能。
  assert(!c.arm());
  assert(c.fault() == robot::RobotFault::encoder);
  fakePcntFailure = 0;
  assert(c.arm());
  assert(c.armed() && c.fault() == robot::RobotFault::none);
  assert(c.commandVelocity({0.45f, 0, 0})); tick(c);
  assert(c.motionAllowed() && fakePwm[0] != 0);
  c.disarm();

  // 当前映射中电气M4对应逻辑RL，编码器方向修正后正转速度应为正。
  robot::RobotController mapped;
  assert(mapped.begin()); fakePcnt[3] = -10; tick(mapped);
  assert(mapped.ticks()[3] == 10);
  assert(mapped.wheelSpeed().value[robot::rearLeft] > 0);
  assert(mapped.wheelSpeed().value[robot::frontLeft] == 0);

  // 反向编码器反馈不可在持续发送新命令时无限增大输出，但停底盘不得关停升降且可 arm 恢复。
  robot::RobotController reversed;
  assert(reversed.begin()); tick(reversed); assert(reversed.arm());
  const uint32_t reversedLiftPwm = fakePwm[4];
  for (int i = 0; i < 80 && reversed.fault() == robot::RobotFault::none; ++i) {
    if (i % 5 == 0) assert(reversed.commandVelocity({0.45f, 0, 0}));
    for (auto &count : fakePcnt) count = -1;
    tick(reversed);
  }
  assert(reversed.fault() == robot::RobotFault::feedback); stopped();
  assert(!reversed.armed());
  if (robot::config::liftOutputsEnabled)
    assert(reversed.liftActive() && fakePwm[4] == reversedLiftPwm);
  assert(reversed.arm());
  assert(reversed.armed() && reversed.fault() == robot::RobotFault::none);
  assert(reversed.commandVelocity({0.45f, 0, 0}));
  for (uint8_t i = 0; i < robot::wheelCount; ++i)
    fakePcnt[i] = static_cast<int16_t>(10 * robot::config::encoderDirection[i]);
  tick(reversed);
  assert(reversed.motionAllowed() && fakePwm[0] != 0);
}
