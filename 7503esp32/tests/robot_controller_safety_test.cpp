/**
 * @file robot_controller_safety_test.cpp
 * @brief Controller integration tests for disabled, calibrated and one-shot bench profiles.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include "../src/app/robot_controller.h"
#include <cassert>
#include <math.h>

static void tick(robot::RobotController &controller, uint32_t elapsed = 10000) {
  fakeNow += elapsed; controller.update();
}
static bool stopped() { for (auto duty : fakePwm) if (duty) return false; return true; }

int main() {
  robot::RobotController controller;
  assert(controller.begin());
  assert(!controller.begin()); // 不能重启一次性模式
  assert(stopped());
  tick(controller);
  assert(controller.speedReady());
#ifdef CALIBRATED_FIXTURE
  assert(controller.arm());
  assert(controller.commandVelocity({0.1f, 0, 0}));
  for (int i = 0; i < 20; ++i) tick(controller);
  assert(controller.motionAllowed());
  assert(!stopped());
  // 零速度必须在接收时立即撤销 PWM，不等待控制周期。
  assert(controller.commandVelocity({0, 0, 0}));
  assert(stopped());
  for (int i = 0; i < 50; ++i) tick(controller);
  assert(!controller.armed()); // 零指令也会过期，不能残留使能
  assert(controller.arm());
  assert(controller.commandVelocity({0.1f, 0, 0}));
  for (int i = 0; i < 50; ++i) tick(controller);
  assert(!controller.motionAllowed());
  assert(!controller.armed());
  assert(stopped());
  assert(!controller.commandVelocity({0.1f, 0, 0}));
  assert(controller.arm());
  assert(controller.commandVelocity({0.1f, 0, 0}));
  tick(controller);
  controller.setEmergencyStop(true);
  assert(stopped()); assert(!controller.motionAllowed());
  assert(!controller.arm());
  assert(!controller.commandVelocity({0.1f, 0, 0}));
  controller.setEmergencyStop(false);
  assert(!controller.commandVelocity({0.1f, 0, 0}));
  assert(controller.arm());
  assert(!controller.commandVelocity({NAN, 0, 0}));
  assert(stopped()); assert(!controller.armed());
  assert(controller.arm()); assert(controller.commandVelocity({0.1f, 0, 0}));
  tick(controller, 50001);
  assert(controller.fault() == robot::RobotFault::timing);
  assert(stopped()); assert(!controller.arm());
  // 持续发送新命令也不能掩盖断开编码器的零反馈。
  robot::RobotController stalled;
  assert(stalled.begin()); tick(stalled); assert(stalled.arm());
  for (int i = 0; i < 80 && stalled.fault() == robot::RobotFault::none; ++i) {
    if (i % 5 == 0) assert(stalled.commandVelocity({0.1f, 0, 0}));
    tick(stalled);
  }
  assert(stalled.fault() == robot::RobotFault::feedback);
  assert(stopped()); assert(!stalled.arm());
#elif defined(BENCH_FIXTURE)
  for (int i = 0; i < 198; ++i) tick(controller);
  assert(stopped());
  tick(controller); assert(!stopped()); // 正好 2 秒开始
  for (int i = 0; i < 49; ++i) tick(controller);
  assert(!stopped());
  tick(controller); assert(stopped()); // 正好 2.5 秒结束
  for (int i = 0; i < 10; ++i) tick(controller);
  assert(stopped()); assert(!controller.begin());
  assert(!controller.arm());
  robot::RobotController interrupted;
  assert(interrupted.begin());
  for (int i = 0; i < 200; ++i) tick(interrupted);
  assert(!stopped());
  interrupted.setEmergencyStop(true); assert(stopped());
  interrupted.setEmergencyStop(false);
  for (int i = 0; i < 100; ++i) tick(interrupted);
  assert(stopped()); assert(!interrupted.begin());
#else
  assert(!controller.arm());
  assert(!controller.commandVelocity({0.1f, 0, 0}));
  assert(!controller.motionAllowed());
  controller.setEmergencyStop(true);
  assert(stopped()); assert(!controller.begin());
  controller.setEmergencyStop(false);
  assert(!controller.arm());
#endif
  robot::RobotController stoppedBeforeBegin;
  stoppedBeforeBegin.setEmergencyStop(true);
  assert(!stoppedBeforeBegin.begin());
}
