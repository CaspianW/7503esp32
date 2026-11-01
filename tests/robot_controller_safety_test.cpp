/**
 * @file robot_controller_safety_test.cpp
 * @brief Controller integration tests for the calibrated closed-loop profile.
 * @author Caspian Wu
 * @date 2026-11-01
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include "../src/app/robot_controller.h"
#include "../src/config/robot_config.h"
#include <cassert>
#include <math.h>

static void tick(robot::RobotController &controller, uint32_t elapsed = 10000) {
  fakeNow += elapsed; controller.update();
}
static bool stopped() {
  // 底盘停机不要求升降舵机释放保持力。
  for (uint8_t i = 0; i < robot::wheelCount; ++i)
    if (fakePwm[i]) return false;
  return true;
}

int main() {
  robot::RobotController controller;
  assert(controller.begin());
  assert(!controller.begin()); // 不能重启一次性模式
  assert(stopped());
  tick(controller);
  assert(controller.speedReady() ==
         (robot::config::encoderReadEnabled || robot::config::chassisControlEnabled));
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
  robot::RobotController stoppedBeforeBegin;
  stoppedBeforeBegin.setEmergencyStop(true);
  assert(!stoppedBeforeBegin.begin());
}
