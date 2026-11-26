/**
 * @file robot_controller_safety_test.cpp
 * @brief Controller integration tests for the calibrated closed-loop profile.
 * @author Caspian Wu
 * @date 2026-11-01
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include <driver/pcnt.h>
#include "../../src/app/robot_controller.h"
#include "../../src/config/robot_config.h"
#include <cassert>
#include <math.h>

static void tick(robot::RobotController &controller, uint32_t elapsed = 10000) {
  fakeNow += elapsed; controller.update();
}
static void spinForward() {
  for (uint8_t i = 0; i < robot::wheelCount; ++i)
    fakePcnt[i] = static_cast<int16_t>(10 * robot::config::encoderDirection[i]);
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
  // 上电即默认使能待命（PWM 为 0），无需等待首帧速度样本即可接受 arm() 或 0x01。
  assert(controller.armed() && stopped());
  assert(controller.arm());
  assert(controller.commandVelocity({0.10f, 0, 0}));
  tick(controller);
  assert(controller.speedReady() ==
         (robot::config::encoderReadEnabled || robot::config::chassisControlEnabled));
  assert(controller.armed() && controller.motionAllowed() && !stopped());
  assert(controller.fault() == robot::RobotFault::none);
  assert(controller.commandVelocity({0, 0, 0}));
  tick(controller);
  assert(controller.armed() && stopped() && controller.fault() == robot::RobotFault::none);
  assert(controller.commandVelocity({0, 0.10f, 0}));
  tick(controller);
  assert(controller.armed() && controller.motionAllowed() && !stopped());
  assert(controller.commandVelocity({0, 0, 0.50f}));
  tick(controller);
  assert(controller.armed() && controller.motionAllowed() && !stopped());
  if (robot::config::liftOutputsEnabled)
    assert(controller.commandLift(robot::LiftPosition::raised));
  assert(controller.commandVelocity({0.45f, 0, 0}));
  for (int i = 0; i < 20; ++i) tick(controller);
  assert(controller.motionAllowed());
  assert(!stopped());
  // 零速度必须在接收时立即撤销 PWM，不等待控制周期。
  assert(controller.commandVelocity({0, 0, 0}));
  assert(stopped());
  // 1000 ms 超时仅停止车轮并清空目标，不得自动解除 armed_，也不影响升降。
  for (int i = 0; i < 99; ++i) tick(controller);
  assert(controller.armed() && controller.motionAllowed() && stopped());
  tick(controller);
  assert(controller.armed() && !controller.motionAllowed() && stopped());
  if (robot::config::liftOutputsEnabled)
    assert(controller.commandLift(robot::LiftPosition::lowered));
  // 超时后无需重新 arm，下一帧速度命令立即恢复运动。
  assert(controller.commandVelocity({0.45f, 0, 0}));
  for (int i = 0; i < 99; ++i) { spinForward(); tick(controller); }
  assert(controller.armed() && controller.motionAllowed() && !stopped());
  spinForward(); tick(controller);
  assert(controller.armed());
  assert(!controller.motionAllowed());
  assert(stopped());
  for (float v : controller.wheelTarget().value) assert(v == 0.0f);
  assert(controller.fault() == robot::RobotFault::none);
  assert(controller.commandVelocity({0.45f, 0, 0}));
  spinForward(); tick(controller);
  assert(controller.armed() && controller.motionAllowed() && !stopped());

  // 显式 disarm 仍正常解除使能并保留升降。
  const uint32_t holdingLift = fakePwm[4];
  controller.disarm();
  assert(!controller.armed() && !controller.motionAllowed() && stopped());
  if (robot::config::liftOutputsEnabled)
    assert(controller.liftActive() && fakePwm[4] == holdingLift);
  assert(!controller.commandVelocity({0.45f, 0, 0}));
  assert(controller.commandVelocity({0, 0, 0}) && !controller.armed() && stopped());

  // 急停立即停底盘；解除急停后需 arm，或者直接显式 arm() 也可解除软件急停并恢复控制。
  assert(controller.arm());
  assert(controller.commandVelocity({0.45f, 0, 0}));
  spinForward(); tick(controller);
  controller.setEmergencyStop(true);
  assert(stopped()); assert(!controller.motionAllowed());
  assert(!controller.commandVelocity({0.45f, 0, 0}));
  controller.setEmergencyStop(false);
  assert(!controller.commandVelocity({0.45f, 0, 0}));
  assert(controller.arm());
  controller.setEmergencyStop(true);
  assert(stopped() && controller.emergencyStopped());
  assert(controller.arm());
  assert(!controller.emergencyStopped() && controller.armed());

  // NaN / Inf 速度指令只安全停轮并返回失败，不解除使能、不锁存故障、不影响升降。
  assert(controller.commandVelocity({0.45f, 0, 0}));
  spinForward(); tick(controller);
  assert(!stopped());
  assert(!controller.commandVelocity({NAN, 0, 0}));
  assert(stopped() && controller.armed() && !controller.motionAllowed());
  assert(controller.fault() == robot::RobotFault::none);
  if (robot::config::liftOutputsEnabled)
    assert(controller.liftActive() && fakePwm[4] == holdingLift);
  assert(controller.commandVelocity({0.45f, 0, 0}));
  spinForward(); tick(controller);
  assert(!stopped());
  assert(!controller.commandVelocity({0, INFINITY, 0}));
  assert(stopped() && controller.armed() && !controller.motionAllowed());
  assert(controller.fault() == robot::RobotFault::none);

  // 瞬时时序抖动（> maxControlGap）跳过本周期输出，不锁死整车也不影响升降，下周期自动恢复。
  assert(controller.commandVelocity({0.45f, 0, 0}));
  spinForward(); tick(controller);
  assert(!stopped());
  tick(controller, 50001);
  assert(controller.fault() == robot::RobotFault::none);
  assert(controller.armed() && stopped());
  if (robot::config::liftOutputsEnabled)
    assert(controller.liftActive() && fakePwm[4] == holdingLift);
  spinForward(); tick(controller);
  assert(controller.fault() == robot::RobotFault::none);
  assert(controller.armed() && controller.motionAllowed() && !stopped());

  // 单周期反馈尖峰（超 3x maxWheel）仅跳过本周期输出，不锁存故障也不解除使能。
  fakePcnt[0] = 500;
  tick(controller);
  assert(controller.fault() == robot::RobotFault::none);
  assert(controller.armed() && stopped());
  spinForward(); tick(controller);
  assert(controller.fault() == robot::RobotFault::none);
  assert(controller.armed() && controller.motionAllowed() && !stopped());

  // 瞬时 IMU 读失败仅降级为 imuReady()==false 并退避轮询，不触发故障，不影响底盘驱动、arm 或升降。
  assert(controller.imuReady());
  fakeImuReadStatus = -1;
  for (int i = 0; i < 55; ++i) {
    if (i % 5 == 0) assert(controller.commandVelocity({0.45f, 0, 0}));
    spinForward(); tick(controller);
  }
  assert(!controller.imuReady());
  assert(controller.fault() == robot::RobotFault::none);
  assert(controller.armed() && controller.motionAllowed() && !stopped());
  if (robot::config::liftOutputsEnabled)
    assert(controller.commandLift(robot::LiftPosition::raised));
  assert(controller.arm());
  fakeImuReadStatus = 0;
  for (int i = 0; i < 20; ++i) tick(controller);
  assert(controller.imuReady());

  // 持续堵转/零反馈不得锁死或解除使能：PID 自动限幅，且随时可反向或变向脱困。
  robot::RobotController stalled;
  assert(stalled.begin()); tick(stalled);
  const uint32_t stalledLiftPwm = fakePwm[4];
  for (int i = 0; i < 80; ++i) {
    if (i % 5 == 0) assert(stalled.commandVelocity({0.45f, 0, 0}));
    tick(stalled);
  }
  assert(stalled.fault() == robot::RobotFault::none);
  assert(stalled.armed() && stalled.motionAllowed() && !stopped());
  if (robot::config::liftOutputsEnabled) {
    assert(stalled.liftActive() && fakePwm[4] == stalledLiftPwm);
    assert(stalled.commandLift(robot::LiftPosition::raised));
  }
  assert(stalled.commandVelocity({-0.30f, 0, 0}));
  spinForward(); tick(stalled);
  assert(stalled.motionAllowed() && !stopped());

  // 升降通道故障与底盘解耦：升降初始化失败不得阻断底盘使能和驱动，修复后可重新控制升降。
  fakeSetupFailure = 4;
  robot::RobotController liftBroken;
  assert(liftBroken.begin());
  assert(liftBroken.initialized());
  if (robot::config::liftOutputsEnabled)
    assert(liftBroken.fault() == robot::RobotFault::lift && !liftBroken.liftActive());
  tick(liftBroken);
  assert(liftBroken.arm());
  assert(liftBroken.commandVelocity({0.45f, 0, 0}));
  spinForward(); tick(liftBroken);
  assert(liftBroken.armed() && liftBroken.motionAllowed() && !stopped());
  if (robot::config::liftOutputsEnabled) {
    assert(!liftBroken.commandLift(robot::LiftPosition::raised));
    assert(liftBroken.armed() && liftBroken.motionAllowed() && !stopped());
    fakeSetupFailure = -1;
    assert(liftBroken.commandLift(robot::LiftPosition::raised));
    assert(liftBroken.liftActive() && fakePwm[4] != 0);
    assert(liftBroken.fault() == robot::RobotFault::none);
  }
  fakeSetupFailure = -1;

  // 真正的启动硬件初始化失败（不可恢复）仍必须拒绝 arm。
  fakeSetupFailure = 0;
  robot::RobotController bootBroken;
  assert(!bootBroken.begin());
  assert(!bootBroken.initialized() && bootBroken.fault() == robot::RobotFault::motor);
  assert(!bootBroken.arm());
  fakeSetupFailure = -1;

  robot::RobotController stoppedBeforeBegin;
  stoppedBeforeBegin.setEmergencyStop(true);
  assert(!stoppedBeforeBegin.begin());
}
