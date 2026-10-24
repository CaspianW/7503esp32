/**
 * @file lift_control_test.cpp
 * @brief Lift arming, endpoint output, watchdog, emergency stop and serial dispatch tests.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include "../src/app/robot_controller.h"
#include "../src/communication/serial_console.h"
#include <cassert>

static void tick(robot::RobotController &c) { fakeNow += 10000; c.update(); }
int main() {
  robot::RobotController c;
  assert(c.begin());
  tick(c);
  assert(!c.commandLift(robot::LiftPosition::raised));
  assert(!c.keepLiftAlive());
  assert(fakePwm[4] == 0);
#ifdef LIFT_FIXTURE
  assert(c.liftConfigured());
  assert(c.armLift());
  assert(c.liftArmed() && !c.liftActive());
  assert(fakePwm[4] == 0); // 使能不自动跳到旧目标
  assert(c.commandLift(robot::LiftPosition::raised));
  assert(c.liftPulse() == 2000 && fakePwm[4] == 1638);
  assert(c.armLift()); // 重复使能不释放保持PWM
  assert(fakePwm[4] == 1638);
  for (int i = 0; i < 49; ++i) tick(c);
  assert(c.liftActive());
  tick(c); // 500ms，恰好超时
  assert(!c.liftArmed() && !c.liftActive());
  assert(c.liftTarget() == 2 && c.liftPulse() == 0 && fakePwm[4] == 0);
  assert(!c.keepLiftAlive());
  assert(!c.commandLift(robot::LiftPosition::raised));
  assert(c.armLift()); assert(fakePwm[4] == 0);
  assert(c.commandLift(robot::LiftPosition::lowered));
  assert(c.liftPulse() == 1000 && fakePwm[4] == 819);
  for (int i = 0; i < 100; ++i) {
    if (i % 10 == 0) assert(c.keepLiftAlive());
    tick(c);
  }
  assert(c.liftActive());
  assert(!c.motionAllowed()); // 升降心跳不能使能底盘
  if (c.arm()) { // 同时启用底盘的临时测试配置
    assert(c.liftActive() && fakePwm[4] == 819); // 使能底盘不撤升降保持力
    assert(c.commandVelocity({0.03f, 0, 0}));
    for (int i = 0; i < 50; ++i) {
      assert(c.keepLiftAlive()); tick(c);
    }
    assert(!c.armed() && !c.motionAllowed());
    assert(c.liftActive()); // 独立升降心跳不会给底盘速度续命
  }
  c.stop(); assert(!c.liftArmed() && fakePwm[4] == 0);

  robot::SerialConsole console;
  console.begin();
  Serial.input = "lift arm\nlift up\nstatus\n";
  console.update(c);
  assert(c.liftActive() && c.liftPulse() == 2000);
  assert(Serial.output.find("L 1 1 1 1 1 2000\n") != std::string::npos);
  Serial.input = "lift stop\n"; console.update(c);
  assert(fakePwm[4] == 0 && !c.liftArmed());
  assert(c.armLift()); assert(c.commandLift(robot::LiftPosition::raised));
  c.setEmergencyStop(true);
  assert(fakePwm[4] == 0 && !c.armLift());
  c.setEmergencyStop(false);
  assert(!c.keepLiftAlive() && !c.commandLift(robot::LiftPosition::raised));
  assert(c.armLift()); assert(c.commandLift(robot::LiftPosition::raised));
  tick(c); fakeNow += 50001; c.update();
  assert(c.fault() == robot::RobotFault::timing && fakePwm[4] == 0);
  assert(!c.armLift());

  robot::RobotController pwmFailure;
  assert(pwmFailure.begin());
  fakeSetupFailure = 4;
  assert(!pwmFailure.armLift());
  assert(pwmFailure.fault() == robot::RobotFault::lift);
  assert(fakePwm[4] == 0);
#else
  assert(!c.liftConfigured());
  assert(!c.armLift());
  assert(!c.liftArmed());
#endif
}
