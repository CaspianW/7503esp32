/**
 * @file lift_control_test.cpp
 * @brief Host tests for lift endpoints and independence from chassis emergency stop.
 * @author Caspian Wu
 * @date 2026-10-24
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include "../src/app/robot_controller.h"
#include "../src/communication/serial_console.h"
#include "../src/config/board_pins.h"
#include "../src/config/robot_config.h"
#include <cassert>

static void tick(robot::RobotController &controller) {
  fakeNow += 10000;
  controller.update();
}

static uint32_t pulseDuty(uint16_t pulse) {
  return static_cast<uint32_t>(
      (static_cast<uint64_t>(pulse) * robot::config::servoPwmFrequency *
       (1ULL << 14)) /
      1000000ULL);
}

int main() {
  assert(robot::config::liftServoChannel == 1);
  assert(robot::board::servoS2 == 21);

  robot::RobotController controller;
  assert(controller.begin());
  if (!robot::config::liftOutputsEnabled) {
    assert(fakeAttached[robot::board::servoS2] == -1);
    assert(!controller.commandLift(robot::LiftPosition::raised));
    return 0;
  }

  assert(controller.liftConfigured());
  assert(fakeAttached[robot::board::servoS2] == 4);
  assert(fakePwm[4] == 0); // 上电初始化PWM，但未收到目标前不输出脉冲
  assert(!controller.liftActive());

  // 收到位置命令即可输出，不需要单独 arm。
  assert(controller.commandLift(robot::LiftPosition::raised));
  assert(controller.liftActive());
  assert(controller.liftTarget() == 1);
  assert(controller.liftPulse() == robot::config::liftRaisedPulse);
  assert(fakePwm[4] == pulseDuty(robot::config::liftRaisedPulse));

  // 输出持续保持，不因等待时间而自动关闭。
  for (int i = 0; i < 1000; ++i) tick(controller);
  assert(controller.liftActive());
  assert(fakePwm[4] == pulseDuty(robot::config::liftRaisedPulse));

  assert(controller.commandLift(robot::LiftPosition::lowered));
  assert(controller.liftTarget() == 0);
  assert(controller.liftPulse() == robot::config::liftLoweredPulse);
  assert(fakePwm[4] == pulseDuty(robot::config::liftLoweredPulse));

  robot::SerialConsole console;
  console.begin();
  Serial.output.clear();
  Serial.input = "lift up\n";
  console.update(controller);
  assert(controller.liftActive());
  assert(controller.liftPulse() == robot::config::liftRaisedPulse);
  assert(Serial.output.find("OK\n") != std::string::npos);

  Serial.input = "lift stop\n";
  console.update(controller);
  assert(!controller.liftActive());
  assert(controller.liftTarget() == 2);
  assert(controller.liftPulse() == 0);
  assert(fakePwm[4] == 0);
  assert(fakeAttached[robot::board::servoS2] == -1);

  // 显式 stop 后的新位置命令会自动重新初始化PWM。
  Serial.input = "lift down\n";
  console.update(controller);
  assert(controller.liftActive());
  assert(controller.liftPulse() == robot::config::liftLoweredPulse);
  assert(fakePwm[4] == pulseDuty(robot::config::liftLoweredPulse));
  assert(fakeAttached[robot::board::servoS2] == 4);

  // 直接输入数字已不是有效舵机命令；拒绝时舵机继续保持原端点脉冲。
  Serial.output.clear();
  Serial.input = "750\n";
  console.update(controller);
  assert(controller.liftActive());
  assert(controller.liftPulse() == robot::config::liftLoweredPulse);
  assert(fakePwm[4] == pulseDuty(robot::config::liftLoweredPulse));
  assert(fakeAttached[robot::board::servoS2] == 4);
  assert(Serial.output.find("ERR command_or_state\n") != std::string::npos);

  // 标定范围内的数字同样不能控制舵机；只有 lift up/down 可输出端点。
  Serial.output.clear();
  Serial.input = "1700\n";
  console.update(controller);
  assert(controller.liftActive());
  assert(controller.liftPulse() == robot::config::liftLoweredPulse);
  assert(fakePwm[4] == pulseDuty(robot::config::liftLoweredPulse));
  assert(Serial.output.find("ERR command_or_state\n") != std::string::npos);

  Serial.output.clear();
  Serial.input = "lift up\n";
  console.update(controller);
  assert(controller.liftActive());
  assert(controller.liftPulse() == robot::config::liftRaisedPulse);
  assert(fakePwm[4] == pulseDuty(robot::config::liftRaisedPulse));

  controller.setEmergencyStop(true);
  assert(controller.emergencyStopped());
  assert(controller.liftActive());
  assert(fakePwm[4] == pulseDuty(robot::config::liftRaisedPulse));
  assert(controller.commandLift(robot::LiftPosition::lowered));
  assert(controller.liftPulse() == robot::config::liftLoweredPulse);
  assert(fakePwm[4] == pulseDuty(robot::config::liftLoweredPulse));

  controller.stop();
  assert(!controller.liftActive()); // 显式 stop 仍会关闭升降PWM
  assert(fakePwm[4] == 0);
}
