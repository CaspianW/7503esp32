/**
 * @file serial_console_test.cpp
 * @brief Verify keyboard commands reach the current robot controller.
 * @author Caspian Wu
 * @date 2026-11-16
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */
#include "../../7503esp32.ino"
#include "../../src/config/robot_config.h"
#include "fakes/Arduino.h"
#include <cassert>
#include <cmath>

static void step() { fakeNow += 10000; loop(); }
static void send(const char *line) { Serial.input = line; step(); }

int main() {
  setup(); step();
  assert(Serial.baud == 115200 && Serial.rx == 44 && Serial.tx == 43);
  assert(!robotController.armed());
  send("manual arm\n");
  assert(robotController.armed() && robotController.manualMode());
  send("manual duty 0.65\n");
  send("manual vel 1 0 0\n"); step();
  for (float duty : robotController.duty().value)
    assert(std::fabs(duty - 0.65f) < 0.001f);
  send("lift up\n");
  const auto pulse = robotController.liftPulse();
  assert(pulse == robot::config::liftRaisedPulse);
  const auto raisedDuty = fakePwm[4];
  // 模拟仅按 U 后保持空闲超过 3s，零速度刷新不能改变舵机输出。
  for (int n = 0; n < 400; ++n) {
    send("manual vel 0 0 0\n");
    assert(robotController.liftPulse() == pulse && fakePwm[4] == raisedDuty);
    assert(robotController.fault() == robot::RobotFault::none);
  }
  assert(robotController.liftCompletionKnown() && robotController.liftState() == 1);
  send("manual stop\n");
  assert(!robotController.armed() && robotController.liftPulse() == pulse);
  for (float duty : robotController.duty().value) assert(duty == 0);
  send("arm\n");
  assert(robotController.armed() && !robotController.manualMode());
  Serial.output.clear();
  send("vel 0.10 0 0\n"); step();
  assert(Serial.output.find("OK\n") != std::string::npos);
  assert(robotController.armed() && robotController.motionAllowed());
  assert(robotController.fault() == robot::RobotFault::none);
  for (float duty : robotController.duty().value) assert(duty > 0.0f);
  Serial.output.clear();
  for (int n = 0; n < 10; ++n) send("vel 0 0 0\n");
  assert(robotController.armed() && robotController.fault() == robot::RobotFault::none);
  assert(Serial.output.find(" 1 1 1 0 0 ") != std::string::npos);
  send("vel 0 0 0.50\n"); step();
  assert(robotController.fault() == robot::RobotFault::none && robotController.motionAllowed());
  send("vel 0.45 0 0\n"); step();
  assert(robotController.motionAllowed());
  send("estop 1\n");
  assert(robotController.emergencyStopped() && !robotController.armed());
  send("estop 0\n");
  assert(!robotController.emergencyStopped() && !robotController.armed());
  Serial.output.clear();
  send("vel nan 0 0\n");
  assert(Serial.output.find("ERR command_or_state") != std::string::npos);
  send("arm\n"); send("vel 0.45 0 0\n");
  for (int n = 0; n < 51; ++n) step();
  assert(!robotController.armed());
}
