/**
 * @file lift_control_test.cpp
 * @brief Lift startup homing, completion and holding regressions.
 * @author Caspian Wu
 * @date 2026-11-01
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include "../../src/app/robot_controller.h"
#include "../../src/config/board_pins.h"
#include "../../src/config/robot_config.h"
#include <cassert>

static void tick(robot::RobotController &c) { fakeNow += 10000; c.update(); }
static uint32_t duty(uint16_t pulse) {
  return (static_cast<uint64_t>(pulse) * robot::config::servoPwmFrequency * (1ULL << 14)) / 1000000;
}
int main() {
  robot::RobotController c;
  assert(c.begin());
  assert(fakeAttached[robot::board::servoS2] == 4);
  assert(fakePwm[4] == duty(robot::config::liftLoweredPulse));
  assert(c.liftActive() && c.liftTarget() == 0 && !c.liftCompletionKnown());
  for (int n = 0; n < 299; ++n) tick(c);
  assert(!c.liftCompletionKnown());
  tick(c);
  assert(c.liftCompletionKnown() && c.liftState() == 0);
  assert(c.commandLift(robot::LiftPosition::raised));
  assert(c.liftTarget() == 1 && fakePwm[4] == duty(robot::config::liftRaisedPulse));
  for (int n = 0; n < 299; ++n) tick(c);
  assert(c.liftCompletionKnown() && c.liftState() == 0);
  tick(c);
  assert(c.liftCompletionKnown() && c.liftState() == 1);
  for (int n = 0; n < 700; ++n) tick(c);
  assert(fakePwm[4] == duty(robot::config::liftRaisedPulse));
  assert(c.commandLift(robot::LiftPosition::lowered));
  assert(c.liftState() == 1 && fakePwm[4] == duty(robot::config::liftLoweredPulse));
  c.setEmergencyStop(true);
  assert(c.emergencyStopped() && c.liftActive());
  assert(c.commandLift(robot::LiftPosition::raised));
  assert(fakePwm[4] == duty(robot::config::liftRaisedPulse));
  c.stopLift();
  assert(!c.liftActive() && fakePwm[4] == 0 && c.liftState() == 1);
  assert(c.commandLift(robot::LiftPosition::lowered));
  assert(fakePwm[4] == duty(robot::config::liftLoweredPulse));
  c.stop();
  assert(!c.armed() && !c.liftActive() && fakePwm[4] == 0);
}
