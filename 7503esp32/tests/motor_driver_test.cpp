/**
 * @file motor_driver_test.cpp
 * @brief Motor output gate, duty limit, direction switching and stop pin tests.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include "../src/drivers/include/motor_driver.h"
#include "../src/config/board_pins.h"
#include "../src/config/robot_config.h"
#include <cassert>
#include <math.h>
int main() {
  robot::MotorDriver driver;
  assert(driver.begin());
  driver.setDuty(robot::frontLeft, 1);
  if (robot::config::motorOutputsEnabled) {
    assert(fabsf(static_cast<float>(fakePwm[0]) / 1023.0f - robot::config::motorDutyLimit) < 0.002f);
    const int positivePin = robot::config::motorDirection[robot::frontLeft] > 0
      ? robot::board::motorPins[0].pwmB : robot::board::motorPins[0].pwmA;
    const int negativePin = robot::config::motorDirection[robot::frontLeft] > 0
      ? robot::board::motorPins[0].pwmA : robot::board::motorPins[0].pwmB;
    assert(fakeAttached[positivePin] == 0);
    driver.setDuty(robot::frontLeft, -0.15f);
    assert(fakeAttached[negativePin] == 0);
    assert(fakeAttached[positivePin] == -1);
    assert(fakePin[positivePin] == LOW);
  } else {
    assert(fakePwm[0] == 0);
  }
  driver.setDuty(robot::frontLeft, NAN); assert(fakePwm[0] == 0);
  driver.setDuty(robot::frontLeft, 0.15f);
  driver.stopAll(false);
  for (auto value : fakePwm) assert(value == 0);
  assert(fakeAttached[robot::board::motorPins[0].pwmA] == -1);
  assert(fakeAttached[robot::board::motorPins[0].pwmB] == -1);
}
