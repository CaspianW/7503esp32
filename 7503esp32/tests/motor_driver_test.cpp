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
#include <cassert>
#include <math.h>
int main() {
  robot::MotorDriver driver;
  assert(driver.begin());
  driver.setDuty(robot::frontLeft, 1);
#ifdef CALIBRATED_FIXTURE
  assert(fakePwm[0] == 204); // motorDutyLimit=0.2，10bit PWM
  assert(fakeAttached[robot::board::motorPins[0].pwmB] == 0);
  driver.setDuty(robot::frontLeft, -0.15f);
  assert(fakeAttached[robot::board::motorPins[0].pwmA] == 0);
  assert(fakeAttached[robot::board::motorPins[0].pwmB] == -1);
  assert(fakePin[robot::board::motorPins[0].pwmB] == LOW);
#else
  assert(fakePwm[0] == 0); // 默认编译开关禁止非零输出
#endif
  driver.setDuty(robot::frontLeft, NAN); assert(fakePwm[0] == 0);
  driver.setDuty(robot::frontLeft, 0.15f);
  driver.stopAll(false);
  for (auto value : fakePwm) assert(value == 0);
  assert(fakeAttached[robot::board::motorPins[0].pwmA] == -1);
  assert(fakeAttached[robot::board::motorPins[0].pwmB] == -1);
}
