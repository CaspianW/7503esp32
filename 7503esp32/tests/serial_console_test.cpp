/**
 * @file serial_console_test.cpp
 * @brief Serial framing, partial timeout, emergency stop and blocked-output tests.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include "../src/communication/serial_console.h"
#include "../src/app/robot_controller.h"
#include <cassert>
int main() {
  robot::RobotController c;
  robot::SerialConsole console;
  c.begin(); console.begin();
  Serial.input = "status\n"; console.update(c);
  assert(Serial.output.find("S ") != std::string::npos);
  assert(Serial.output.find("E ") != std::string::npos);
  assert(Serial.output.find("OK\n") != std::string::npos);
  Serial.output.clear(); Serial.input = "estop 1\n"; console.update(c);
  assert(c.emergencyStopped());
  Serial.input = "estop 0\n"; console.update(c); assert(!c.emergencyStopped());
  Serial.input = std::string(130, 'a') + "\narm\n";
  while (Serial.available()) console.update(c);
  assert(Serial.output.find("ERR frame") != std::string::npos);
  Serial.output.clear(); Serial.input = "estop "; console.update(c);
  fakeNow += 101000; Serial.input = "1\n"; console.update(c);
  assert(!c.emergencyStopped());
  assert(Serial.output.find("ERR frame") != std::string::npos);
  Serial.capacity = 0; Serial.input = "status\n"; console.update(c); // 输出堵塞不阻塞控制
}
