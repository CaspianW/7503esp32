// Emit the real firmware's status block against deterministic hardware fakes.
#include "fakes/Arduino.h"
#include "../src/app/robot_controller.h"
#include "../src/communication/serial_console.h"
#include <cassert>
#include <iostream>

int main() {
  robot::RobotController controller;
  robot::SerialConsole console;
  assert(controller.begin());
  console.begin();
  fakeNow += 10000;
  controller.update();
  Serial.input = "status\n";
  console.update(controller);
  std::cout << Serial.output;
}
