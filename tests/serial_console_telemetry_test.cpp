// Exercise actual startup, scheduled UART reporting and stale sensor suppression.
#include "fakes/Arduino.h"
#include "../src/app/robot_controller.h"
#include "../src/communication/serial_console.h"
#include "../src/config/robot_config.h"
#include <cassert>
#include <string>

int main() {
  robot::RobotController controller;
  robot::SerialConsole console;
  assert(controller.begin());
  console.begin();

  // Before the first control sample, status/lift are available but O/I are absent.
  Serial.input = "status\n";
  console.update(controller);
  assert(Serial.output.find("S ") == 0);
  assert(Serial.output.find("\nL ") != std::string::npos);
  assert(Serial.output.find("\nO ") == std::string::npos);
  assert(Serial.output.find("\nI ") == std::string::npos);
  Serial.output.clear();

  // No host handshake, ROS process or command is required for periodic telemetry.
  for (unsigned n = 0; n < robot::config::telemetryPeriod / robot::config::controlPeriod; ++n) {
    fakeNow += robot::config::controlPeriod;
    controller.update();
    console.update(controller);
  }
  assert(Serial.output.find("S 100 ") == 0);
  assert(Serial.output.find("\nO 100 ") != std::string::npos);
  assert(Serial.output.find("\nI 100 ") != std::string::npos);
  assert(Serial.output.find("\nL ") != std::string::npos);
  assert(!controller.armed());
  for (unsigned channel = 0; channel < 4; ++channel) assert(fakePwm[channel] == 0);

  // If data collection stalls, report status without presenting cached samples as new.
  Serial.output.clear();
  fakeNow += robot::config::telemetrySampleMaxAge * 1000;
  console.update(controller);
  assert(Serial.output.find("S 200 ") == 0);
  assert(Serial.output.find("\nO ") == std::string::npos);
  assert(Serial.output.find("\nI ") == std::string::npos);
  assert(Serial.output.find("\nL ") != std::string::npos);

  // TX backpressure drops the whole block; it must not block or emit a partial line.
  Serial.output.clear();
  Serial.capacity = 1;
  fakeNow += robot::config::telemetryPeriod;
  console.update(controller);
  assert(Serial.output.empty());
  Serial.capacity = 2048;
  fakeNow += robot::config::telemetryPeriod;
  console.update(controller);
  assert(Serial.output.find("S ") == 0);
  assert(Serial.output.back() == '\n');
}
