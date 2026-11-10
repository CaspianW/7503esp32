#include "fakes/Arduino.h"
#include <driver/pcnt.h>
#include "../../src/app/robot_controller.h"
#include "../../src/communication/motor_debug.h"
#include "serial_test_helpers.h"
#include <cassert>

int main() {
  robot::RobotController c;
  robot::MotorDebug debug;
  assert(c.begin());
  debug.begin();
  assert(Serial.baud == 115200 && Serial.rx == 44 && Serial.tx == 43);
  // Binary arm commands are discarded, never dispatched or acknowledged.
  Serial.input = wire(robot::PacketType::arm, {1});
  debug.update(c);
  assert(Serial.input.empty() && Serial.output.empty() && !c.armed());
  for (int n = 0; n < 10; ++n) {
    fakePcnt[0] = 1; fakePcnt[1] = 2;
    fakePcnt[2] = -3; fakePcnt[3] = -4;
    fakeNow += 10000;
    c.update(); debug.update(c);
  }
  assert(Serial.output.find("FL(M4) ticks=40 speed=0.0785") != std::string::npos);
  assert(Serial.output.find("RL(M3) ticks=30 speed=0.0589") != std::string::npos);
  assert(Serial.output.find("FR(M2) ticks=20 speed=0.0393") != std::string::npos);
  assert(Serial.output.find("RR(M1) ticks=10 speed=0.0196") != std::string::npos);
  assert(Serial.output.find("fault=0") != std::string::npos);
  assert(!c.armed());
  Serial.output.clear(); Serial.capacity = 0;
  fakeNow += 100000;
  debug.update(c);
  assert(Serial.output.empty());
}
