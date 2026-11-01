#include "fakes/Arduino.h"
#include "serial_test_helpers.h"
#include "../src/communication/serial_link.h"
#include "../src/app/robot_controller.h"
#include "../src/config/robot_config.h"

static void tick(robot::RobotController &c, robot::SerialLink &link) {
  fakeNow += 10000; c.update(); link.update(c);
}
int main() {
  using robot::PacketType;
  robot::RobotController c;
  robot::SerialLink link;
  assert(c.begin()); link.begin();
  Serial.input = wire(PacketType::statusRequest);
  link.update(c);
  auto values = packets(Serial.output);
  assert(findPacket(values, PacketType::status));
  assert(!findPacket(values, PacketType::odometry) && !findPacket(values, PacketType::imu));
  assert(!findPacket(values, PacketType::liftState));
  Serial.output.clear();
  for (int n = 0; n < 10; ++n) tick(c, link);
  values = packets(Serial.output);
  assert(values.size() == 3 && !c.armed());
  assert(robot::readU32LE(findPacket(values, PacketType::odometry)->data) == 100);
  assert(robot::readU32LE(findPacket(values, PacketType::imu)->data) == 100);

  Serial.input = wire(PacketType::lift, {0}); link.update(c);
  assert(fakePwm[4] != 0 && !c.liftCompletionKnown());
  for (int n = 0; n < 300; ++n) tick(c, link);
  values = packets(Serial.output);
  assert(c.liftState() == 0 && findPacket(values, PacketType::liftState));
  Serial.output.clear();
  Serial.input = wire(PacketType::lift, {1}); link.update(c);
  for (int n = 0; n < 299; ++n) {
    tick(c, link);
    assert(c.liftState() == 0);
  }
  values = packets(Serial.output);
  for (const auto &p : values) if (p.type == PacketType::liftState) assert(p.data[0] == 0);
  Serial.output.clear();
  tick(c, link);
  values = packets(Serial.output);
  assert(c.liftState() == 1 && findPacket(values, PacketType::liftState)->data[0] == 1);

  Serial.output.clear(); fakeNow += 100000; link.update(c);
  values = packets(Serial.output);
  assert(!findPacket(values, PacketType::odometry) && !findPacket(values, PacketType::imu));
  assert(findPacket(values, PacketType::liftState)->data[0] == 1);
  Serial.output.clear(); Serial.capacity = 1;
  fakeNow += 100000; link.update(c);
  assert(Serial.output.empty());
  Serial.capacity = 2048;
  fakeNow += 100000; link.update(c);
  assert(findPacket(packets(Serial.output), PacketType::status));
}
