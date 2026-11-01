/**
 * @file serial_link_test.cpp
 * @brief UART dispatch, watchdog and malformed-stream recovery tests.
 * @author Caspian Wu
 * @date 2026-11-01
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include "serial_test_helpers.h"
#include "../../src/communication/serial_link.h"
#include "../../src/app/robot_controller.h"
#include "../../src/config/board_pins.h"
#include <limits>

int main() {
  using robot::PacketType;
  robot::RobotController c;
  robot::SerialLink link;
  assert(c.begin()); link.begin();
  assert(Serial.baud == 115200 && Serial.format == SERIAL_8N1);
  assert(Serial.rx == 44 && Serial.tx == 43);
  const auto send = [&](const std::string &bytes) {
    Serial.output.clear(); Serial.input = bytes;
    while (Serial.available()) link.update(c);
  };
  send(wire(PacketType::arm, {1})); // No speed sample yet.
  assert(acknowledged(Serial.output, PacketType::arm, 1) && !c.armed());
  fakeNow += 10000; c.update();
  send(wire(PacketType::arm, {1}));
  assert(acknowledged(Serial.output, PacketType::arm, 0) && c.armed());
  send(velocityWire(.1f, 0, 0));
  assert(acknowledged(Serial.output, PacketType::velocity, 0));
  fakeNow += 10000; c.update();
  assert(c.motionAllowed() && fakePwm[0] != 0);
  send(wire(PacketType::lift, {1}));
  assert(c.liftActive() && fakePwm[4] != 0);
  const uint32_t holding = fakePwm[4];
  send(wire(PacketType::arm, {0}));
  assert(!c.armed() && fakePwm[0] == 0 && fakePwm[4] == holding);

  send(wire(PacketType::emergencyStop, {1}));
  assert(c.emergencyStopped() && c.liftActive());
  send(wire(PacketType::emergencyStop, {0}));
  assert(!c.emergencyStopped() && !c.armed());
  send(wire(PacketType::lift, {0}));
  assert(c.liftTarget() == 0);
  send(wire(PacketType::lift, {2}));
  assert(acknowledged(Serial.output, PacketType::lift, 1) && c.liftTarget() == 0);
  send(wire(PacketType::arm, {2}));
  assert(acknowledged(Serial.output, PacketType::arm, 1));
  send(wire(PacketType::velocity, {1}));
  assert(acknowledged(Serial.output, PacketType::velocity, 1));
  send(wire(static_cast<PacketType>(0xFE)));
  assert(acknowledged(Serial.output, static_cast<PacketType>(0xFE), 1));

  std::string broken = wire(PacketType::lift, {1});
  broken[5] ^= 1;
  send(broken);
  assert(Serial.output.empty() && c.liftTarget() == 0);
  send(std::string("garbage") + wire(PacketType::lift, {1}) + wire(PacketType::lift, {0}));
  assert(c.liftTarget() == 0 && packets(Serial.output).size() == 2);

  send(wire(PacketType::arm, {1}));
  send(velocityWire(.1f, 0, 0));
  for (int n = 0; n < 50; ++n) {
    fakeNow += 10000;
    c.update();
    send(broken); // CRC errors must not refresh the velocity watchdog.
  }
  assert(!c.armed() && !c.motionAllowed() && fakePwm[0] == 0);
  send(wire(PacketType::arm, {1}));
  assert(c.armed());
  send(velocityWire(std::numeric_limits<float>::quiet_NaN(), 0, 0));
  assert(!c.armed() && acknowledged(Serial.output, PacketType::velocity, 1));

  // A timed-out prefix must not consume a new valid command.
  const std::string complete = wire(PacketType::lift, {1});
  send(complete.substr(0, 4));
  assert(!acknowledged(Serial.output, PacketType::lift, 0));
  fakeNow += 101000;
  send(complete);
  assert(c.liftTarget() == 1 && acknowledged(Serial.output, PacketType::lift, 0));

  send(wire(PacketType::statusRequest));
  auto values = packets(Serial.output);
  assert(findPacket(values, PacketType::status) && acknowledged(Serial.output, PacketType::statusRequest, 0));
  send(wire(PacketType::liftStop));
  assert(!c.liftActive() && fakePwm[4] == 0);
  send(wire(PacketType::stop));
  assert(!c.armed() && !c.liftActive());
  Serial.capacity = 0;
  send(wire(PacketType::lift, {1}));
  assert(c.liftActive() && Serial.output.empty()); // TX cannot block dispatch.
  Serial.capacity = 2048;

  // 假长帧头不得在超时清理时吞掉后续完整急停包。
  send(std::string("\xAA\x55\x01\x20", 4) + wire(PacketType::emergencyStop, {1}));
  assert(!c.emergencyStopped());
  fakeNow += 101000;
  link.update(c);
  assert(c.emergencyStopped());
  assert(acknowledged(Serial.output, PacketType::emergencyStop, 0));
  send(wire(PacketType::emergencyStop, {0}));

  // 超时恢复必须依次执行所有完整包，并丢弃末尾残片。
  send(std::string("\xAA\x55\x01\x20", 4) +
       wire(PacketType::lift, {0}) + wire(PacketType::liftStop) + complete.substr(0, 4));
  fakeNow += 101000;
  link.update(c);
  assert(!c.liftActive());
  assert(acknowledged(Serial.output, PacketType::lift, 0));
  assert(acknowledged(Serial.output, PacketType::liftStop, 0));
  send(complete.substr(4)); // 残片不能与新数据拼成旧命令。
  assert(!c.liftActive());
  send(complete);
  assert(c.liftActive() && acknowledged(Serial.output, PacketType::lift, 0));
}
