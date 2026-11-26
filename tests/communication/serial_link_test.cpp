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
#include <driver/pcnt.h>
#include "../../src/communication/serial_link.h"
#include "../../src/app/robot_controller.h"
#include "../../src/config/board_pins.h"
#include "../../src/config/robot_config.h"
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
  // Boot auto-arms immediately and accepts 0x03(1) / 0x01 with zero startup race.
  assert(c.armed());
  send(wire(PacketType::arm, {1}));
  assert(acknowledged(Serial.output, PacketType::arm, 0) && c.armed());
  send(velocityWire(.45f, 0, 0));
  assert(acknowledged(Serial.output, PacketType::velocity, 0));
  fakeNow += 10000; c.update();
  assert(c.motionAllowed() && fakePwm[0] != 0);
  send(wire(PacketType::lift, {1}));
  assert(c.liftActive() && fakePwm[4] != 0);
  const uint32_t holding = fakePwm[4];
  send(wire(PacketType::stop));
  assert(acknowledged(Serial.output, PacketType::stop, 0));
  assert(!c.armed() && !c.motionAllowed());
  for (uint8_t n = 0; n < robot::wheelCount; ++n) assert(fakePwm[n] == 0);
  assert(c.liftActive() && c.liftTarget() == 1 && fakePwm[4] == holding);
  send(wire(PacketType::arm, {1}));
  send(velocityWire(.10f, 0, 0));
  assert(acknowledged(Serial.output, PacketType::velocity, 0));
  fakeNow += 10000; c.update();
  assert(c.armed() && c.motionAllowed() && fakePwm[0] != 0);
  assert(c.fault() == robot::RobotFault::none);
  send(wire(PacketType::statusRequest));
  {
    const auto statusPackets = packets(Serial.output);
    const auto *status = findPacket(statusPackets, PacketType::status);
    assert(status && status->data[1] == 0 && (status->data[0] & 2));
  }
  send(velocityWire(0, 0, 0));
  assert(c.armed() && fakePwm[0] == 0 && c.fault() == robot::RobotFault::none);
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
  send(velocityWire(.45f, 0, 0));
  for (int n = 0; n < 99; ++n) {
    for (uint8_t i = 0; i < robot::wheelCount; ++i)
      fakePcnt[i] = static_cast<int16_t>(10 * robot::config::encoderDirection[i]);
    fakeNow += 10000;
    c.update();
    send(broken); // CRC errors must not refresh the velocity watchdog.
  }
  assert(c.armed() && c.motionAllowed() && fakePwm[0] != 0);
  for (uint8_t i = 0; i < robot::wheelCount; ++i)
    fakePcnt[i] = static_cast<int16_t>(10 * robot::config::encoderDirection[i]);
  fakeNow += 10000;
  c.update();
  send(broken);
  // Watchdog stops wheels and zeroes targets, but keeps armed so next 0x01 resumes immediately.
  assert(c.armed() && !c.motionAllowed() && fakePwm[0] == 0 && c.fault() == robot::RobotFault::none);
  for (float v : c.wheelTarget().value) assert(v == 0.0f);
  send(wire(PacketType::lift, {0}));
  assert(acknowledged(Serial.output, PacketType::lift, 0) && c.liftTarget() == 0);
  send(velocityWire(.45f, 0, 0));
  assert(acknowledged(Serial.output, PacketType::velocity, 0));
  fakeNow += 10000; c.update();
  assert(c.armed() && c.motionAllowed() && fakePwm[0] != 0);
  // 0x01 with NaN/Inf stops wheels safely and replies ACK error without disarming or faulting.
  const uint32_t liftBeforeNan = fakePwm[4];
  send(velocityWire(std::numeric_limits<float>::quiet_NaN(), 0, 0));
  assert(c.armed() && !c.motionAllowed() && fakePwm[0] == 0 &&
         c.fault() == robot::RobotFault::none && c.liftActive() && fakePwm[4] == liftBeforeNan &&
         acknowledged(Serial.output, PacketType::velocity, 1));
  send(velocityWire(.45f, 0, 0));
  assert(acknowledged(Serial.output, PacketType::velocity, 0));
  fakeNow += 10000; c.update();
  assert(c.armed() && c.motionAllowed() && fakePwm[0] != 0);
  send(velocityWire(0, std::numeric_limits<float>::infinity(), 0));
  assert(c.armed() && !c.motionAllowed() && fakePwm[0] == 0 &&
         c.fault() == robot::RobotFault::none &&
         acknowledged(Serial.output, PacketType::velocity, 1));

  // Transient IMU dropout sets imuValid=0 without faulting or blocking chassis/lift.
  fakeImuReadStatus = -1;
  for (int n = 0; n < 55; ++n) {
    fakeNow += 10000; c.update();
  }
  assert(!c.imuReady() && c.fault() == robot::RobotFault::none && c.armed());
  send(wire(PacketType::statusRequest));
  {
    const auto sp = packets(Serial.output);
    const auto *st = findPacket(sp, PacketType::status);
    assert(st && st->data[1] == 0 && (st->data[0] & 2) && !(st->data[0] & 16));
    assert(!findPacket(sp, PacketType::imu) && findPacket(sp, PacketType::odometry));
  }
  send(wire(PacketType::lift, {0}));
  assert(acknowledged(Serial.output, PacketType::lift, 0));
  fakeImuReadStatus = 0;
  for (int n = 0; n < 20; ++n) {
    fakeNow += 10000; c.update();
  }
  assert(c.imuReady());

  // Runtime encoder fault pauses wheel PWM while staying armed, preserves lift, and auto-recovers.
  send(velocityWire(.45f, 0, 0));
  fakePcntFailure = 1;
  fakeNow += 10000; c.update();
  assert(c.armed() && fakePwm[0] == 0 && c.fault() == robot::RobotFault::encoder);
  assert(c.liftActive() && fakePwm[4] == liftBeforeNan);
  send(wire(PacketType::statusRequest));
  {
    const auto sp = packets(Serial.output);
    const auto *st = findPacket(sp, PacketType::status);
    assert(st && st->data[1] == static_cast<uint8_t>(robot::RobotFault::encoder) && (st->data[0] & 2));
  }
  send(wire(PacketType::lift, {0}));
  assert(acknowledged(Serial.output, PacketType::lift, 0));
  fakePcntFailure = 0;
  fakeNow += 10000; c.update();
  assert(c.armed() && c.fault() == robot::RobotFault::none);
  send(velocityWire(.45f, 0, 0));
  assert(acknowledged(Serial.output, PacketType::velocity, 0));
  fakeNow += 10000; c.update();
  assert(c.motionAllowed() && fakePwm[0] != 0);
  send(wire(PacketType::arm, {0}));
  assert(acknowledged(Serial.output, PacketType::arm, 0) && !c.armed() && fakePwm[0] == 0);

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
  // Removed 0x07 must be rejected without detaching the servo PWM.
  const uint32_t raisedPwm = fakePwm[4];
  const auto removedCommand = static_cast<PacketType>(0x07);
  send(wire(removedCommand));
  assert(acknowledged(Serial.output, removedCommand, 1));
  assert(c.liftActive() && fakePwm[4] == raisedPwm);
  send(wire(PacketType::stop));
  assert(!c.armed() && c.liftActive() && fakePwm[4] == raisedPwm);
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
       wire(PacketType::lift, {0}) + wire(PacketType::stop) + complete.substr(0, 4));
  fakeNow += 101000;
  link.update(c);
  assert(c.liftActive() && c.liftTarget() == 0);
  assert(acknowledged(Serial.output, PacketType::lift, 0));
  assert(acknowledged(Serial.output, PacketType::stop, 0));
  send(complete.substr(4)); // 残片不能与新数据拼成旧命令。
  assert(c.liftActive() && c.liftTarget() == 0);
  send(complete);
  assert(c.liftActive() && acknowledged(Serial.output, PacketType::lift, 0));
}
