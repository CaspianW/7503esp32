#include "serial_link.h"
#include "serial_telemetry.h"
#include "../app/robot_controller.h"
#include "../config/board_pins.h"
#include "../config/robot_config.h"
#include <Arduino.h>
#include <math.h>

namespace robot {
void SerialLink::begin() {
  Serial.setTxBufferSize(2048);
  Serial.begin(config::serialBaud, SERIAL_8N1, board::serialRx, board::serialTx);
  parser_.reset();
  lastTelemetryUs_ = micros();
}
void SerialLink::send(const uint8_t *data, size_t size) {
  if (Serial.availableForWrite() >= static_cast<int>(size)) Serial.write(data, size);
}
bool SerialLink::dispatch(const SerialPacket &p, RobotController &c) {
  switch (p.type) {
  case PacketType::velocity: {
    if (p.length != 12) return false;
    const Twist2D value{readFloatLE(p.data), readFloatLE(p.data + 4), readFloatLE(p.data + 8)};
    if (!isfinite(value.vx) || !isfinite(value.vy) || !isfinite(value.wz)) {
      c.disarm(); return false;
    }
    return c.commandVelocity(value);
  }
  case PacketType::lift:
    if (p.length != 1 || p.data[0] > 1) return false;
    return c.commandLift(p.data[0] ? LiftPosition::raised : LiftPosition::lowered);
  case PacketType::arm:
    if (p.length != 1 || p.data[0] > 1) return false;
    if (p.data[0]) return c.arm();
    c.disarm(); return true;
  case PacketType::emergencyStop:
    if (p.length != 1 || p.data[0] > 1) return false;
    c.setEmergencyStop(p.data[0] != 0);
    return c.emergencyStopped() == (p.data[0] != 0);
  case PacketType::stop:
    if (p.length) return false;
    c.stop(); return true;
  case PacketType::liftStop:
    if (p.length) return false;
    c.stopLift(); return true;
  case PacketType::statusRequest:
    if (p.length) return false;
    telemetry(c); return true;
  default: return false;
  }
}
void SerialLink::update(RobotController &c) {
  // Incomplete commands cannot be joined to a new packet after a long pause.
  if (parser_.pending() && static_cast<uint32_t>(millis() - lastByteMs_) > 100)
    parser_.reset();
  uint8_t budget = 64;
  while (budget-- && Serial.available()) {
    const int byte = Serial.read();
    if (byte < 0) break;
    lastByteMs_ = millis();
    SerialPacket packet;
    if (!parser_.feed(static_cast<uint8_t>(byte), packet)) continue;
    do {
      const bool accepted = dispatch(packet, c);
      const uint8_t response[] = {static_cast<uint8_t>(packet.type),
                                 static_cast<uint8_t>(accepted ? 0 : 1)};
      uint8_t buffer[10];
      const size_t size = encodeSerialPacket(buffer, sizeof(buffer), PacketType::ack, response, 2);
      send(buffer, size);
    } while (parser_.take(packet));
  }
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - lastTelemetryUs_) >= config::telemetryPeriod) {
    lastTelemetryUs_ = now;
    telemetry(c);
  }
}
void SerialLink::telemetry(const RobotController &c) {
  SerialTelemetry f;
  f.uptimeMs = millis();
  f.initialized = c.initialized();
  f.armed = c.armed();
  f.emergencyStopped = c.emergencyStopped();
  f.fault = static_cast<uint8_t>(c.fault());
  f.encoderReady = c.encoderReady();
  f.imuReady = c.imuReady();
  f.speedReady = c.speedReady();
  f.odometry = c.odometry();
  f.imu = c.imuSample();
  f.liftStateKnown = c.liftCompletionKnown();
  f.liftState = c.liftState();
  uint8_t buffer[serialTelemetryCapacity];
  const size_t size = encodeSerialTelemetry(buffer, sizeof(buffer), f, config::telemetrySampleMaxAge);
  if (size) send(buffer, size);
}
} // namespace robot
