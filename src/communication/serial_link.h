#pragma once
#include "serial_protocol.h"

namespace robot {
class RobotController;
class SerialLink {
public:
  void begin();
  void update(RobotController &controller);
private:
  bool dispatch(const SerialPacket &packet, RobotController &controller);
  void send(const uint8_t *data, size_t size);
  void telemetry(const RobotController &controller);
  SerialPacketParser parser_;
  uint32_t lastByteMs_ = 0;
  uint32_t lastTelemetryUs_ = 0;
};
} // namespace robot
