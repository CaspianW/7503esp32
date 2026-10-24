/**
 * @file serial_console.h
 * @brief Bounded nonblocking commissioning serial interface and telemetry.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace robot {
class RobotController;
class SerialConsole {
public:
  void begin();
  void update(RobotController &controller);
private:
  void reply(const char *text);
  void telemetry(const RobotController &controller);
  char line_[96] = {};
  size_t size_ = 0;
  bool discard_ = false;
  uint32_t lastByte_ = 0; // 单位 ms
  uint32_t lastTelemetry_ = 0; // 单位 µs
};
} // namespace robot
