#pragma once
#include "../common/robot_types.h"
#include <stddef.h>
#include <stdint.h>

namespace robot {
struct SerialTelemetry {
  uint32_t uptimeMs = 0;
  bool initialized = false;
  bool armed = false;
  bool emergencyStopped = false;
  uint8_t fault = 0;
  bool encoderReady = false;
  bool imuReady = false;
  bool speedReady = false;
  OdometryState odometry{};
  ImuSample imu{};
  bool liftStateKnown = false;
  uint8_t liftState = 0;
};
constexpr size_t serialTelemetryCapacity = 128;
// Encodes complete packets into a bounded buffer. No partial batch is sent.
// Odom/IMU are omitted when unavailable; lift appears after first 3s completion.
size_t encodeSerialTelemetry(uint8_t *out, size_t capacity,
                              const SerialTelemetry &frame, uint32_t maxSampleAgeMs);
} // namespace robot
