/**
 * @file serial_telemetry.h
 * @brief ESP32 telemetry snapshot and bounded ASCII wire encoding.
 */
#pragma once

#include "../common/robot_types.h"
#include <stddef.h>
#include <stdint.h>

namespace robot {

// Field names/types belong to the MCU wire contract, not ROS messages.
struct SerialTelemetry {
  uint32_t uptimeMs = 0;
  bool initialized = false;
  bool armed = false;
  bool motionAllowed = false;
  bool emergencyStopped = false;
  uint8_t fault = 0;
  bool encoderReady = false;
  bool imuReady = false;
  bool speedReady = false;
  bool benchMode = false;
  bool motorOutputsEnabled = false;
  bool chassisControlEnabled = false;
  bool manualMode = false;
  WheelValues wheelTarget{};
  WheelValues wheelSpeed{};
  WheelValues duty{};
  OdometryState odometry{};
  ImuSample imu{};
  bool liftOutputsEnabled = false;
  bool liftConfigured = false;
  bool liftReady = false;
  bool liftActive = false;
  uint8_t liftTarget = 2; // 0=lowered target, 1=raised target, 2=no active target
  uint16_t liftPulseUs = 0;
};

constexpr size_t serialTelemetryCapacity = 900;

// S/E/L are periodic diagnostics/status. O/I are included only for finite,
// initialized, fresh measurements. Source timestamps stay in MCU millis().
// Returns bytes excluding NUL, or 0 with an empty buffer if it does not fit.
size_t formatSerialTelemetry(char *buffer, size_t capacity,
                             const SerialTelemetry &frame, uint32_t maxSampleAgeMs);

} // namespace robot
