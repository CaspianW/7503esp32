/**
 * @file imu.cpp
 * @brief Deterministic stationary IMU boundary for scheduler tests.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../../src/drivers/include/imu.h"
#include "Arduino.h"
namespace robot {
bool Imu::begin() { return true; }
bool Imu::read(ImuSample &sample) {
  sample = {}; sample.accelZ = 9.80665f; sample.timestamp = millis(); return true;
}
} // namespace robot
