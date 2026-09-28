/**
 * @file imu.h
 * @brief ICM-42670-P IMU interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../common/robot_types.h"

#include <ICM42670P.h>

namespace robot {

class Imu {
public:
  bool begin();
  bool read(ImuSample &sample);
  bool calibrateGyro(uint16_t sampleCount);

private:
  ICM42670 sensorLowAddress_{Wire, false};
  ICM42670 sensorHighAddress_{Wire, true};
  ICM42670 *sensor_ = nullptr;
  bool initialized_ = false;
  float gyroBiasRadPerSec_[3] = {};
};

} // robot 命名空间结束
