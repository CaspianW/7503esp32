#pragma once

#include "robot_types.h"

namespace robot {

class Imu {
public:
  bool begin();
  bool read(ImuSample &sample);
  bool calibrateGyro(uint16_t sampleCount);
};

} // namespace robot
