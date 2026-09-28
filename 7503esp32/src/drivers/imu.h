#pragma once

#include "../common/robot_types.h"

namespace robot {

class Imu {
public:
  bool begin();
  bool read(ImuSample &sample);
  bool calibrateGyro(uint16_t sampleCount);
};

} // robot 命名空间结束
