#pragma once

#include "robot_types.h"

namespace robot {

class Encoder {
public:
  bool begin();
  int32_t readTicks(WheelId wheel) const;
  void readAllTicks(int32_t ticks[kWheelCount]) const;
  void resetAll();
};

} // namespace robot
