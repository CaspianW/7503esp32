#pragma once

#include "../common/robot_types.h"

namespace robot {

class Encoder {
public:
  bool begin();
  int32_t readTicks(WheelId wheel) const;
  void readAllTicks(int32_t ticks[kWheelCount]) const;
  void resetAll();
};

} // robot 命名空间结束
