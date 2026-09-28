#pragma once

#include "../common/robot_types.h"

namespace robot {

class MotorDriver {
public:
  bool begin();
  void setDuty(WheelId wheel, float signedDuty);
  void setAllDuty(const WheelValues &signedDuty);
  void stopAll(bool brake);
};

} // robot 命名空间结束
