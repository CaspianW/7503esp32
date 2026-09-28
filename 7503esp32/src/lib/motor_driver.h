#pragma once

#include "robot_types.h"

namespace robot {

class MotorDriver {
public:
  bool begin();
  void setDuty(WheelId wheel, float signedDuty);
  void setAllDuty(const WheelValues &signedDuty);
  void stopAll(bool brake);
};

} // namespace robot
