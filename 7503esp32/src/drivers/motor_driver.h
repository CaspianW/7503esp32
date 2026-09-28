/**
 * @file motor_driver.h
 * @brief Four-channel motor driver interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../common/robot_types.h"

namespace robot {

class MotorDriver {
public:
  bool begin();
  void setDuty(WheelId wheel, float signedDuty);
  void setAllDuty(const WheelValues &signedDuty);
  void stopAll(bool brake);

private:
  bool initialized_ = false;
  int8_t activePin_[wheelCount] = {-1, -1, -1, -1};
};

} // robot 命名空间结束
