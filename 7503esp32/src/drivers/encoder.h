/**
 * @file encoder.h
 * @brief Four-channel motor encoder interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../common/robot_types.h"

namespace robot {

class Encoder {
public:
  bool begin();
  int32_t readTicks(WheelId wheel);
  void readAllTicks(int32_t ticks[wheelCount]);
  void resetAll();

private:
  bool initialized_ = false;
  int32_t accumulatedTicks_[wheelCount] = {};
};

} // robot 命名空间结束
