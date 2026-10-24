/**
 * @file encoder.h
 * @brief Four-channel motor encoder interface.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../../common/robot_types.h"

namespace robot {

class Encoder {
public:
  bool begin();
  int64_t readTicks(WheelId wheel);
  bool readAllTicks(int64_t ticks[wheelCount]);
  bool healthy() const { return initialized_ && healthy_; }
  void resetAll();

private:
  bool initialized_ = false;
  bool healthy_ = false;
  int64_t accumulatedTicks_[wheelCount] = {};
};

} // robot 命名空间结束
