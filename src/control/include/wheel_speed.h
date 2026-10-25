/**
 * @file wheel_speed.h
 * @brief Wheel speed estimation from accumulated encoder counts.
 * @author Caspian Wu
 * @date 2026-10-20
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../../common/robot_types.h"

namespace robot {

class WheelSpeed {
public:
  // counts 单位 count/rev，radius 单位 m。
  bool configure(float counts, float radius);
  // ticks 是四路累计计数；now 单位 µs，输出轮速单位 m/s。
  // 第一帧只建立基准，返回 false；失败时 speed 清零。
  // 单个采样间隔的计数增量超过 int32 范围时拒绝更新。
  bool update(const int64_t (&ticks)[wheelCount], uint32_t now,
              WheelValues &speed);
  // 与 Encoder::resetAll() 同时调用，避免计数清零造成速度尖峰。
  void reset();

private:
  float scale_ = 0.0f; // 单位 m/count
  int64_t lastTicks_[wheelCount] = {};
  uint32_t lastTime_ = 0; // 单位 µs
  bool primed_ = false;
};

} // robot 命名空间结束
