/**
 * @file wheel_speed.cpp
 * @brief Wheel speed estimation from accumulated encoder counts.
 * @author Caspian Wu
 * @date 2026-10-20
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../include/wheel_speed.h"

#include <math.h>

namespace robot {

bool WheelSpeed::configure(float counts, float radius) {
  scale_ = 0.0f;
  reset();
  if (!isfinite(counts) || !isfinite(radius) ||
      counts <= 0.0f || radius <= 0.0f) return false;
  const float scale = 2.0f * 3.14159265358979323846f * radius / counts;
  if (!isfinite(scale) || scale <= 0.0f) return false;
  scale_ = scale;
  return true;
}

void WheelSpeed::reset() {
  primed_ = false;
  lastTime_ = 0;
  for (uint8_t i = 0; i < wheelCount; ++i) lastTicks_[i] = 0;
}

bool WheelSpeed::update(const int64_t (&ticks)[wheelCount], uint32_t now,
                        WheelValues &speed) {
  speed = {};
  if (scale_ <= 0.0f) return false;
  if (!primed_) {
    for (uint8_t i = 0; i < wheelCount; ++i) lastTicks_[i] = ticks[i];
    lastTime_ = now;
    primed_ = true;
    return false;
  }

  const uint32_t elapsed = now - lastTime_; // 单位 µs，兼容 micros() 回绕
  if (elapsed == 0) return false;
  const float dt = static_cast<float>(elapsed) * 0.000001f; // 单位 s
  WheelValues next{};
  for (uint8_t i = 0; i < wheelCount; ++i) {
    if ((lastTicks_[i] < 0 && ticks[i] > INT64_MAX + lastTicks_[i]) ||
        (lastTicks_[i] > 0 && ticks[i] < INT64_MIN + lastTicks_[i])) {
      return false;
    }
    // ticks 是启动以来的累计值；相减得到本采样间隔的相对计数。
    const int64_t deltaTicks = static_cast<int64_t>(ticks[i]) - lastTicks_[i];
    // 单次采样增量若异常巨大，不能安全推断速度。
    if (deltaTicks < INT32_MIN || deltaTicks > INT32_MAX) return false;
    next.value[i] = static_cast<float>(deltaTicks) * scale_ / dt;
    if (!isfinite(next.value[i])) return false;
  }
  for (uint8_t i = 0; i < wheelCount; ++i) {
    lastTicks_[i] = ticks[i];
  }
  lastTime_ = now;
  speed = next;
  return true;
}

} // robot 命名空间结束
