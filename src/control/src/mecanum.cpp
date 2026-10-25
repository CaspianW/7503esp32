/**
 * @file mecanum.cpp
 * @brief Forward and inverse kinematics for an X-layout mecanum chassis.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../include/mecanum.h"

#include <math.h>

namespace robot {

MecanumKinematics::MecanumKinematics()
    : arm_(0.0f), configured_(false) {}

bool MecanumKinematics::configure(float base, float track) {
  configured_ = false;
  arm_ = 0.0f;
  if (!isfinite(base) || !isfinite(track) || base <= 0.0f ||
      track <= 0.0f) {
    return false;
  }

  const float arm = base * 0.5f + track * 0.5f; // 单位 m
  if (!isfinite(arm) || arm <= 0.0f) {
    return false;
  }
  arm_ = arm;
  configured_ = true;
  return true;
}

bool MecanumKinematics::inverse(const Twist2D &body,
                                WheelValues &wheel) const {
  wheel = {};
  if (!configured_ || !isfinite(body.vx) || !isfinite(body.vy) ||
      !isfinite(body.wz)) {
    return false;
  }

  // X 形滚轮：左移时左前、右后反转；逆时针旋转时左轮反转。
  const float turn = arm_ * body.wz; // 单位 m/s
  WheelValues next{};
  next.value[frontLeft] = body.vx - body.vy - turn;
  next.value[rearLeft] = body.vx + body.vy - turn;
  next.value[frontRight] = body.vx + body.vy + turn;
  next.value[rearRight] = body.vx - body.vy + turn;
  for (uint8_t i = 0; i < wheelCount; ++i) {
    if (!isfinite(next.value[i])) {
      return false;
    }
  }
  wheel = next;
  return true;
}

bool MecanumKinematics::forward(const WheelValues &wheel,
                                Twist2D &body) const {
  body = {};
  if (!configured_) {
    return false;
  }
  for (uint8_t i = 0; i < wheelCount; ++i) {
    if (!isfinite(wheel.value[i])) {
      return false;
    }
  }

  const float fl = wheel.value[frontLeft];  // 单位 m/s
  const float rl = wheel.value[rearLeft];   // 单位 m/s
  const float fr = wheel.value[frontRight]; // 单位 m/s
  const float rr = wheel.value[rearRight];  // 单位 m/s
  Twist2D next{};
  next.vx = (fl + rl + fr + rr) * 0.25f;
  next.vy = (-fl + rl + fr - rr) * 0.25f;
  next.wz = (-fl - rl + fr + rr) * 0.25f / arm_;
  if (!isfinite(next.vx) || !isfinite(next.vy) ||
      !isfinite(next.wz)) {
    return false;
  }
  body = next;
  return true;
}

void MecanumKinematics::scaleToLimit(WheelValues &wheel, float max) {
  if (!isfinite(max) || max <= 0.0f) {
    wheel = {};
    return;
  }

  float peak = 0.0f; // 单位 m/s
  for (uint8_t i = 0; i < wheelCount; ++i) {
    if (!isfinite(wheel.value[i])) {
      wheel = {};
      return;
    }
    const float speed = fabsf(wheel.value[i]); // 单位 m/s
    if (speed > peak) {
      peak = speed;
    }
  }
  if (peak <= max) {
    return;
  }

  const float scale = max / peak;
  for (uint8_t i = 0; i < wheelCount; ++i) {
    wheel.value[i] *= scale;
  }
}

} // robot 命名空间结束
