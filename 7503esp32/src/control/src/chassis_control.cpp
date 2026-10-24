/**
 * @file chassis_control.cpp
 * @brief Mecanum closed-loop control with validated inputs and conservative stop behavior.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../include/chassis_control.h"

#include <math.h>

namespace robot {

bool ChassisControl::configure(const ChassisParam &param) {
  ready_ = false;
  reset();
  if (!isfinite(param.maxLinear) || param.maxLinear <= 0.0f ||
      !isfinite(param.maxAngular) || param.maxAngular <= 0.0f ||
      !isfinite(param.maxWheel) || param.maxWheel <= 0.0f ||
      !isfinite(param.acceleration) || param.acceleration <= 0.0f ||
      !isfinite(param.pid.kp) || param.pid.kp <= 0.0f ||
      !isfinite(param.pid.ki) || param.pid.ki < 0.0f ||
      !isfinite(param.pid.kd) || param.pid.kd < 0.0f ||
      !isfinite(param.pid.outMax) || param.pid.outMax <= 0.0f ||
      param.pid.outMax > 1.0f || !isfinite(param.pid.iMax) ||
      param.pid.iMax < 0.0f ||
      !kinematics_.configure(param.base, param.track)) return false;
  param_ = param;
  wheels_.configure(param.pid);
  ready_ = true;
  return true;
}

bool ChassisControl::update(const Twist2D &command,
                            const WheelValues &measured, float dt,
                            WheelValues &duty) {
  duty = {};
  if (!ready_ || !isfinite(dt) || dt <= 0.0f || dt > 0.05f ||
      !isfinite(command.vx) || !isfinite(command.vy) ||
      !isfinite(command.wz)) { reset(); return false; }
  for (float value : measured.value) {
    if (!isfinite(value) || fabsf(value) > param_.maxWheel * 3.0f) {
      reset(); return false;
    }
  }
  // 零命令直接取消输出和积分，不通过减速斜坡或 PID 产生反向制动力。
  if (command.vx == 0.0f && command.vy == 0.0f && command.wz == 0.0f) {
    reset(); return true;
  }
  Twist2D limited = command;
  const float linear = hypotf(command.vx, command.vy);
  if (!isfinite(linear)) { reset(); return false; }
  if (linear > param_.maxLinear) {
    limited.vx *= param_.maxLinear / linear;
    limited.vy *= param_.maxLinear / linear;
  }
  if (limited.wz > param_.maxAngular) limited.wz = param_.maxAngular;
  if (limited.wz < -param_.maxAngular) limited.wz = -param_.maxAngular;
  WheelValues next{};
  if (!kinematics_.inverse(limited, next)) { reset(); return false; }
  MecanumKinematics::scaleToLimit(next, param_.maxWheel);
  float peak = 0.0f;
  for (uint8_t i = 0; i < wheelCount; ++i) {
    const float delta = fabsf(next.value[i] - target_.value[i]);
    if (delta > peak) peak = delta;
  }
  const float maxStep = param_.acceleration * dt;
  const float scale = peak > maxStep ? maxStep / peak : 1.0f;
  for (uint8_t i = 0; i < wheelCount; ++i)
    target_.value[i] += (next.value[i] - target_.value[i]) * scale;
  duty = wheels_.update(target_, measured, dt);
  return true;
}

void ChassisControl::reset() {
  target_ = {};
  wheels_.reset();
}

} // namespace robot
