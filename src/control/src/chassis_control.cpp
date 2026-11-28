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
      !isfinite(param.ks) || param.ks < 0.0f ||
      !isfinite(param.kv) || param.kv < 0.0f ||
      !isfinite(param.minDuty) || param.minDuty < 0.0f ||
      !isfinite(param.kickstartDuty) || param.kickstartDuty < 0.0f ||
      !isfinite(param.kickstartSpeed) || param.kickstartSpeed < 0.0f ||
      !isfinite(param.brakeMinDuty) || param.brakeMinDuty < 0.0f ||
      !isfinite(param.brakeGain) || param.brakeGain < 0.0f ||
      !isfinite(param.brakeMaxDuty) || param.brakeMaxDuty < 0.0f ||
      !isfinite(param.brakeStopSpeed) || param.brakeStopSpeed < 0.0f ||
      !isfinite(param.pid.kp) || param.pid.kp <= 0.0f ||
      !isfinite(param.pid.ki) || param.pid.ki < 0.0f ||
      !isfinite(param.pid.kd) || param.pid.kd < 0.0f ||
      !isfinite(param.pid.outMax) || param.pid.outMax <= 0.0f ||
      param.pid.outMax > 1.0f || !isfinite(param.pid.iMax) ||
      param.pid.iMax < 0.0f || param.minDuty > param.pid.outMax ||
      param.kickstartDuty > param.pid.outMax ||
      param.brakeMaxDuty > param.pid.outMax ||
      param.brakeMinDuty > param.brakeMaxDuty ||
      !kinematics_.configure(param.base, param.track)) return false;
  param_ = param;
  wheels_.configure(param.pid);
  ready_ = true;
  return true;
}

bool ChassisControl::solveWheelTargets(const Twist2D &command,
                                       WheelValues &wheel) const {
  wheel = {};
  if (!ready_ || !isfinite(command.vx) || !isfinite(command.vy) ||
      !isfinite(command.wz)) {
    return false;
  }
  if (command.vx == 0.0f && command.vy == 0.0f && command.wz == 0.0f) {
    return true;
  }
  Twist2D limited = command;
  const float linear = hypotf(command.vx, command.vy);
  if (!isfinite(linear)) {
    return false;
  }
  if (linear > param_.maxLinear) {
    limited.vx *= param_.maxLinear / linear;
    limited.vy *= param_.maxLinear / linear;
  }
  if (limited.wz > param_.maxAngular) limited.wz = param_.maxAngular;
  if (limited.wz < -param_.maxAngular) limited.wz = -param_.maxAngular;
  if (!kinematics_.inverse(limited, wheel)) {
    return false;
  }
  MecanumKinematics::scaleToLimit(wheel, param_.maxWheel);
  // 消除斜向等复合运动中 vx ± vy 抵消后的微小浮点残差，避免误触发最低占空比。
  for (float &value : wheel.value) {
    if (fabsf(value) <= 1e-5f) value = 0.0f;
  }
  return true;
}

bool ChassisControl::update(const Twist2D &command,
                            const WheelValues &measured, float dt,
                            WheelValues &duty) {
  duty = {};
  if (!ready_ || !isfinite(dt) || dt <= 0.0f || dt > 0.05f) {
    reset(); return false;
  }
  for (float value : measured.value) {
    if (!isfinite(value) || fabsf(value) > param_.maxWheel * 3.0f) {
      reset(); return false;
    }
  }
  WheelValues next{};
  if (!solveWheelTargets(command, next)) {
    reset(); return false;
  }
  const uint32_t stepUs = static_cast<uint32_t>(lroundf(dt * 1000000.0f));
  // 零命令：清空运动设定与积分；若启用了主动制动且车轮仍在转动，输出短时有界反接制动，
  // 速度降到 brakeStopSpeed 以下、过零或超过 brakeMaxMs 立即切断为 0，绝不反向回弹。
  if (command.vx == 0.0f && command.vy == 0.0f && command.wz == 0.0f) {
    target_ = {};
    wheels_.reset();
    for (uint8_t i = 0; i < wheelCount; ++i) {
      kickstartActive_[i] = false;
      kickstartElapsedUs_[i] = 0U;
      prevTargetSign_[i] = 0;
    }
    const bool brakeEnabled = param_.brakeMaxDuty > 0.0f && param_.brakeMaxMs > 0U;
    if (!brakeEnabled) {
      zeroLatched_ = false;
      for (uint8_t i = 0; i < wheelCount; ++i) {
        brakingActive_[i] = false;
        brakeSpinSign_[i] = 0;
        brakeElapsedUs_[i] = 0U;
      }
      return true;
    }
    if (!zeroLatched_) {
      zeroLatched_ = true;
      for (uint8_t i = 0; i < wheelCount; ++i) {
        const float v = measured.value[i];
        if (fabsf(v) > param_.brakeStopSpeed) {
          brakingActive_[i] = true;
          brakeSpinSign_[i] = v > 0.0f ? 1 : -1;
          brakeElapsedUs_[i] = 0U;
        } else {
          brakingActive_[i] = false;
          brakeSpinSign_[i] = 0;
          brakeElapsedUs_[i] = 0U;
        }
      }
    }
    const uint32_t brakeLimitUs = param_.brakeMaxMs * 1000U;
    for (uint8_t i = 0; i < wheelCount; ++i) {
      if (!brakingActive_[i]) continue;
      const float v = measured.value[i];
      const bool stillSpinningSameDir =
          (brakeSpinSign_[i] > 0 && v > param_.brakeStopSpeed) ||
          (brakeSpinSign_[i] < 0 && v < -param_.brakeStopSpeed);
      if (!stillSpinningSameDir || brakeElapsedUs_[i] >= brakeLimitUs) {
        brakingActive_[i] = false;
        duty.value[i] = 0.0f;
      } else {
        float mag = param_.brakeMinDuty + param_.brakeGain * fabsf(v);
        if (mag > param_.brakeMaxDuty) mag = param_.brakeMaxDuty;
        duty.value[i] = -static_cast<float>(brakeSpinSign_[i]) * mag;
        brakeElapsedUs_[i] += stepUs;
      }
    }
    return true;
  }
  zeroLatched_ = false;
  for (uint8_t i = 0; i < wheelCount; ++i) {
    brakingActive_[i] = false;
    brakeSpinSign_[i] = 0;
    brakeElapsedUs_[i] = 0U;
  }
  float peak = 0.0f;
  for (uint8_t i = 0; i < wheelCount; ++i) {
    const float delta = fabsf(next.value[i] - target_.value[i]);
    if (delta > peak) peak = delta;
  }
  const float maxStep = param_.acceleration * dt;
  const float scale = peak > maxStep ? maxStep / peak : 1.0f;
  for (uint8_t i = 0; i < wheelCount; ++i)
    target_.value[i] += (next.value[i] - target_.value[i]) * scale;
  WheelValues feedforward{};
  for (uint8_t i = 0; i < wheelCount; ++i) {
    const float target = target_.value[i];
    if (target != 0.0f) {
      feedforward.value[i] = copysignf(param_.ks, target) + param_.kv * target;
    }
  }
  duty = wheels_.update(target_, measured, dt, feedforward, param_.minDuty);
  // 静止起转脉冲：仅在静止起动或换向且尚未转起来时给短时高占空比，一检测到转动即刻切回纯 PI。
  const uint32_t kickstartLimitUs = param_.kickstartMs * 1000U;
  const bool kickstartConfigured =
      param_.kickstartDuty > 0.0f && kickstartLimitUs > 0U;
  for (uint8_t i = 0; i < wheelCount; ++i) {
    const float target = target_.value[i];
    if (target == 0.0f) {
      kickstartActive_[i] = false;
      kickstartElapsedUs_[i] = 0U;
      prevTargetSign_[i] = 0;
      continue;
    }
    const int8_t sign = target > 0.0f ? 1 : -1;
    const float signedMeasured = sign > 0 ? measured.value[i] : -measured.value[i];
    const bool movingSameDir = signedMeasured >= param_.kickstartSpeed;
    if (kickstartConfigured && prevTargetSign_[i] != sign) {
      kickstartActive_[i] = !movingSameDir;
      kickstartElapsedUs_[i] = 0U;
    }
    prevTargetSign_[i] = sign;
    if (kickstartActive_[i]) {
      if (movingSameDir || kickstartElapsedUs_[i] >= kickstartLimitUs) {
        kickstartActive_[i] = false;
      } else {
        if (fabsf(duty.value[i]) < param_.kickstartDuty ||
            duty.value[i] * static_cast<float>(sign) < 0.0f) {
          duty.value[i] = copysignf(param_.kickstartDuty, target);
        }
        // 起转脉冲期间不累积 PID 积分，脱离静摩擦瞬间无超调。
        wheels_.reset(static_cast<WheelId>(i));
        kickstartElapsedUs_[i] += stepUs;
      }
    }
  }
  return true;
}

void ChassisControl::reset() {
  target_ = {};
  zeroLatched_ = false;
  for (uint8_t i = 0; i < wheelCount; ++i) {
    kickstartActive_[i] = false;
    kickstartElapsedUs_[i] = 0U;
    prevTargetSign_[i] = 0;
    brakingActive_[i] = false;
    brakeSpinSign_[i] = 0;
    brakeElapsedUs_[i] = 0U;
  }
  wheels_.reset();
}

} // namespace robot
