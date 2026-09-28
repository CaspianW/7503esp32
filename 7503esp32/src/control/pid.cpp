/**
 * @file pid.cpp
 * @brief Discrete PID controller with output limiting and anti-windup.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "pid.h"

#include <math.h>

namespace robot {

Pid::Pid()
    : param_{}, iSum_(0.0f), prevErr_(0.0f), prevMeas_(0.0f),
      hasPrev_(false) {}

void Pid::configure(const PidParam &param) {
  param_ = param;
  if (!isfinite(param_.kp)) param_.kp = 0.0f;
  if (!isfinite(param_.ki)) param_.ki = 0.0f;
  if (!isfinite(param_.kd)) param_.kd = 0.0f;
  if (!isfinite(param_.outMax)) param_.outMax = 0.0f;
  if (!isfinite(param_.iMax)) param_.iMax = 0.0f;
  param_.outMax = fabsf(param_.outMax);
  param_.iMax = fabsf(param_.iMax);
  reset();
}

float Pid::update(float target, float measured, float dt) {
  if (!isfinite(target) || !isfinite(measured) || !isfinite(dt) ||
      dt <= 0.0f) {
    // 忽略无效采样，并清除微分历史，避免下一次采样产生微分尖峰。
    hasPrev_ = false;
    return 0.0f;
  }

  const float err = target - measured;
  if (!isfinite(err)) {
    reset();
    return 0.0f;
  }

  const float pTerm = param_.kp * err;
  float dTerm = 0.0f;
  if (hasPrev_ && param_.kd != 0.0f) {
    if (param_.dMeas) {
      // 测量值微分前加负号，避免微分项与控制方向相反。
      dTerm = -param_.kd * (measured - prevMeas_) / dt;
    } else {
      dTerm = param_.kd * (err - prevErr_) / dt;
    }
  }

  float nextISum = iSum_;
  if (param_.ki != 0.0f) {
    nextISum += err * dt;
    if (!isfinite(nextISum)) {
      nextISum = err > 0.0f ? param_.iMax : -param_.iMax;
    }
    if (nextISum > param_.iMax) {
      nextISum = param_.iMax;
    } else if (nextISum < -param_.iMax) {
      nextISum = -param_.iMax;
    }
  }

  const float pdTerm = pTerm + dTerm;
  const float nextOut = pdTerm + param_.ki * nextISum;
  const float iDelta = param_.ki * (nextISum - iSum_);
  const bool windsUp =
      (nextOut > param_.outMax && iDelta > 0.0f) ||
      (nextOut < -param_.outMax && iDelta < 0.0f);
  // NaN 表示中间计算已溢出或出现无效组合，不能据此更新积分状态。
  if (!isnan(nextOut) && !isnan(iDelta) && !windsUp) {
    iSum_ = nextISum;
  }

  const float iTerm = param_.ki * iSum_;
  float out = pdTerm + iTerm;
  if (isnan(out)) {
    out = 0.0f;
  }
  if (out > param_.outMax) {
    out = param_.outMax;
  } else if (out < -param_.outMax) {
    out = -param_.outMax;
  }

  prevErr_ = err;
  prevMeas_ = measured;
  hasPrev_ = true;
  return out;
}

void Pid::reset() {
  iSum_ = 0.0f;
  prevErr_ = 0.0f;
  prevMeas_ = 0.0f;
  hasPrev_ = false;
}

} // robot 命名空间结束
