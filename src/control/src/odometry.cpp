/**
 * @file odometry.cpp
 * @brief Chassis odometry integration.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../include/odometry.h"

#include <math.h>

namespace robot {

namespace {

float wrapYaw(float yaw) {
  const float pi = 3.14159265358979323846f;
  float wrapped = remainderf(yaw, 2.0f * pi); // 单位 rad
  if (wrapped >= pi) wrapped -= 2.0f * pi;
  return wrapped;
}

} // 匿名命名空间结束

void Odometry::reset(float x, float y, float yaw) {
  if (!isfinite(x) || !isfinite(y) || !isfinite(yaw)) return;
  state_ = {x, y, wrapYaw(yaw), 0.0f, 0.0f, 0.0f, 0};
}

void Odometry::update(const Twist2D &body, float imuWz,
                      float dt, uint32_t timestamp) {
  if (!isfinite(body.vx) || !isfinite(body.vy) ||
      !isfinite(body.wz) || !isfinite(imuWz) ||
      !isfinite(dt) || dt <= 0.0f) {
    return;
  }

  const float yawStep = imuWz * dt; // 单位 rad
  if (!isfinite(yawStep)) return;
  const float yawMid = state_.yaw + yawStep * 0.5f; // 单位 rad
  if (!isfinite(yawMid)) return;
  const float cosYaw = cosf(yawMid);
  const float sinYaw = sinf(yawMid);
  OdometryState next = state_;
  next.x += (body.vx * cosYaw - body.vy * sinYaw) * dt;
  next.y += (body.vx * sinYaw + body.vy * cosYaw) * dt;
  next.yaw = wrapYaw(state_.yaw + yawStep);
  if (!isfinite(next.x) || !isfinite(next.y) || !isfinite(next.yaw)) return;
  next.vx = body.vx;
  next.vy = body.vy;
  next.wz = imuWz;
  next.timestamp = timestamp;
  state_ = next;
}

const OdometryState &Odometry::state() const { return state_; }

} // robot 命名空间结束
