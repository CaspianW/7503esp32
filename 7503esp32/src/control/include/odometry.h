/**
 * @file odometry.h
 * @brief Chassis odometry interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../../common/robot_types.h"

namespace robot {

class Odometry {
public:
  // x、y 的单位为 m，yaw 的单位为 rad。
  void reset(float x = 0.0f, float y = 0.0f, float yaw = 0.0f);
  // imuYawRate 单位 rad/s，dt 单位 s，timestamp 单位 ms。
  void update(const Twist2D &bodyVelocity, float imuYawRate,
              float dt, uint32_t timestamp);
  const OdometryState &state() const;

private:
  OdometryState state_;
};

} // robot 命名空间结束
