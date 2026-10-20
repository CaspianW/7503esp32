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
  // body 须在 ROS base_link 坐标系；imuWz 单位 rad/s，dt 单位 s，timestamp 单位 ms。
  // 位置使用 body.vx/vy，航向和输出角速度使用 imuWz；body.wz 仅校验有效性。
  void update(const Twist2D &body, float imuWz,
              float dt, uint32_t timestamp);
  const OdometryState &state() const;

private:
  OdometryState state_{};
};

} // robot 命名空间结束
