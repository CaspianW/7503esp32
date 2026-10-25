/**
 * @file mecanum.h
 * @brief Mecanum-wheel kinematics interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../../common/robot_types.h"

namespace robot {

class MecanumKinematics {
public:
  MecanumKinematics();
  // base 为前后轮中心距，track 为左右轮中心距，单位均为 m。
  bool configure(float base, float track);

  // X 形滚轮；逻辑轮序为左前、左后、右前、右后，实际通道待实车核对。
  // 车体速度采用 x 向前、y 向左、绕 z 逆时针为正；车轮速度单位为 m/s。
  bool inverse(const Twist2D &body, WheelValues &wheel) const;
  bool forward(const WheelValues &wheel, Twist2D &body) const;
  // 等比例缩放四轮速度，使其绝对值不超过 max，单位 m/s。
  static void scaleToLimit(WheelValues &wheel, float max);

private:
  float arm_; // 旋转等效系数，单位 m；不是轮心到车体中心的直线距离
  bool configured_;
};

} // robot 命名空间结束
