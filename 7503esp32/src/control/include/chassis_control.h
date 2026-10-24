/**
 * @file chassis_control.h
 * @brief Validated mecanum speed control with wheel limits and acceleration limiting.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "mecanum.h"
#include "wheel_control.h"

namespace robot {

struct ChassisParam {
  float base; // 单位 m
  float track; // 单位 m
  float maxLinear; // 单位 m/s，平移速度矢量模长
  float maxAngular; // 单位 rad/s
  float maxWheel; // 单位 m/s
  float acceleration; // 单位 m/s²，四轮设定值变化率
  PidParam pid;
};

class ChassisControl {
public:
  bool configure(const ChassisParam &param);
  bool update(const Twist2D &command, const WheelValues &measured,
              float dt, WheelValues &duty);
  void reset();
  const WheelValues &target() const { return target_; }
private:
  ChassisParam param_{};
  MecanumKinematics kinematics_;
  WheelControl wheels_;
  WheelValues target_{};
  bool ready_ = false;
};

} // namespace robot
