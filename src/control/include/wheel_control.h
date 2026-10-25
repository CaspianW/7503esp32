/**
 * @file wheel_control.h
 * @brief Four-wheel speed controller interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../../common/robot_types.h"
#include "pid.h"

namespace robot {

// 负责四轮速度闭环；编码器测速和电机输出由应用层连接。
class WheelControl {
public:
  void configure(const PidParam &param);
  void configure(const PidParam (&param)[wheelCount]);
  void configure(WheelId wheel, const PidParam &param);
  // target、measured 的轮速单位为 m/s；dt 单位为 s。
  WheelValues update(const WheelValues &target,
                     const WheelValues &measured, float dt);
  void reset();

private:
  Pid pid_[wheelCount];
};

} // robot 命名空间结束
