/**
 * @file robot_controller.h
 * @brief Top-level robot controller interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../communication/ros_bridge.h"
#include "../control/include/mecanum.h"
#include "../control/include/odometry.h"
#include "../control/include/lift_controller.h"
#include "../control/include/wheel_control.h"
#include "../drivers/include/encoder.h"
#include "../drivers/include/imu.h"
#include "../drivers/include/motor_driver.h"
#include "../drivers/include/servo_driver.h"
#include "safety.h"

namespace robot {

enum class MotorBenchPhase : uint8_t {
  wait,
  run,
  done
};

// 只根据上电后的经过时间决定测试阶段；无符号减法兼容 millis() 回绕。
// start、now、wait、run 的单位均为 ms。
inline MotorBenchPhase motorBenchPhase(uint32_t start, uint32_t now,
                                      uint32_t wait, uint32_t run) {
  const uint32_t elapsed = now - start; // 单位 ms
  if (elapsed < wait) return MotorBenchPhase::wait;
  if (elapsed - wait < run) return MotorBenchPhase::run;
  return MotorBenchPhase::done;
}

class RobotController {
public:
  bool begin();
  void update();
  // 仅在上层验证速度指令后调用；timestamp 单位 ms，不直接驱动电机。
  void noteCmdVel(uint32_t timestamp);
  // 软件急停入口；触发时立即停机，解除后仍需新的速度指令。
  // 急停后同一实例不能再次 begin()，以免重启台架测试。
  void setEmergencyStop(bool active);
  bool motionAllowed() const;

private:
  MotorDriver motorDriver_;
  Encoder encoder_;
  WheelControl wheelControl_;
  ServoDriver servoDriver_;
  LiftController liftController_;
  Imu imu_;
  MecanumKinematics kinematics_;
  Odometry odometry_;
  RosBridge rosBridge_;
  Safety safety_;

  bool initialized_ = false;
  bool benchMode_ = false;
  bool stopLatched_ = false;
  uint32_t motorBenchStart_ = 0; // 单位 ms
};

} // robot 命名空间结束
