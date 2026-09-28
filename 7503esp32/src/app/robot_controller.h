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
#include "../control/mecanum.h"
#include "../control/odometry.h"
#include "../control/lift_controller.h"
#include "../control/wheel_control.h"
#include "../drivers/encoder.h"
#include "../drivers/imu.h"
#include "../drivers/motor_driver.h"
#include "../drivers/servo_driver.h"
#include "safety.h"

namespace robot {

class RobotController {
public:
  bool begin();
  void update();

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

  bool initialized_;
};

} // robot 命名空间结束
