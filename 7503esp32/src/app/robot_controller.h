#pragma once

#include "../communication/ros_bridge.h"
#include "../control/mecanum.h"
#include "../control/odometry.h"
#include "../control/wheel_pid.h"
#include "../drivers/encoder.h"
#include "../drivers/imu.h"
#include "../drivers/motor_driver.h"
#include "safety.h"

namespace robot {

class RobotController {
public:
  bool begin();
  void update();

private:
  MotorDriver motorDriver_;
  Encoder encoder_;
  WheelPid wheelPid_[kWheelCount];
  Imu imu_;
  MecanumKinematics kinematics_;
  Odometry odometry_;
  RosBridge rosBridge_;
  Safety safety_;

  bool initialized_;
};

} // robot 命名空间结束
