#pragma once

#include "../lib/encoder.h"
#include "../lib/imu.h"
#include "../lib/mecanum.h"
#include "../lib/motor_driver.h"
#include "../lib/odometry.h"
#include "../lib/wheel_pid.h"
#include "ros_bridge.h"
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

} // namespace robot
