/**
 * @file ros_bridge.h
 * @brief ESP32-to-Raspberry Pi micro-ROS interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../common/robot_types.h"

namespace robot {

class RosBridge {
public:
  bool begin();
  void spinOnce();
  bool takeVelocityCommand(Twist2D &command);
  bool takeLiftCommand(LiftPosition &target);
  void publishOdometry(const OdometryState &odometry);
  void publishImu(const ImuSample &sample);
  bool connected() const;
};

} // robot 命名空间结束
