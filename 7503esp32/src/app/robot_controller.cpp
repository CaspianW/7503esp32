#include "robot_controller.h"

namespace robot {

bool RobotController::begin() {
  // Scaffold only: keep the robot disabled until the drivers, geometry, PID
  // gains, and micro-ROS transport have been implemented and calibrated.
  initialized_ = false;
  return initialized_;
}

void RobotController::update() {
  // The non-blocking periodic control loop will be implemented here.
}

} // namespace robot
