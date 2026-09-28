#include "robot_controller.h"

namespace robot {

bool RobotController::begin() {
  // 目前只有框架；驱动、底盘尺寸、PID 参数和 micro-ROS 通信完成并校准前，
  // 保持机器人禁用。
  initialized_ = false;
  return initialized_;
}

void RobotController::update() {
  // 在这里实现非阻塞的周期控制循环。
}

} // robot 命名空间结束
