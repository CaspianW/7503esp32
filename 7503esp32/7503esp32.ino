#include "src/app/robot_controller.h"

robot::RobotController robotController;
bool robotControllerReady = false;

void setup() {
  // USB 串口将用于 micro-ROS 通信；启用后不要再向该串口输出调试文本。
  robotControllerReady = robotController.begin();
}

void loop() {
  if (robotControllerReady) {
    robotController.update();
  }
}
