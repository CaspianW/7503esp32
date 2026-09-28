#include "src/app/robot_controller.h"

robot::RobotController robotController;
bool robotControllerReady = false;

void setup() {
  // The USB serial port will be used by micro-ROS. Avoid writing debug text
  // to it once the micro-ROS transport is enabled.
  robotControllerReady = robotController.begin();
}

void loop() {
  if (robotControllerReady) {
    robotController.update();
  }
}
