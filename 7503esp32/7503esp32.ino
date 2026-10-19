/**
 * @file 7503esp32.ino
 * @brief Arduino robot-control sketch and MCU status LED entry point.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include <Arduino.h>

#include "src/app/robot_controller.h"
#include "src/config/board_pins.h"

robot::RobotController robotController;
bool robotControllerReady = false;

void setup() {
  // 上电初始化阶段先熄灭 MCU 蓝灯；红色电源灯不由软件控制。
  digitalWrite(robot::board::statusLed,
               robot::board::statusLedActiveLow ? HIGH : LOW);
  pinMode(robot::board::statusLed, OUTPUT);
  // USB 串口将用于 micro-ROS 通信；启用后不要再向该串口输出调试文本。
  robotControllerReady = robotController.begin();
}

void loop() {
  // 进入主循环后常亮，与电机台架测试是否启用无关。
  digitalWrite(robot::board::statusLed,
               robot::board::statusLedActiveLow ? LOW : HIGH);
  if (robotControllerReady) {
    robotController.update();
  }
}
