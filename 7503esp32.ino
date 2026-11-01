/**
 * @file 7503esp32.ino
 * @brief Robot startup, binary UART communication and fault status LED.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include <Arduino.h>

#include "src/app/robot_controller.h"
#include "src/communication/serial_link.h"
#include "src/config/board_pins.h"

robot::RobotController robotController;
robot::SerialLink serialLink;

void setup() {
  digitalWrite(robot::board::statusLed,
               robot::board::statusLedActiveLow ? HIGH : LOW);
  pinMode(robot::board::statusLed, OUTPUT);
  serialLink.begin();
  robotController.begin();
}

void loop() {
  // 先执行安全检查和控制，再处理有字节预算的串口输入。
  robotController.update();
  serialLink.update(robotController);
  // 健康时常亮，初始化失败/锁定故障时 2Hz 闪烁，状态可由串口定位。
  const bool healthy = robotController.initialized() &&
    robotController.fault() == robot::RobotFault::none;
  const bool on = healthy || (millis() / 250) % 2 == 0;
  digitalWrite(robot::board::statusLed,
    (on != robot::board::statusLedActiveLow) ? HIGH : LOW);
  delay(1); // 让出 CPU，控制周期仍由 micros() 调度
}
