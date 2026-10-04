/**
 * @file robot_controller.cpp
 * @brief Robot initialization and one-shot motor bench test.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "robot_controller.h"

#include "../config/board_pins.h"
#include "../config/robot_config.h"

#include <Arduino.h>

namespace robot {

bool RobotController::begin() {
  initialized_ = false;
  // 正常整车控制尚未启用；台架测试必须显式打开两个安全开关。
  if (!config::motorBenchEnabled || !config::motorOutputsEnabled) return false;
  if (config::motorBenchChannel >= board::motorCount ||
      !(config::motorBenchDuty > 0.0f && config::motorBenchDuty <= 0.2f) ||
      config::motorBenchRun == 0) return false;

  if (!motorDriver_.begin()) return false;
  motorDriver_.stopAll(false);
  motorBenchStart_ = millis();
  initialized_ = true;
  return true;
}

void RobotController::update() {
  if (!initialized_) return;

  const MotorBenchPhase phase = motorBenchPhase(
      motorBenchStart_, millis(), config::motorBenchWait,
      config::motorBenchRun);
  if (phase == MotorBenchPhase::wait) return;
  if (phase == MotorBenchPhase::run) {
    // 这里只使用 M1～M4 电气通道编号，不推断实际车轮位置。
    motorDriver_.setDuty(static_cast<WheelId>(config::motorBenchChannel),
                         config::motorBenchDuty);
    return;
  }

  motorDriver_.stopAll(false);
  initialized_ = false; // 一次性测试结束，本次上电不再重新启动。
}

} // robot 命名空间结束
