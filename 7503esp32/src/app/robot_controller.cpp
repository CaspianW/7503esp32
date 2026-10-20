/**
 * @file robot_controller.cpp
 * @brief Safe robot startup and isolated one-shot motor bench test.
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
  // 禁止再次初始化绕过本次运行中已触发的软件急停。
  if (stopLatched_) return false;
  initialized_ = false;
  benchMode_ = config::motorBenchEnabled;
  safety_.begin();

  // 台架测试仍需两个安全开关同时打开；正常模式默认只会停机。
  if (benchMode_ &&
      (!config::motorOutputsEnabled ||
       config::motorBenchChannel >= board::motorCount ||
       !(config::motorBenchDuty > 0.0f && config::motorBenchDuty <= 0.2f) ||
       config::motorBenchRun == 0)) return false;

  if (!motorDriver_.begin()) return false;
  motorDriver_.stopAll(false);
  if (benchMode_) motorBenchStart_ = millis();
  initialized_ = true;
  return true;
}

void RobotController::update() {
  if (!initialized_) return;

  if (!benchMode_) {
    safety_.update(millis());
    if (!safety_.motionAllowed()) {
      motorDriver_.stopAll(false);
      return;
    }
    // 编码器闭环尚未接入；收到新指令后也不产生非零电机输出。
    motorDriver_.stopAll(false);
    return;
  }

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

void RobotController::noteCmdVel(uint32_t timestamp) {
  if (initialized_ && !benchMode_) safety_.noteVelocityCommand(timestamp);
}

void RobotController::setEmergencyStop(bool active) {
  safety_.setEmergencyStop(active);
  if (!active) return;

  stopLatched_ = true;
  motorDriver_.stopAll(false);
  // 台架测试是一次性的；急停解除后不继续未完成的测试。
  if (benchMode_) initialized_ = false;
}

bool RobotController::motionAllowed() const {
  return initialized_ && !benchMode_ && safety_.motionAllowed();
}

} // robot 命名空间结束
