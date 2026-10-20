/**
 * @file safety.cpp
 * @brief Command-timeout and emergency-stop logic.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "safety.h"

#include "../config/robot_config.h"

namespace robot {

void Safety::begin() {
  lastCommand_ = 0;
  hasCommand_ = false;
  emergencyStop_ = false;
  commandTimedOut_ = true;
}

void Safety::noteVelocityCommand(uint32_t timestamp) {
  if (emergencyStop_) return;
  lastCommand_ = timestamp;
  hasCommand_ = true;
  commandTimedOut_ = false;
}

void Safety::setEmergencyStop(bool active) {
  emergencyStop_ = active;
  if (active) {
    // 急停解除后也不能沿用旧指令，必须重新收到速度命令。
    hasCommand_ = false;
    commandTimedOut_ = true;
  }
}

void Safety::update(uint32_t now) {
  // 无符号减法兼容 millis() 回绕；恰好达到超时值时禁止运动。
  commandTimedOut_ = !hasCommand_ ||
                     now - lastCommand_ >= config::commandTimeout;
}

bool Safety::motionAllowed() const {
  return !emergencyStop_ && hasCommand_ && !commandTimedOut_;
}

bool Safety::commandTimedOut() const {
  return commandTimedOut_;
}

} // robot 命名空间结束
