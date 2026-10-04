/**
 * @file motor_driver.cpp
 * @brief Four-channel motor driver.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../include/motor_driver.h"

#include "../../config/board_pins.h"
#include "../../config/robot_config.h"

#include <Arduino.h>
#include <math.h>

namespace robot {
namespace {

constexpr uint8_t pwmResolution = 10; // 单位 bit
constexpr uint32_t pwmMaxDuty = (1U << pwmResolution) - 1U;

} // 匿名命名空间结束

bool MotorDriver::begin() {
  if (initialized_) {
    return true;
  }

  // 先将 H 桥两路输入拉低，再配置 PWM；启动时绝不产生运动命令。
  for (uint8_t i = 0; i < wheelCount; ++i) {
    const board::MotorPins &pins = board::motorPins[i];
    pinMode(pins.pwmA, OUTPUT);
    pinMode(pins.pwmB, OUTPUT);
    digitalWrite(pins.pwmA, LOW);
    digitalWrite(pins.pwmB, LOW);
    activePin_[i] = -1;
  }

  // 每个电机共用一路 PWM，按转向把该通道切换到 A 或 B 引脚。
  // ESP32-S3 只有 8 路 LEDC，预留通道 4 给舵机。
  for (uint8_t i = 0; i < wheelCount; ++i) {
    if (ledcSetup(i, config::motorPwmFrequency, pwmResolution) == 0) {
      return false;
    }
    ledcWrite(i, 0);
  }
  initialized_ = true;
  return true;
}

void MotorDriver::setDuty(WheelId wheel, float signedDuty) {
  const uint8_t index = static_cast<uint8_t>(wheel);
  if (!initialized_ || index >= wheelCount) {
    return;
  }

  if (!config::motorOutputsEnabled || !isfinite(signedDuty)) {
    signedDuty = 0.0f;
  }
  const int8_t direction = config::motorDirection[index];
  signedDuty *= direction;
  if (signedDuty > 1.0f) signedDuty = 1.0f;
  if (signedDuty < -1.0f) signedDuty = -1.0f;

  const board::MotorPins &pins = board::motorPins[index];
  // Yahboom 示例把 MxB 作为正向 PWM、MxA 作为反向 PWM。
  const int8_t desiredPin = signedDuty > 0.0f ? pins.pwmB : pins.pwmA;
  if (signedDuty == 0.0f) {
    ledcWrite(index, 0);
    return;
  }

  if (activePin_[index] != desiredPin) {
    ledcWrite(index, 0);
    if (activePin_[index] >= 0) {
      ledcDetachPin(activePin_[index]);
      digitalWrite(activePin_[index], LOW);
    }
    digitalWrite(desiredPin, LOW);
    ledcAttachPin(desiredPin, index);
    activePin_[index] = desiredPin;
  }
  const uint32_t duty = static_cast<uint32_t>(fabsf(signedDuty) * pwmMaxDuty);
  ledcWrite(index, duty);
}

void MotorDriver::setAllDuty(const WheelValues &signedDuty) {
  for (uint8_t i = 0; i < wheelCount; ++i) {
    setDuty(static_cast<WheelId>(i), signedDuty.value[i]);
  }
}

void MotorDriver::stopAll(bool brake) {
  // 板载 H 桥的主动制动电平尚未实测；一律采用两路低电平停机。
  (void)brake;
  for (uint8_t i = 0; i < wheelCount; ++i) {
    setDuty(static_cast<WheelId>(i), 0.0f);
  }
}

} // robot 命名空间结束
