/**
 * @file servo_driver.cpp
 * @brief Low-level lift servo motor driver.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../include/servo_driver.h"

#include "../../config/board_pins.h"
#include "../../config/robot_config.h"

#include <Arduino.h>

namespace robot {
namespace {

constexpr uint8_t servoPwmChannel = 4;
constexpr uint8_t servoPwmResolution = 14; // 单位 bit
constexpr uint16_t absoluteMinPulse = 500; // 单位 µs
constexpr uint16_t absoluteMaxPulse = 2500; // 单位 µs

} // 匿名命名空间结束

bool ServoDriver::begin(uint8_t pin, uint32_t frequency) {
  if (!config::liftOutputsEnabled) return false;
  if (initialized_) {
    return pin == pin_ && frequency == frequency_;
  }

  const uint16_t lower = config::liftLoweredPulse;
  const uint16_t upper = config::liftRaisedPulse;
  if ((pin != board::servoS1 && pin != board::servoS2) ||
      frequency != config::servoPwmFrequency ||
      lower < absoluteMinPulse || lower > absoluteMaxPulse ||
      upper < absoluteMinPulse || upper > absoluteMaxPulse ||
      lower == upper) {
    // 未标定升降端点时不启用舵机输出。
    return false;
  }

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  if (ledcSetup(servoPwmChannel, frequency, servoPwmResolution) == 0) {
    return false;
  }
  ledcWrite(servoPwmChannel, 0);
  ledcAttachPin(pin, servoPwmChannel);
  minimumPulse_ = lower < upper ? lower : upper;
  maximumPulse_ = lower > upper ? lower : upper;
  pin_ = pin;
  frequency_ = frequency;
  initialized_ = true;
  return true;
}

void ServoDriver::writePulse(uint16_t pulse) {
  if (!initialized_) {
    return;
  }
  if (pulse < minimumPulse_) pulse = minimumPulse_;
  if (pulse > maximumPulse_) pulse = maximumPulse_;

  const uint64_t periodScale = 1ULL << servoPwmResolution;
  const uint32_t duty = static_cast<uint32_t>(
      (static_cast<uint64_t>(pulse) * frequency_ * periodScale) /
      1000000ULL);
  ledcWrite(servoPwmChannel, duty);
}

void ServoDriver::detach() {
  if (!initialized_) {
    return;
  }
  ledcWrite(servoPwmChannel, 0);
  ledcDetachPin(pin_);
  digitalWrite(pin_, LOW);
  initialized_ = false;
}

} // robot 命名空间结束
