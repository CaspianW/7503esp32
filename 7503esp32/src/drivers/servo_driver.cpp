/**
 * @file servo_driver.cpp
 * @brief Low-level lift servo motor driver.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "servo_driver.h"

#include "../config/board_pins.h"
#include "../config/robot_config.h"

#include <Arduino.h>

namespace robot {
namespace {

constexpr uint8_t servoPwmChannel = 4;
constexpr uint8_t servoPwmResolutionBits = 14;
constexpr uint16_t absoluteMinPulseUs = 500;
constexpr uint16_t absoluteMaxPulseUs = 2500;

} // 匿名命名空间结束

bool ServoDriver::begin(uint8_t pin, uint32_t frequencyHz) {
  if (initialized_) {
    return pin == pin_ && frequencyHz == frequencyHz_;
  }

  const uint16_t lower = config::liftLoweredPulseUs;
  const uint16_t upper = config::liftRaisedPulseUs;
  if ((pin != board::servoS1 && pin != board::servoS2) ||
      frequencyHz != config::servoPwmFrequencyHz ||
      lower < absoluteMinPulseUs || lower > absoluteMaxPulseUs ||
      upper < absoluteMinPulseUs || upper > absoluteMaxPulseUs ||
      lower == upper) {
    // 未标定升降端点时不启用舵机输出。
    return false;
  }

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  if (ledcSetup(servoPwmChannel, frequencyHz, servoPwmResolutionBits) == 0) {
    return false;
  }
  ledcWrite(servoPwmChannel, 0);
  ledcAttachPin(pin, servoPwmChannel);
  pin_ = pin;
  frequencyHz_ = frequencyHz;
  initialized_ = true;
  return true;
}

void ServoDriver::writePulseUs(uint16_t pulseUs) {
  if (!initialized_) {
    return;
  }
  const uint16_t lower = config::liftLoweredPulseUs;
  const uint16_t upper = config::liftRaisedPulseUs;
  const uint16_t minimum = lower < upper ? lower : upper;
  const uint16_t maximum = lower > upper ? lower : upper;
  if (pulseUs < minimum) pulseUs = minimum;
  if (pulseUs > maximum) pulseUs = maximum;

  const uint64_t periodScale = 1ULL << servoPwmResolutionBits;
  const uint32_t duty = static_cast<uint32_t>(
      (static_cast<uint64_t>(pulseUs) * frequencyHz_ * periodScale) /
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
