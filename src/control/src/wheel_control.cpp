/**
 * @file wheel_control.cpp
 * @brief Four-wheel speed control logic.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../include/wheel_control.h"

namespace robot {

void WheelControl::configure(const PidParam &param) {
  for (uint8_t i = 0; i < wheelCount; ++i) {
    pid_[i].configure(param);
  }
}

void WheelControl::configure(const PidParam (&param)[wheelCount]) {
  for (uint8_t i = 0; i < wheelCount; ++i) {
    pid_[i].configure(param[i]);
  }
}

void WheelControl::configure(WheelId wheel, const PidParam &param) {
  const uint8_t index = static_cast<uint8_t>(wheel);
  if (index < wheelCount) {
    pid_[index].configure(param);
  }
}

WheelValues WheelControl::update(const WheelValues &target,
                                 const WheelValues &measured,
                                 float dt) {
  WheelValues duty{};
  for (uint8_t i = 0; i < wheelCount; ++i) {
    duty.value[i] = pid_[i].update(target.value[i], measured.value[i], dt);
  }
  return duty;
}

void WheelControl::reset() {
  for (uint8_t i = 0; i < wheelCount; ++i) {
    pid_[i].reset();
  }
}

} // robot 命名空间结束
