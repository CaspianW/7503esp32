/**
 * @file servo_driver.h
 * @brief Low-level lift servo motor driver interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include <stdint.h>

namespace robot {

// 只负责舵机 PWM 输出，不判断升降机构是否真正到位。
class ServoDriver {
public:
  bool begin(uint8_t pin, uint32_t frequencyHz);
  void writePulseUs(uint16_t pulseUs);
  void detach();

private:
  bool initialized_ = false;
  uint8_t pin_ = 0;
  uint32_t frequencyHz_ = 0;
};

} // robot 命名空间结束
