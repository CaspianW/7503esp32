/**
 * @file servo_driver.h
 * @brief Low-level lift servo motor driver interface.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include <stdint.h>

namespace robot {

// 只负责舵机 PWM 输出，不判断升降机构是否真正到位。
class ServoDriver {
public:
  bool begin(uint8_t pin, uint32_t frequency); // frequency 单位 Hz
  void writePulse(uint16_t pulse); // pulse 单位 µs
  void detach();

private:
  bool initialized_ = false;
  uint8_t pin_ = 0;
  uint32_t frequency_ = 0; // 单位 Hz
};

} // robot 命名空间结束
