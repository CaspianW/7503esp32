/**
 * @file motor_driver.h
 * @brief Four-channel motor driver interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../../common/robot_types.h"

namespace robot {

struct PwmState {
  int8_t pin = -1; // 驱动记录的挂载引脚，-1 表示已脱离
  uint32_t duty = 0; // LEDC回读值，10位PWM范围0～1024
  uint32_t freq = 0; // 单位 Hz
  int16_t route = -1; // GPIO矩阵实际信号编号
  int8_t out = -1; // LEDC输出使能
  int8_t gpio = -1; // GPIO输出使能寄存器位
  int8_t pause = -1; // 定时器暂停位；-1 表示平台不支持回读
};

class MotorDriver {
public:
  bool begin();
  void setDuty(WheelId wheel, float signedDuty);
  void setAllDuty(const WheelValues &signedDuty);
  void stopAll(bool brake);
  PwmState pwm(uint8_t channel) const;

private:
  bool initialized_ = false;
  int8_t activePin_[wheelCount] = {-1, -1, -1, -1};
};

} // robot 命名空间结束
