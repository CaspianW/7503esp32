/**
 * @file lift_controller.h
 * @brief Lift mechanism controller interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../../common/robot_types.h"
#include "../../drivers/include/servo_driver.h"

namespace robot {

// 将升降目标映射为已校准的舵机脉宽；不把指令位置当作实测位置。
class LiftController {
public:
  // loweredPulse、raisedPulse 单位为 µs。
  bool begin(ServoDriver &servo, uint16_t loweredPulse,
             uint16_t raisedPulse);
  void request(LiftPosition target);
  void stop();

private:
  ServoDriver *servo_ = nullptr;
  uint16_t loweredPulse_ = 0; // 单位 µs
  uint16_t raisedPulse_ = 0;  // 单位 µs
};

} // robot 命名空间结束
