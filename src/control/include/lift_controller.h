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

// 输出标定脉宽，经过设定动作时间后更新完成标志；没有实际位置反馈。
class LiftController {
public:
  // loweredPulse、raisedPulse 单位为 µs。
  bool begin(ServoDriver &servo, uint16_t loweredPulse,
             uint16_t raisedPulse);
  void request(LiftPosition target, uint32_t nowMs);
  void update(uint32_t nowMs);
  void stop();
  bool completionKnown() const { return completionKnown_; }
  uint8_t completedState() const { return completed_ == LiftPosition::raised ? 1 : 0; }

private:
  ServoDriver *servo_ = nullptr;
  uint16_t loweredPulse_ = 0; // 单位 µs
  uint16_t raisedPulse_ = 0;  // 单位 µs
  LiftPosition target_ = LiftPosition::lowered;
  LiftPosition completed_ = LiftPosition::lowered;
  uint32_t startedAtMs_ = 0;
  bool targetApplied_ = false;
  bool moving_ = false;
  bool completionKnown_ = false;
};

} // robot 命名空间结束
