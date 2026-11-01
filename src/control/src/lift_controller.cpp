/**
 * @file lift_controller.cpp
 * @brief Calibrated lift endpoint commands.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../include/lift_controller.h"
#include "../../config/robot_config.h"

namespace robot {
namespace {

constexpr uint16_t minPulse = 500; // 单位 µs
constexpr uint16_t maxPulse = 2500; // 单位 µs

} // 匿名命名空间结束

bool LiftController::begin(ServoDriver &servo, uint16_t loweredPulse,
                           uint16_t raisedPulse) {
  servo_ = nullptr;
  // 仅接受与低层驱动相同、已校准的端点，初始化不产生动作。
  if (loweredPulse < minPulse || loweredPulse > maxPulse ||
      raisedPulse < minPulse || raisedPulse > maxPulse || loweredPulse == raisedPulse ||
      loweredPulse != config::liftLoweredPulse ||
      raisedPulse != config::liftRaisedPulse) return false;
  servo_ = &servo;
  targetApplied_ = false;
  moving_ = false;
  loweredPulse_ = loweredPulse;
  raisedPulse_ = raisedPulse;
  return true;
}

void LiftController::request(LiftPosition target, uint32_t nowMs) {
  if (!servo_ || (target != LiftPosition::lowered && target != LiftPosition::raised)) return;
  if (target == LiftPosition::lowered) servo_->writePulse(loweredPulse_);
  if (target == LiftPosition::raised) servo_->writePulse(raisedPulse_);
  // 重复的相同目标不重启计时；改变方向则重新等待完整动作时间。
  if (targetApplied_ && target_ == target) return;
  target_ = target;
  targetApplied_ = true;
  moving_ = true;
  startedAtMs_ = nowMs;
}

void LiftController::update(uint32_t nowMs) {
  if (moving_ && static_cast<uint32_t>(nowMs - startedAtMs_) >= config::liftTravelTimeMs) {
    completed_ = target_;
    completionKnown_ = true;
    moving_ = false;
  }
}

void LiftController::stop() {
  if (servo_) servo_->detach();
  servo_ = nullptr;
  targetApplied_ = false;
  moving_ = false; // 停止动作不会把未完成的目标标记为完成。
}

} // namespace robot
