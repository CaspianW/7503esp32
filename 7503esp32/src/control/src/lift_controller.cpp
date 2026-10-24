/**
 * @file lift_controller.cpp
 * @brief Calibrated lift endpoint commands and explicit servo detach.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../include/lift_controller.h"
#include "../../config/robot_config.h"

namespace robot {

bool LiftController::begin(ServoDriver &servo, uint16_t loweredPulse,
                           uint16_t raisedPulse) {
  servo_ = nullptr;
  // 仅接受与低层驱动相同、已校准的端点，初始化不产生动作。
  if (loweredPulse < 500 || loweredPulse > 2500 ||
      raisedPulse < 500 || raisedPulse > 2500 || loweredPulse == raisedPulse ||
      loweredPulse != config::liftLoweredPulse ||
      raisedPulse != config::liftRaisedPulse) return false;
  servo_ = &servo;
  loweredPulse_ = loweredPulse;
  raisedPulse_ = raisedPulse;
  return true;
}

void LiftController::request(LiftPosition target) {
  if (!servo_) return;
  if (target == LiftPosition::lowered) servo_->writePulse(loweredPulse_);
  if (target == LiftPosition::raised) servo_->writePulse(raisedPulse_);
}

void LiftController::stop() {
  if (servo_) servo_->detach();
  servo_ = nullptr;
}

} // namespace robot
