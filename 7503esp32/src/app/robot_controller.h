/**
 * @file robot_controller.h
 * @brief Robot scheduling, telemetry, arming, motor bench and latched fault handling.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../control/include/chassis_control.h"
#include "../control/include/odometry.h"
#include "../control/include/wheel_speed.h"
#include "../drivers/include/encoder.h"
#include "../drivers/include/imu.h"
#include "../drivers/include/motor_driver.h"
#include "../control/include/lift_controller.h"
#include "safety.h"

namespace robot {

enum class MotorBenchPhase : uint8_t { wait, run, done };
inline MotorBenchPhase motorBenchPhase(uint32_t start, uint32_t now,
                                      uint32_t wait, uint32_t run) {
  const uint32_t elapsed = now - start;
  if (elapsed < wait) return MotorBenchPhase::wait;
  if (elapsed - wait < run) return MotorBenchPhase::run;
  return MotorBenchPhase::done;
}

enum class RobotFault : uint8_t {
  none, configuration, motor, encoder, timing, feedback, imu, lift
};

class RobotController {
public:
  bool begin();
  void update();
  bool arm();
  void stop(); // 立即停机、解除使能并丢弃旧指令
  bool commandVelocity(const Twist2D &command);
  bool armLift();
  bool commandLift(LiftPosition target);
  bool keepLiftAlive();
  void stopLift();
  bool liftConfigured() const;
  bool liftArmed() const { return liftArmed_; }
  bool liftActive() const { return liftActive_; }
  uint8_t liftTarget() const { return liftActive_ ? static_cast<uint8_t>(liftTarget_) : 2; }
  uint16_t liftPulse() const;
  void setEmergencyStop(bool active);
  bool motionAllowed() const;
  bool initialized() const { return initialized_; }
  bool armed() const { return armed_; }
  bool emergencyStopped() const { return emergencyStop_; }
  bool benchMode() const { return benchMode_; }
  bool encoderReady() const { return encoder_.healthy(); }
  bool imuReady() const;
  bool speedReady() const { return speedReady_; }
  RobotFault fault() const { return fault_; }
  const int64_t *ticks() const { return ticks_; } // M1～M4 电气顺序
  const WheelValues &wheelSpeed() const { return speed_; } // 逻辑轮序
  const WheelValues &duty() const { return duty_; } // 逻辑轮序
  const OdometryState &odometry() const { return odometry_.state(); }
  const ImuSample &imuSample() const { return imuSample_; } // 芯片坐标系

private:
  void halt();
  void stopChassis();
  void fail(RobotFault fault);
  bool physicalStop() const;
  bool feedbackPlausible(uint32_t now);
  MotorDriver motorDriver_;
  Encoder encoder_;
  WheelSpeed wheelSpeed_;
  ChassisControl chassis_;
  MecanumKinematics kinematics_;
  Odometry odometry_;
  Imu imu_;
  Safety safety_;
  ServoDriver servoDriver_;
  LiftController liftController_;
  LiftPosition liftTarget_ = LiftPosition::lowered;
  bool liftArmed_ = false;
  bool liftActive_ = false;
  uint32_t lastLiftCommand_ = 0; // 单位 ms
  Twist2D command_{};
  WheelValues speed_{};
  WheelValues duty_{};
  ImuSample imuSample_{};
  int64_t ticks_[wheelCount] = {};
  uint32_t badFeedbackSince_[wheelCount] = {}; // 单位 ms
  bool badFeedback_[wheelCount] = {};
  RobotFault fault_ = RobotFault::none;
  bool started_ = false;
  bool initialized_ = false;
  bool benchMode_ = false;
  bool benchDone_ = false;
  bool armed_ = false;
  bool commandReceived_ = false;
  bool emergencyStop_ = false;
  bool stopLatched_ = false;
  bool imuStarted_ = false;
  bool hasImuSample_ = false;
  bool speedReady_ = false;
  uint32_t motorBenchStart_ = 0; // 单位 ms
  uint32_t lastControl_ = 0; // 单位 µs
  uint32_t lastImu_ = 0; // 单位 µs
};

} // namespace robot
