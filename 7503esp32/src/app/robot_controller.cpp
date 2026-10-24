/**
 * @file robot_controller.cpp
 * @brief Scheduled encoder feedback, odometry, chassis control and immediate motor shutdown.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "robot_controller.h"

#include "../config/board_pins.h"
#include "../config/robot_config.h"
#include <Arduino.h>
#include <math.h>

namespace robot {
namespace {
bool validMapping() {
  bool used[wheelCount] = {};
  for (uint8_t channel : config::wheelChannel) {
    if (channel >= wheelCount || used[channel]) return false;
    used[channel] = true;
  }
  return true;
}
bool validStopPin() {
  const int pin = config::emergencyStopPin;
  // 仅允许板上未被本固件使用的引脚；接线前仍需核对板卡版本。
  return pin == -1 || pin == 17 || pin == 18 || pin == 21 || pin == 42;
}
} // namespace

bool RobotController::begin() {
  // 任何 begin 尝试后都不可重启一次性台架或清除故障。
  if (started_ || stopLatched_) return false;
  started_ = true;
  safety_.begin();
  benchMode_ = config::motorBenchEnabled;
  if (!motorDriver_.begin()) { fail(RobotFault::motor); return false; }
  halt();
  if (!validMapping() || !validStopPin() || config::controlPeriod == 0 ||
      config::controlPeriod > config::maxControlGap ||
      config::maxControlGap > 50000 ||
      (config::motorBenchEnabled && config::chassisControlEnabled) ||
      (benchMode_ && (!config::motorOutputsEnabled ||
        config::motorBenchChannel >= board::motorCount ||
        !isfinite(config::motorBenchDuty) || config::motorBenchDuty == 0.0f ||
        fabsf(config::motorBenchDuty) > 0.2f ||
        fabsf(config::motorBenchDuty) > config::motorDutyLimit ||
        config::motorBenchRun == 0 || config::motorBenchRun > 500 ||
        config::motorBenchWait < 2000))) {
    fail(RobotFault::configuration); return false;
  }
  if (config::emergencyStopPin >= 0)
    pinMode(config::emergencyStopPin, INPUT_PULLUP);
  if (physicalStop()) setEmergencyStop(true);
  // 诊断和台架都读取编码器；未初始化成功时绝不允许运动。
  if (!encoder_.begin() ||
      !wheelSpeed_.configure(config::encoderCounts, config::wheelRadius)) {
    fail(RobotFault::encoder); return false;
  }
  const bool geometryReady = kinematics_.configure(config::wheelbase, config::trackWidth);
  if (config::chassisControlEnabled) {
    ChassisParam param{config::wheelbase, config::trackWidth,
      config::maxLinearSpeed, config::maxAngularSpeed, config::maxWheelSpeed,
      config::maxWheelAcceleration,
      PidParam(config::wheelPidKp, config::wheelPidKi, config::wheelPidKd,
               config::wheelPidOutputLimit, config::wheelPidIntegralLimit, true)};
    if (!config::motorOutputsEnabled || !config::wheelMappingConfirmed ||
        !geometryReady || !chassis_.configure(param) ||
        config::wheelPidOutputLimit > config::motorDutyLimit) {
      fail(RobotFault::configuration); return false;
    }
  }
  imuStarted_ = !benchMode_ && imu_.begin();
  if (config::useImuYaw &&
      (!imuStarted_ || (config::imuYawDirection != 1 && config::imuYawDirection != -1))) {
    fail(RobotFault::imu); return false;
  }
  if (!encoder_.readAllTicks(ticks_)) { fail(RobotFault::encoder); return false; }
  int64_t logical[wheelCount] = {};
  for (uint8_t i = 0; i < wheelCount; ++i) logical[i] = ticks_[config::wheelChannel[i]];
  lastControl_ = micros(); lastImu_ = lastControl_;
  wheelSpeed_.update(logical, lastControl_, speed_);
  motorBenchStart_ = millis();
  initialized_ = true;
  return true;
}

bool RobotController::physicalStop() const {
  return config::emergencyStopPin >= 0 &&
    (digitalRead(config::emergencyStopPin) == (config::emergencyStopActiveHigh ? HIGH : LOW));
}

bool RobotController::feedbackPlausible(uint32_t now) {
  for (uint8_t i = 0; i < wheelCount; ++i) {
    const float target = chassis_.target().value[i];
    const float measured = speed_.value[i];
    const bool bad = fabsf(duty_.value[i]) >= config::feedbackMinDuty &&
      fabsf(target) >= config::feedbackMinTarget &&
      (fabsf(measured) < config::feedbackMinSpeed || target * measured < 0.0f);
    if (!bad) { badFeedback_[i] = false; continue; }
    if (!badFeedback_[i]) { badFeedback_[i] = true; badFeedbackSince_[i] = now; }
    if (static_cast<uint32_t>(now - badFeedbackSince_[i]) >= config::feedbackTimeout)
      return false;
  }
  return true;
}

void RobotController::halt() {
  motorDriver_.stopAll(false);
  duty_ = {}; command_ = {};
  chassis_.reset();
  for (bool &bad : badFeedback_) bad = false;
}

void RobotController::stopChassis() {
  halt(); armed_ = false; commandReceived_ = false;
  safety_.invalidateCommand();
  if (benchMode_) benchDone_ = true;
}

void RobotController::stop() {
  stopChassis();
  stopLift();
}

bool RobotController::liftConfigured() const {
  return config::liftServoChannel < 2 && config::servoPwmFrequency == 50 &&
    config::liftCommandTimeout > 0 && config::liftCommandTimeout <= 1000 &&
    config::liftLoweredPulse >= 500 && config::liftLoweredPulse <= 2500 &&
    config::liftRaisedPulse >= 500 && config::liftRaisedPulse <= 2500 &&
    config::liftLoweredPulse != config::liftRaisedPulse &&
    !(config::liftServoChannel == 1 && config::emergencyStopPin == board::servoS2);
}

uint16_t RobotController::liftPulse() const {
  if (!liftActive_) return 0;
  return liftTarget_ == LiftPosition::raised ? config::liftRaisedPulse : config::liftLoweredPulse;
}

void RobotController::stopLift() {
  liftController_.stop();
  servoDriver_.detach();
  liftArmed_ = false; liftActive_ = false;
}

bool RobotController::armLift() {
  if (!initialized_ || benchMode_ || emergencyStop_ || physicalStop() ||
      fault_ != RobotFault::none || !config::liftOutputsEnabled || !liftConfigured()) return false;
  // 重复使能不撤掉已有的保持PWM；也不会回放上次停机前的目标。
  if (!liftArmed_) {
    const uint8_t pin = config::liftServoChannel == 0 ? board::servoS1 : board::servoS2;
    if (!servoDriver_.begin(pin, config::servoPwmFrequency) ||
        !liftController_.begin(servoDriver_, config::liftLoweredPulse, config::liftRaisedPulse)) {
      fail(RobotFault::lift); return false;
    }
    liftArmed_ = true;
  }
  lastLiftCommand_ = millis();
  return true;
}

bool RobotController::keepLiftAlive() {
  if (!liftArmed_ || emergencyStop_ || physicalStop() || fault_ != RobotFault::none) return false;
  lastLiftCommand_ = millis();
  return true;
}

bool RobotController::commandLift(LiftPosition target) {
  if ((target != LiftPosition::lowered && target != LiftPosition::raised) || !keepLiftAlive()) return false;
  liftController_.request(target);
  liftTarget_ = target; liftActive_ = true;
  return true;
}

void RobotController::fail(RobotFault fault) {
  if (fault_ == RobotFault::none) fault_ = fault;
  stop();
}

bool RobotController::arm() {
  if (!initialized_ || fault_ != RobotFault::none || emergencyStop_ ||
      physicalStop() || benchMode_ || !config::chassisControlEnabled ||
      !config::motorOutputsEnabled || !encoder_.healthy() || !speedReady_ ||
      (config::useImuYaw && !imuReady())) return false;
  // 重复 arm 也丢弃旧速度，必须随后发送新命令。
  stopChassis(); armed_ = true;
  return true;
}

bool RobotController::commandVelocity(const Twist2D &command) {
  if (!isfinite(command.vx) || !isfinite(command.vy) || !isfinite(command.wz)) {
    stop(); return false;
  }
  if (!armed_ || emergencyStop_ || fault_ != RobotFault::none || physicalStop()) return false;
  command_ = command;
  commandReceived_ = true;
  safety_.noteVelocityCommand(millis());
  if (command.vx == 0.0f && command.vy == 0.0f && command.wz == 0.0f) halt();
  return true;
}

void RobotController::setEmergencyStop(bool active) {
  if (!active && physicalStop()) return;
  emergencyStop_ = active;
  safety_.setEmergencyStop(active);
  // 解除急停仍保持 disarmed；台架测试永久结束。
  stop();
  if (active) stopLatched_ = true;
}

bool RobotController::motionAllowed() const {
  return initialized_ && !benchMode_ && armed_ && fault_ == RobotFault::none &&
    !emergencyStop_ && safety_.motionAllowed();
}

bool RobotController::imuReady() const {
  return imuStarted_ && hasImuSample_ &&
    static_cast<uint32_t>(millis() - imuSample_.timestamp) < config::imuFreshness;
}

void RobotController::update() {
  if (!initialized_) return;
  if (physicalStop()) setEmergencyStop(true);
  if (liftArmed_ && static_cast<uint32_t>(millis() - lastLiftCommand_) >= config::liftCommandTimeout)
    stopLift();
  const uint32_t now = micros();
  const uint32_t elapsed = now - lastControl_;
  // 在任何 I2C 或串口操作前先检查超时并撤销输出。
  safety_.update(millis());
  if (!benchMode_ && !safety_.motionAllowed()) {
    // 尚未收到首帧 vel 时维持 armed；一旦已有输出超时就 disarm。
    const bool hadCommand = commandReceived_;
    halt();
    if (hadCommand) { armed_ = false; commandReceived_ = false; }
  }
  if (elapsed > config::maxControlGap) fail(RobotFault::timing);
  if (elapsed < config::controlPeriod) return;
  lastControl_ = now;
  if (!encoder_.readAllTicks(ticks_)) { fail(RobotFault::encoder); return; }
  int64_t logical[wheelCount] = {};
  for (uint8_t i = 0; i < wheelCount; ++i) logical[i] = ticks_[config::wheelChannel[i]];
  speedReady_ = wheelSpeed_.update(logical, now, speed_);
  if (!speedReady_) { fail(RobotFault::feedback); return; }
  const float dt = elapsed * 0.000001f;
  Twist2D body{};
  if (kinematics_.forward(speed_, body) && (!config::useImuYaw || imuReady()))
    odometry_.update(body, config::useImuYaw ? imuSample_.gyroZ * config::imuYawDirection : body.wz,
                     dt, millis());
  if (benchMode_) {
    const MotorBenchPhase phase = motorBenchPhase(motorBenchStart_, millis(),
      config::motorBenchWait, config::motorBenchRun);
    if (phase == MotorBenchPhase::done) benchDone_ = true;
    if (phase == MotorBenchPhase::run && !benchDone_ &&
        fault_ == RobotFault::none && !emergencyStop_) {
      motorDriver_.setDuty(static_cast<WheelId>(config::motorBenchChannel), config::motorBenchDuty);
      duty_ = {};
      for (uint8_t i = 0; i < wheelCount; ++i)
        if (config::wheelChannel[i] == config::motorBenchChannel) duty_.value[i] = config::motorBenchDuty;
    } else halt();
    return;
  }
  if (config::useImuYaw && !imuReady() && armed_) fail(RobotFault::imu);
  if (motionAllowed()) {
    if (!chassis_.update(command_, speed_, dt, duty_)) { fail(RobotFault::feedback); return; }
    if (!feedbackPlausible(millis())) { fail(RobotFault::feedback); return; }
    WheelValues electrical{};
    for (uint8_t i = 0; i < wheelCount; ++i) electrical.value[config::wheelChannel[i]] = duty_.value[i];
    motorDriver_.setAllDuty(electrical);
  }
  if (imuStarted_ && static_cast<uint32_t>(now - lastImu_) >= config::imuPeriod) {
    lastImu_ = now;
    ImuSample next{};
    if (imu_.read(next)) { imuSample_ = next; hasImuSample_ = true; }
  }
}

} // namespace robot
