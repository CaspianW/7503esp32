/**
 * @file robot_controller.cpp
 * @brief Scheduled chassis control, manual driving and lift pulse output.
 * @author Caspian Wu
 * @date 2026-10-26
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
  manualDuty_ = config::manualDuty;
  safety_.begin();
  benchMode_ = config::motorBenchEnabled;
  if (!motorDriver_.begin()) { fail(RobotFault::motor); return false; }
  halt();
  if (!validMapping() || !validStopPin() || config::controlPeriod == 0 ||
      config::controlPeriod > config::maxControlGap ||
      config::maxControlGap > 50000 ||
      (config::motorBenchEnabled && config::chassisControlEnabled) ||
      (benchMode_ && (!config::motorOutputsEnabled ||
        !isfinite(config::motorBenchDuty) || config::motorBenchDuty == 0.0f ||
        fabsf(config::motorBenchDuty) > config::motorDutyLimit)) ||
      (config::manualOpenLoopEnabled && (!config::motorOutputsEnabled ||
        !isfinite(config::manualDuty) || config::manualDuty <= 0.0f ||
        config::manualDuty > config::motorDutyLimit))) {
    fail(RobotFault::configuration); return false;
  }
  if (config::emergencyStopPin >= 0)
    pinMode(config::emergencyStopPin, INPUT_PULLUP);
  if (physicalStop()) setEmergencyStop(true);
  const bool geometryReady = kinematics_.configure(config::wheelbase, config::trackWidth);
  if ((config::chassisControlEnabled || config::manualOpenLoopEnabled) && !geometryReady) {
    fail(RobotFault::configuration); return false;
  }
  // 闭环始终需要编码器；开环可单独启用只读采样用于诊断和状态回传。
  const bool encoderRequired = config::chassisControlEnabled || config::encoderReadEnabled;
  if (encoderRequired && (!encoder_.begin() ||
      !wheelSpeed_.configure(config::encoderCounts, config::wheelRadius))) {
    fail(RobotFault::encoder); return false;
  }
  if (config::chassisControlEnabled) {
    ChassisParam param{config::wheelbase, config::trackWidth,
      config::maxLinearSpeed, config::maxAngularSpeed, config::maxWheelSpeed,
      config::maxWheelAcceleration, config::wheelKs, config::wheelKv,
      config::wheelMinDuty,
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
  lastControl_ = micros(); lastImu_ = lastControl_;
  if (encoderRequired) {
    if (!encoder_.readAllTicks(ticks_)) { fail(RobotFault::encoder); return false; }
    int64_t logical[wheelCount] = {};
    for (uint8_t i = 0; i < wheelCount; ++i) logical[i] = ticks_[config::wheelChannel[i]];
    wheelSpeed_.update(logical, lastControl_, speed_);
  }
  if (config::liftOutputsEnabled) {
    if (!liftConfigured() || !initLift()) {
      fail(RobotFault::lift); return false;
    }
    // 上电立即输出已标定的最低位脉宽，并持续保持该位置。
    liftController_.request(LiftPosition::lowered);
    liftTarget_ = 0;
    liftPulse_ = config::liftLoweredPulse;
    liftActive_ = true;
  }
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
  manualMode_ = false;
  safety_.invalidateCommand();
  if (benchMode_) benchDone_ = true;
}

void RobotController::stop() {
  stopChassis();
  stopLift();
}

bool RobotController::liftConfigured() const {
  return config::liftServoChannel < 2 && config::servoPwmFrequency == 50 &&
    config::liftLoweredPulse >= 500 && config::liftLoweredPulse <= 2500 &&
    config::liftRaisedPulse >= 500 && config::liftRaisedPulse <= 2500 &&
    config::liftLoweredPulse != config::liftRaisedPulse &&
    !(config::liftServoChannel == 1 && config::emergencyStopPin == board::servoS2);
}

void RobotController::stopLift() {
  liftController_.stop();
  servoDriver_.detach();
  liftReady_ = false; liftActive_ = false; liftTarget_ = 2; liftPulse_ = 0;
}

bool RobotController::initLift() {
  if (liftReady_) return true;
  if (!config::liftOutputsEnabled || !liftConfigured()) return false;
  const uint8_t pin = config::liftServoChannel == 0 ? board::servoS1 : board::servoS2;
  if (!servoDriver_.begin(pin, config::servoPwmFrequency) ||
      !liftController_.begin(servoDriver_, config::liftLoweredPulse,
                             config::liftRaisedPulse)) return false;
  liftReady_ = true;
  return true;
}

bool RobotController::commandLift(LiftPosition target) {
  if ((target != LiftPosition::lowered && target != LiftPosition::raised) ||
      !initialized_ || benchMode_ || fault_ != RobotFault::none ||
      !config::liftOutputsEnabled ||
      !liftConfigured()) return false;
  if (!initLift()) { fail(RobotFault::lift); return false; }
  liftController_.request(target);
  liftTarget_ = target == LiftPosition::raised ? 1 : 0;
  liftPulse_ = target == LiftPosition::raised ? config::liftRaisedPulse : config::liftLoweredPulse;
  liftActive_ = true;
  return true;
}

bool RobotController::commandLiftPulse(uint16_t pulse) {
  if (pulse < 500 || pulse > 2500 || !initialized_ || benchMode_ ||
      fault_ != RobotFault::none || !config::liftOutputsEnabled ||
      !liftConfigured()) return false;
  if (!initLift()) { fail(RobotFault::lift); return false; }
  if (!liftController_.requestPulse(pulse)) return false;
  liftTarget_ = 2; // 自定义脉宽，不对应已标定的 up/down 端点
  liftPulse_ = pulse;
  liftActive_ = true;
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

bool RobotController::armManual() {
  if (!initialized_ || !config::manualOpenLoopEnabled ||
      !config::motorOutputsEnabled || fault_ != RobotFault::none ||
      emergencyStop_ || physicalStop() || benchMode_) return false;
  stopChassis();
  manualMode_ = true;
  armed_ = true;
  return true;
}

bool RobotController::commandVelocity(const Twist2D &command) {
  if (!isfinite(command.vx) || !isfinite(command.vy) || !isfinite(command.wz)) {
    stop(); return false;
  }
  if (!armed_ || manualMode_ || emergencyStop_ || fault_ != RobotFault::none || physicalStop()) return false;
  command_ = command;
  commandReceived_ = true;
  safety_.noteVelocityCommand(millis());
  if (command.vx == 0.0f && command.vy == 0.0f && command.wz == 0.0f) halt();
  return true;
}

bool RobotController::commandManualVelocity(const Twist2D &command) {
  if (!isfinite(command.vx) || !isfinite(command.vy) || !isfinite(command.wz)) {
    stop(); return false;
  }
  if (!manualMode_ || !armed_ || emergencyStop_ || fault_ != RobotFault::none || physicalStop())
    return false;
  command_ = command;
  commandReceived_ = true;
  safety_.noteVelocityCommand(millis());
  if (command.vx == 0.0f && command.vy == 0.0f && command.wz == 0.0f) halt();
  return true;
}

bool RobotController::setManualDuty(float duty) {
  if (!isfinite(duty) || duty <= 0.0f || duty > config::motorDutyLimit ||
      !manualMode_ || !armed_ || emergencyStop_ ||
      fault_ != RobotFault::none || physicalStop()) return false;
  manualDuty_ = duty;
  return true;
}

void RobotController::setEmergencyStop(bool active) {
  if (!active && physicalStop()) return;
  emergencyStop_ = active;
  safety_.setEmergencyStop(active);
  // 急停只作用于底盘；升降保持原输出。解除后仍需重新使能，台架测试永久结束。
  stopChassis();
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
  const float dt = elapsed * 0.000001f;
  const bool encoderRequired = config::chassisControlEnabled || config::encoderReadEnabled;
  if (encoderRequired) {
    if (!encoder_.readAllTicks(ticks_)) { fail(RobotFault::encoder); return; }
    int64_t logical[wheelCount] = {};
    for (uint8_t i = 0; i < wheelCount; ++i) logical[i] = ticks_[config::wheelChannel[i]];
    speedReady_ = wheelSpeed_.update(logical, now, speed_);
    if (!speedReady_) { fail(RobotFault::feedback); return; }
    Twist2D body{};
    if (kinematics_.forward(speed_, body) && (!config::useImuYaw || imuReady()))
      odometry_.update(body, config::useImuYaw ? imuSample_.gyroZ * config::imuYawDirection : body.wz,
                       dt, millis());
  }
  if (benchMode_) {
    if (!benchDone_ && fault_ == RobotFault::none && !emergencyStop_) {
      WheelValues electrical{};
      for (float &value : electrical.value) value = config::motorBenchDuty;
      motorDriver_.setAllDuty(electrical);
      for (uint8_t i = 0; i < wheelCount; ++i)
        duty_.value[i] = electrical.value[config::wheelChannel[i]];
    } else halt();
    return;
  }
  if (config::useImuYaw && !imuReady() && armed_) fail(RobotFault::imu);
  if (motionAllowed()) {
    WheelValues electrical{};
    if (manualMode_) {
      WheelValues wheel{};
      if (!kinematics_.inverse(command_, wheel)) { fail(RobotFault::configuration); return; }
      for (uint8_t i = 0; i < wheelCount; ++i) {
        duty_.value[i] = wheel.value[i] > 0.0f ? manualDuty_ :
                         wheel.value[i] < 0.0f ? -manualDuty_ : 0.0f;
        electrical.value[config::wheelChannel[i]] = duty_.value[i];
      }
    } else {
      if (!chassis_.update(command_, speed_, dt, duty_)) { fail(RobotFault::feedback); return; }
      if (!feedbackPlausible(millis())) { fail(RobotFault::feedback); return; }
      for (uint8_t i = 0; i < wheelCount; ++i)
        electrical.value[config::wheelChannel[i]] = duty_.value[i];
    }
    motorDriver_.setAllDuty(electrical);
  }
  if (imuStarted_ && static_cast<uint32_t>(now - lastImu_) >= config::imuPeriod) {
    lastImu_ = now;
    ImuSample next{};
    if (imu_.read(next)) { imuSample_ = next; hasImuSample_ = true; }
  }
}

} // namespace robot
