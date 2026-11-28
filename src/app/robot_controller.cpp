/**
 * @file robot_controller.cpp
 * @brief Scheduled chassis control, manual driving and lift pulse output.
 * @author Caspian Wu
 * @date 2026-11-01
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
               config::wheelPidOutputLimit, config::wheelPidIntegralLimit, true),
      config::wheelKickstartDuty, config::wheelKickstartMs, config::wheelKickstartSpeed,
      config::wheelBrakeMinDuty, config::wheelBrakeGain, config::wheelBrakeMaxDuty,
      config::wheelBrakeStopSpeed, config::wheelBrakeMaxMs};
    if (!config::motorOutputsEnabled || !config::wheelMappingConfirmed ||
        !geometryReady || !chassis_.configure(param) ||
        !isfinite(config::minWheelSpeed) || config::minWheelSpeed < 0.0f ||
        config::minWheelSpeed > config::maxWheelSpeed ||
        config::wheelPidOutputLimit > config::motorDutyLimit) {
      fail(RobotFault::configuration); return false;
    }
  }
  imuStarted_ = !benchMode_ && imu_.begin();
  if (config::useImuYaw &&
      config::imuYawDirection != 1 && config::imuYawDirection != -1) {
    fail(RobotFault::configuration); return false;
  }
  lastControl_ = micros(); lastImu_ = lastControl_;
  if (encoderRequired) {
    if (!encoder_.readAllTicks(ticks_) && (!encoder_.recover() || !encoder_.readAllTicks(ticks_))) {
      fail(RobotFault::encoder); return false;
    }
    int64_t logical[wheelCount] = {};
    for (uint8_t i = 0; i < wheelCount; ++i) logical[i] = ticks_[config::wheelChannel[i]];
    wheelSpeed_.update(logical, lastControl_, speed_);
  }
  if (config::liftOutputsEnabled) {
    if (!liftConfigured() || !initLift()) {
      fail(RobotFault::lift);
    } else {
      // 上电直接下降，经过动作时间后再上报下降完成位。
      liftController_.request(LiftPosition::lowered, millis());
      liftTarget_ = 0;
      liftPulse_ = config::liftLoweredPulse;
      liftActive_ = true;
    }
  }
  initialized_ = true;
  // 上电默认使能待命（PWM 仍为 0，收到有效 0x01 后才运动），避免树莓派未先发 arm 时失控。
  if (config::chassisControlEnabled && config::motorOutputsEnabled &&
      !benchMode_ && !emergencyStop_ && !physicalStop()) {
    armed_ = true;
  }
  return true;
}

bool RobotController::physicalStop() const {
  return config::emergencyStopPin >= 0 &&
    (digitalRead(config::emergencyStopPin) == (config::emergencyStopActiveHigh ? HIGH : LOW));
}

void RobotController::halt() {
  motorDriver_.stopAll(false);
  duty_ = {}; command_ = {};
  chassis_.reset();
}

void RobotController::stopChassis() {
  clearLowSpeedFault();
  halt(); armed_ = false;
  manualMode_ = false;
  safety_.invalidateCommand();
  if (benchMode_) benchDone_ = true;
}

void RobotController::stop() {
  stopChassis();
  stopLift();
}

void RobotController::disarm() {
  stopChassis();
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

bool RobotController::recoverEncoder() {
  if (encoder_.healthy()) return true;
  if (!encoder_.recover()) return false;
  if (!encoder_.readAllTicks(ticks_)) return false;
  int64_t logical[wheelCount] = {};
  for (uint8_t i = 0; i < wheelCount; ++i) logical[i] = ticks_[config::wheelChannel[i]];
  const uint32_t now = micros();
  wheelSpeed_.reset();
  wheelSpeed_.update(logical, now, speed_);
  lastControl_ = now;
  return true;
}

void RobotController::pollImu(uint32_t now) {
  if (!imuStarted_) return;
  const uint32_t period = imuFailCount_ >= 5 ? 200000U : config::imuPeriod;
  if (static_cast<uint32_t>(now - lastImu_) < period) return;
  lastImu_ = now;
  ImuSample next{};
  if (imu_.read(next)) {
    imuSample_ = next;
    hasImuSample_ = true;
    imuFailCount_ = 0;
  } else if (imuFailCount_ < 10) {
    ++imuFailCount_;
  }
}

void RobotController::clearLowSpeedFault() {
  if (lowSpeedFault_) {
    lowSpeedFault_ = false;
    fault_ = RobotFault::none;
  }
}

bool RobotController::commandTooSlow(const Twist2D &command) const {
  if (config::minWheelSpeed <= 0.0f) return false;
  WheelValues wheel{};
  if (!chassis_.solveWheelTargets(command, wheel)) return true;
  const float threshold = config::minWheelSpeed - 1e-4f;
  bool anyMoving = false;
  for (uint8_t i = 0; i < wheelCount; ++i) {
    const float speed = fabsf(wheel.value[i]);
    if (speed == 0.0f) continue;
    anyMoving = true;
    if (speed < threshold) return true;
  }
  return !anyMoving;
}

bool RobotController::commandLift(LiftPosition target) {
  if ((target != LiftPosition::lowered && target != LiftPosition::raised) ||
      !initialized_ || benchMode_ ||
      !config::liftOutputsEnabled ||
      !liftConfigured()) return false;
  if (!initLift()) { fail(RobotFault::lift); return false; }
  liftFault_ = false;
  liftController_.request(target, millis());
  liftTarget_ = target == LiftPosition::raised ? 1 : 0;
  liftPulse_ = target == LiftPosition::raised ? config::liftRaisedPulse : config::liftLoweredPulse;
  liftActive_ = true;
  return true;
}

void RobotController::fail(RobotFault fault) {
  if (fault == RobotFault::lift) {
    liftFault_ = true;
    stopLift();
    return;
  }
  if (fault_ == RobotFault::none || lowSpeedFault_) {
    fault_ = fault;
    lowSpeedFault_ = false;
  }
  stopChassis();
}

bool RobotController::arm() {
  if (!initialized_ || physicalStop() || benchMode_ ||
      !config::chassisControlEnabled || !config::motorOutputsEnabled) return false;
  // 无物理急停触发时，显式 arm (0x03=1) 可直接解除软件急停并恢复控制，避免卡死。
  if (emergencyStop_) {
    emergencyStop_ = false;
    safety_.setEmergencyStop(false);
  }
  const bool encoderOk = recoverEncoder();
  // 重复 arm 也丢弃旧速度，必须随后发送新命令。
  stopChassis();
  fault_ = encoderOk ? RobotFault::none : RobotFault::encoder;
  lowSpeedFault_ = false;
  armed_ = true;
  return true;
}

bool RobotController::armManual() {
  if (!initialized_ || !config::manualOpenLoopEnabled ||
      !config::motorOutputsEnabled ||
      physicalStop() || benchMode_) return false;
  if (emergencyStop_) {
    emergencyStop_ = false;
    safety_.setEmergencyStop(false);
  }
  if ((config::chassisControlEnabled || config::encoderReadEnabled) && !encoder_.healthy())
    recoverEncoder();
  fault_ = RobotFault::none;
  lowSpeedFault_ = false;
  stopChassis();
  manualMode_ = true;
  armed_ = true;
  return true;
}

bool RobotController::commandVelocity(const Twist2D &command) {
  if (!isfinite(command.vx) || !isfinite(command.vy) || !isfinite(command.wz)) {
    halt();
    safety_.invalidateCommand();
    return false;
  }
  if (command.vx == 0.0f && command.vy == 0.0f && command.wz == 0.0f) {
    if (!armed_ || manualMode_ || emergencyStop_ || physicalStop()) {
      halt();
      return true;
    }
    bool spinning = false;
    for (uint8_t i = 0; i < wheelCount; ++i) {
      if (fabsf(speed_.value[i]) > config::wheelBrakeStopSpeed) {
        spinning = true;
        break;
      }
    }
    if (!spinning || config::wheelBrakeMaxDuty <= 0.0f || config::wheelBrakeMaxMs == 0U) {
      halt();
      safety_.noteVelocityCommand(millis());
      return true;
    }
    // 车轮仍在滚动时交由闭环控制周期输出短时主动反接制动，停稳后自动切断 PWM。
    clearLowSpeedFault();
    command_ = command;
    safety_.noteVelocityCommand(millis());
    return true;
  }
  if (!armed_ || manualMode_ || emergencyStop_ || physicalStop()) return false;
  if (commandTooSlow(command)) {
    // 轮速低于可控下限时停机并持续报故障 5，但保持使能，收到正常速度后自动恢复。
    halt();
    safety_.noteVelocityCommand(millis());
    fault_ = RobotFault::feedback;
    lowSpeedFault_ = true;
    return true;
  }
  clearLowSpeedFault();
  command_ = command;
  safety_.noteVelocityCommand(millis());
  return true;
}

bool RobotController::commandManualVelocity(const Twist2D &command) {
  if (!isfinite(command.vx) || !isfinite(command.vy) || !isfinite(command.wz)) {
    halt();
    safety_.invalidateCommand();
    return false;
  }
  if (!manualMode_ || !armed_ || emergencyStop_ || fault_ != RobotFault::none || physicalStop())
    return false;
  command_ = command;
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
  const uint32_t nowMs = millis();
  const uint32_t elapsed = now - lastControl_;
  // 在任何 I2C 或串口操作前先检查超时并撤销输出。
  safety_.update(nowMs);
  if (!benchMode_ && !safety_.motionAllowed()) {
    // 看门狗超时仅安全停轮（清零目标与 PWM），保持 armed_ 以便下一帧 0x01 立即恢复。
    halt();
  }
  if (elapsed < config::controlPeriod) return;
  liftController_.update(nowMs);
  lastControl_ = now;
  const bool encoderRequired = config::chassisControlEnabled || config::encoderReadEnabled;
  if (elapsed > config::maxControlGap) {
    // 瞬时时序抖动不锁死整车：跳过本周期控制并安全停轮，刷新传感器基线后下周期自动恢复。
    motorDriver_.stopAll(false);
    duty_ = {};
    chassis_.reset();
    if (encoderRequired) {
      if (encoder_.healthy() && encoder_.readAllTicks(ticks_)) {
        int64_t logical[wheelCount] = {};
        for (uint8_t i = 0; i < wheelCount; ++i) logical[i] = ticks_[config::wheelChannel[i]];
        if (wheelSpeed_.update(logical, now, speed_)) speedReady_ = true;
        if (fault_ == RobotFault::encoder) fault_ = RobotFault::none;
      } else if (recoverEncoder()) {
        if (fault_ == RobotFault::encoder) fault_ = RobotFault::none;
      } else {
        fault_ = RobotFault::encoder;
      }
    }
    pollImu(now);
    return;
  }
  const float dt = elapsed * 0.000001f;
  bool speedValidThisCycle = true;
  if (encoderRequired) {
    if (!encoder_.healthy() || !encoder_.readAllTicks(ticks_)) {
      // PCNT 异常仅暂停本拍轮输出并立即尝试自恢复，不 disarm、不永久锁死、不影响升降。
      motorDriver_.stopAll(false);
      duty_ = {};
      chassis_.reset();
      if (recoverEncoder()) {
        if (fault_ == RobotFault::encoder) fault_ = RobotFault::none;
      } else {
        fault_ = RobotFault::encoder;
      }
      pollImu(now);
      return;
    }
    if (fault_ == RobotFault::encoder) fault_ = RobotFault::none;
    int64_t logical[wheelCount] = {};
    for (uint8_t i = 0; i < wheelCount; ++i) logical[i] = ticks_[config::wheelChannel[i]];
    if (wheelSpeed_.update(logical, now, speed_)) {
      speedReady_ = true;
      Twist2D body{};
      if (kinematics_.forward(speed_, body)) {
        const float yawRate = (config::useImuYaw && imuReady())
          ? imuSample_.gyroZ * config::imuYawDirection
          : body.wz;
        odometry_.update(body, yawRate, dt, nowMs);
      }
    } else {
      // 单次测速异常（含自恢复后的首帧基线）仅跳过本拍输出，不锁死也不影响升降。
      speedValidThisCycle = false;
      motorDriver_.stopAll(false);
      duty_ = {};
      chassis_.reset();
    }
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
  if (speedValidThisCycle && motionAllowed()) {
    WheelValues electrical{};
    if (manualMode_) {
      WheelValues wheel{};
      if (!kinematics_.inverse(command_, wheel)) {
        motorDriver_.stopAll(false);
        duty_ = {};
        pollImu(now);
        return;
      }
      for (uint8_t i = 0; i < wheelCount; ++i) {
        duty_.value[i] = wheel.value[i] > 0.0f ? manualDuty_ :
                         wheel.value[i] < 0.0f ? -manualDuty_ : 0.0f;
        electrical.value[config::wheelChannel[i]] = duty_.value[i];
      }
      motorDriver_.setAllDuty(electrical);
    } else {
      if (!chassis_.update(command_, speed_, dt, duty_)) {
        // 单次反馈超限或计算异常仅跳过本周期输出，不永久锁死。
        motorDriver_.stopAll(false);
        duty_ = {};
        chassis_.reset();
      } else {
        for (uint8_t i = 0; i < wheelCount; ++i)
          electrical.value[config::wheelChannel[i]] = duty_.value[i];
        motorDriver_.setAllDuty(electrical);
      }
    }
  }
  pollImu(now);
}

} // namespace robot
