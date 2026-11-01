/**
 * @file serial_console.cpp
 * @brief Serial command dispatch and telemetry without waiting for a host connection.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "serial_console.h"

#include "serial_protocol.h"
#include "../app/robot_controller.h"
#include "../config/robot_config.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

namespace robot {
void SerialConsole::begin() {
  // 通过 USB-UART 桥使用 UART0；禁止同时打开 micro-ROS agent。
  Serial.setTxBufferSize(2048);
  Serial.begin(config::serialBaud);
  lastTelemetry_ = micros();
}
void SerialConsole::reply(const char *text) {
  const size_t length = strlen(text);
  if (Serial.availableForWrite() >= static_cast<int>(length))
    Serial.write(reinterpret_cast<const uint8_t *>(text), length);
}
void SerialConsole::update(RobotController &controller) {
  // 不完整行超过 100ms 必须丢弃到下一个换行，不能把尾部当成新命令。
  if (size_ && static_cast<uint32_t>(millis() - lastByte_) > 100) {
    size_ = 0; discard_ = true; controller.stop();
  }
  uint8_t budget = 64;
  while (budget-- && Serial.available()) {
    const int input = Serial.read();
    if (input < 0) break;
    lastByte_ = millis();
    if (input == '\r') continue;
    if (input != '\n') {
      if (!discard_) {
        if (size_ + 1 >= sizeof(line_) || input == 0) {
          discard_ = true; size_ = 0; controller.stop();
        } else line_[size_++] = static_cast<char>(input);
      }
      continue;
    }
    if (discard_) {
      discard_ = false; size_ = 0; reply("ERR frame\n"); continue;
    }
    line_[size_] = '\0'; size_ = 0;
    const SerialCommand command = parseSerialCommand(line_);
    bool accepted = true;
    switch (command.kind) {
    case CommandKind::status: telemetry(controller); break;
    case CommandKind::stop: controller.stop(); break;
    case CommandKind::arm: accepted = controller.arm(); break;
    case CommandKind::disarm: controller.disarm(); break;
    case CommandKind::manualArm: accepted = controller.armManual(); break;
    case CommandKind::manualStop: controller.stop(); break;
    case CommandKind::emergencyStop:
      controller.setEmergencyStop(command.active);
      accepted = controller.emergencyStopped() == command.active; break;
    case CommandKind::velocity:
      accepted = controller.commandVelocity(command.velocity); break;
    case CommandKind::manualVelocity:
      accepted = controller.commandManualVelocity(command.velocity); break;
    case CommandKind::manualDuty:
      accepted = controller.setManualDuty(command.duty); break;
    case CommandKind::liftStop: controller.stopLift(); break;
    case CommandKind::liftTarget: accepted = controller.commandLift(command.lift); break;
    // 无效完整命令只拒绝，不改变执行器状态；底盘旧指令仍受超时保护。
    default: accepted = false; break;
    }
    reply(accepted ? "OK\n" : "ERR command_or_state\n");
  }
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - lastTelemetry_) >= config::telemetryPeriod) {
    lastTelemetry_ = now; telemetry(controller);
  }
}
void SerialConsole::telemetry(const RobotController &c) {
  char buffer[900];
  const auto &target = c.wheelTarget();
  const auto &speed = c.wheelSpeed();
  const auto &duty = c.duty();
  const auto &odom = c.odometry();
  const auto &imu = c.imuSample();
  const int length = snprintf(buffer, sizeof(buffer),
    "S %lu %u %u %u %u %u %u %u %u %u %u %u %u\n"
    "E %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.4f %.4f %.4f %.4f\n"
    "O %lu %.5f %.5f %.5f %.5f %.5f %.5f\n"
    "I %lu %.5f %.5f %.5f %.5f %.5f %.5f %.3f\n"
    "L %u %u %u %u %u %u\n",
    static_cast<unsigned long>(millis()), c.initialized(), c.armed(), c.motionAllowed(),
    c.emergencyStopped(), static_cast<unsigned>(c.fault()), c.encoderReady(),
    c.imuReady(), c.speedReady(), c.benchMode(), config::motorOutputsEnabled,
    config::chassisControlEnabled, c.manualMode(),
    target.value[0], target.value[1], target.value[2], target.value[3],
    speed.value[0], speed.value[1], speed.value[2], speed.value[3],
    duty.value[0], duty.value[1], duty.value[2], duty.value[3],
    static_cast<unsigned long>(odom.timestamp), odom.x, odom.y, odom.yaw,
    odom.vx, odom.vy, odom.wz, static_cast<unsigned long>(imu.timestamp),
    imu.accelX, imu.accelY, imu.accelZ, imu.gyroX, imu.gyroY, imu.gyroZ, imu.temperature,
    config::liftOutputsEnabled, c.liftConfigured(), c.liftReady(), c.liftActive(),
    c.liftTarget(), c.liftPulse());
  if (length > 0 && static_cast<size_t>(length) < sizeof(buffer)) reply(buffer);
  // 台架模式额外回读PWM外设，便于对比E行中的软件占空比目标。
  if (c.benchMode()) {
    for (uint8_t channel = 0; channel < wheelCount; ++channel) {
      const PwmState pwm = c.pwm(channel);
      const int size = snprintf(buffer, sizeof(buffer), "P %u %d %lu %lu %d %d %d %d\n",
        static_cast<unsigned>(channel + 1), static_cast<int>(pwm.pin),
        static_cast<unsigned long>(pwm.duty), static_cast<unsigned long>(pwm.freq),
        static_cast<int>(pwm.route), static_cast<int>(pwm.out),
        static_cast<int>(pwm.gpio), static_cast<int>(pwm.pause));
      if (size > 0 && static_cast<size_t>(size) < sizeof(buffer)) reply(buffer);
    }
  }
}
} // namespace robot
