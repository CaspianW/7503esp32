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
#include "serial_telemetry.h"
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
  SerialTelemetry frame;
  frame.uptimeMs = millis();
  frame.initialized = c.initialized();
  frame.armed = c.armed();
  frame.motionAllowed = c.motionAllowed();
  frame.emergencyStopped = c.emergencyStopped();
  frame.fault = static_cast<uint8_t>(c.fault());
  frame.encoderReady = c.encoderReady();
  frame.imuReady = c.imuReady();
  frame.speedReady = c.speedReady();
  frame.benchMode = c.benchMode();
  frame.motorOutputsEnabled = config::motorOutputsEnabled;
  frame.chassisControlEnabled = config::chassisControlEnabled;
  frame.manualMode = c.manualMode();
  frame.wheelTarget = c.wheelTarget();
  frame.wheelSpeed = c.wheelSpeed();
  frame.duty = c.duty();
  frame.odometry = c.odometry();
  frame.imu = c.imuSample();
  frame.liftOutputsEnabled = config::liftOutputsEnabled;
  frame.liftConfigured = c.liftConfigured();
  frame.liftReady = c.liftReady();
  frame.liftActive = c.liftActive();
  frame.liftTarget = c.liftTarget();
  frame.liftPulseUs = c.liftPulse();
  char buffer[serialTelemetryCapacity];
  if (formatSerialTelemetry(buffer, sizeof(buffer), frame, config::telemetrySampleMaxAge))
    reply(buffer);
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
