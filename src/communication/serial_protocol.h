/**
 * @file serial_protocol.h
 * @brief Strict ASCII command parser for chassis and lift pulse commands.
 * @author Caspian Wu
 * @date 2026-10-26
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include "../common/robot_types.h"
#include <stddef.h>

namespace robot {
enum class CommandKind { invalid, status, arm, stop, emergencyStop, velocity,
                         manualArm, manualStop, manualVelocity, manualDuty,
                         liftStop, liftTarget, liftPulse };
struct SerialCommand {
  CommandKind kind = CommandKind::invalid;
  Twist2D velocity{};
  float duty = 0.0f;
  bool active = false;
  LiftPosition lift = LiftPosition::lowered;
  uint16_t pulse = 0; // 单位 µs
};
// 一行最多 95 个可见字符，只接受完整命令、有限数和指定数量的参数。
SerialCommand parseSerialCommand(const char *line);
} // namespace robot
