/**
 * @file safety_test.cpp
 * @brief Host tests for velocity-command timeout and emergency stop.
 * @author Caspian Wu
 * @date 2026-10-19
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../../src/app/safety.h"
#include "../../src/config/robot_config.h"

#include <cassert>

int main() {
  robot::Safety safety;
  safety.begin();

  // 上电未收到指令时禁止运动；超时边界恰好为 1000 ms。
  assert(robot::config::commandTimeout == 1000);
  assert(!safety.motionAllowed());
  assert(safety.commandTimedOut());
  safety.noteVelocityCommand(100);
  safety.update(1099);
  assert(safety.motionAllowed());
  safety.update(1100);
  assert(!safety.motionAllowed());
  assert(safety.commandTimedOut());

  // 新指令恢复许可，但急停触发后必须重新收到指令。
  safety.noteVelocityCommand(1200);
  safety.update(1200);
  assert(safety.motionAllowed());
  safety.setEmergencyStop(true);
  assert(!safety.motionAllowed());
  safety.noteVelocityCommand(1210);
  safety.setEmergencyStop(false);
  safety.update(1211);
  assert(!safety.motionAllowed());
  safety.noteVelocityCommand(1220);
  safety.update(1220);
  assert(safety.motionAllowed());

  // millis() 回绕时仍按无符号时间差判定超时。
  safety.begin();
  safety.noteVelocityCommand(0xFFFFFF00U);
  safety.update(0x000002E7U);
  assert(safety.motionAllowed());
  safety.update(0x000002E8U);
  assert(!safety.motionAllowed());
}
