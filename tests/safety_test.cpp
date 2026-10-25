/**
 * @file safety_test.cpp
 * @brief Host tests for velocity-command timeout and emergency stop.
 * @author Caspian Wu
 * @date 2026-10-19
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../src/app/safety.h"

#include <cassert>

int main() {
  robot::Safety safety;
  safety.begin();

  // 上电未收到指令时禁止运动；超时边界恰好为 500 ms。
  assert(!safety.motionAllowed());
  assert(safety.commandTimedOut());
  safety.noteVelocityCommand(100);
  safety.update(599);
  assert(safety.motionAllowed());
  safety.update(600);
  assert(!safety.motionAllowed());
  assert(safety.commandTimedOut());

  // 新指令恢复许可，但急停触发后必须重新收到指令。
  safety.noteVelocityCommand(700);
  safety.update(700);
  assert(safety.motionAllowed());
  safety.setEmergencyStop(true);
  assert(!safety.motionAllowed());
  safety.noteVelocityCommand(710);
  safety.setEmergencyStop(false);
  safety.update(711);
  assert(!safety.motionAllowed());
  safety.noteVelocityCommand(720);
  safety.update(720);
  assert(safety.motionAllowed());

  // millis() 回绕时仍按无符号时间差判定超时。
  safety.begin();
  safety.noteVelocityCommand(0xFFFFFF00U);
  safety.update(0x000000F3U);
  assert(safety.motionAllowed());
  safety.update(0x000000F4U);
  assert(!safety.motionAllowed());
}
