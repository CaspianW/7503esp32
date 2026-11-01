/**
 * @file serial_protocol_test.cpp
 * @brief Validate serial commands, including bounded lift pulse input.
 * @author Caspian Wu
 * @date 2026-10-26
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../src/communication/serial_protocol.h"
#include <cassert>
#include <string>
int main() {
  using robot::CommandKind;
  assert(robot::parseSerialCommand("arm").kind == CommandKind::arm);
  assert(robot::parseSerialCommand("disarm").kind == CommandKind::disarm);
  assert(robot::parseSerialCommand("disarm now").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("manual arm").kind != CommandKind::invalid);
  assert(robot::parseSerialCommand("manual stop").kind != CommandKind::invalid);
  assert(robot::parseSerialCommand("status").kind == CommandKind::status);
  assert(robot::parseSerialCommand("stop").kind == CommandKind::stop);
  auto command = robot::parseSerialCommand(" vel 0.1 -0.2 3e-1 \t");
  assert(command.kind == CommandKind::velocity);
  assert(command.velocity.vy == -0.2f);
  command = robot::parseSerialCommand("manual vel 0.05 -0.03 0.2");
  assert(command.kind != CommandKind::invalid);
  assert(command.velocity.vx == 0.05f);
  assert(command.velocity.vy == -0.03f);
  command = robot::parseSerialCommand("manual duty 0.55");
  assert(command.kind == CommandKind::manualDuty);
  assert(command.duty == 0.55f);
  assert(robot::parseSerialCommand("estop 1").active);
  assert(robot::parseSerialCommand("lift arm").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("lift keep").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("lift stop").kind == CommandKind::liftStop);
  assert(robot::parseSerialCommand("lift up").lift == robot::LiftPosition::raised);
  assert(robot::parseSerialCommand("lift down").lift == robot::LiftPosition::lowered);
  for (const auto &value : {500, 1700, 2500}) {
    command = robot::parseSerialCommand(std::to_string(value).c_str());
    assert(command.kind == CommandKind::liftPulse);
    assert(command.pulse == value);
  }
  assert(robot::parseSerialCommand("499").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("2501").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("0").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("3000").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("lift raw 2500").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("750.5").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("+750").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("750us").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("750 800").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("lift up extra").kind == CommandKind::invalid);
  for (const char *line : {"", "vel nan 0 0", "vel inf 0 0", "vel 1e99 0 0",
       "vel 0 0", "vel 0 0 0 x", "vel 1x 0 0", "arm x", "estop 2", "vel 0\n0 0",
       "manual", "manual vel nan 0 0", "manual vel 0 0", "manual arm extra",
       "manual duty", "manual duty nan", "manual duty 0.5 extra"})
    assert(robot::parseSerialCommand(line).kind == CommandKind::invalid);
  assert(robot::parseSerialCommand(std::string(96, 'a').c_str()).kind == CommandKind::invalid);
}
