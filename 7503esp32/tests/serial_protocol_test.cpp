/**
 * @file serial_protocol_test.cpp
 * @brief Malformed serial commands cannot refresh velocity authority.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../src/communication/serial_protocol.h"
#include <cassert>
#include <string>
int main() {
  using robot::CommandKind;
  assert(robot::parseSerialCommand("arm").kind == CommandKind::arm);
  assert(robot::parseSerialCommand("status").kind == CommandKind::status);
  assert(robot::parseSerialCommand("stop").kind == CommandKind::stop);
  auto command = robot::parseSerialCommand(" vel 0.1 -0.2 3e-1 \t");
  assert(command.kind == CommandKind::velocity);
  assert(command.velocity.vy == -0.2f);
  assert(robot::parseSerialCommand("estop 1").active);
  assert(robot::parseSerialCommand("lift arm").kind == CommandKind::liftArm);
  assert(robot::parseSerialCommand("lift keep").kind == CommandKind::liftKeep);
  assert(robot::parseSerialCommand("lift stop").kind == CommandKind::liftStop);
  assert(robot::parseSerialCommand("lift up").lift == robot::LiftPosition::raised);
  assert(robot::parseSerialCommand("lift down").lift == robot::LiftPosition::lowered);
  assert(robot::parseSerialCommand("lift raw 2500").kind == CommandKind::invalid);
  assert(robot::parseSerialCommand("lift up extra").kind == CommandKind::invalid);
  for (const char *line : {"", "vel nan 0 0", "vel inf 0 0", "vel 1e99 0 0",
       "vel 0 0", "vel 0 0 0 x", "vel 1x 0 0", "arm x", "estop 2", "vel 0\n0 0"})
    assert(robot::parseSerialCommand(line).kind == CommandKind::invalid);
  assert(robot::parseSerialCommand(std::string(96, 'a').c_str()).kind == CommandKind::invalid);
}
