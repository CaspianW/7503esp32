/**
 * @file serial_protocol.cpp
 * @brief Validated velocity, arming and emergency-stop command parsing.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "serial_protocol.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace robot {
SerialCommand parseSerialCommand(const char *line) {
  SerialCommand out;
  if (!line) return out;
  char buffer[96];
  size_t size = 0;
  while (line[size]) {
    if (size >= sizeof(buffer) - 1 ||
        (!isprint(static_cast<unsigned char>(line[size])) && line[size] != '\t'))
      return out;
    buffer[size] = line[size]; ++size;
  }
  buffer[size] = '\0';
  char *context = nullptr;
  char *token[5] = {};
  size_t count = 0;
  for (char *part = strtok_r(buffer, " \t", &context); part;
       part = strtok_r(nullptr, " \t", &context)) {
    if (count == 5) return out;
    token[count++] = part;
  }
  if (count == 1) {
    if (!strcmp(token[0], "status")) out.kind = CommandKind::status;
    if (!strcmp(token[0], "arm")) out.kind = CommandKind::arm;
    if (!strcmp(token[0], "stop")) out.kind = CommandKind::stop;
  } else if (count == 2 && !strcmp(token[0], "lift")) {
    if (!strcmp(token[1], "arm")) out.kind = CommandKind::liftArm;
    if (!strcmp(token[1], "stop")) out.kind = CommandKind::liftStop;
    if (!strcmp(token[1], "keep")) out.kind = CommandKind::liftKeep;
    if (!strcmp(token[1], "up") || !strcmp(token[1], "down")) {
      out.kind = CommandKind::liftTarget;
      out.lift = !strcmp(token[1], "up") ? LiftPosition::raised : LiftPosition::lowered;
    }
  } else if (count == 2 && !strcmp(token[0], "estop") &&
             (!strcmp(token[1], "0") || !strcmp(token[1], "1"))) {
    out.kind = CommandKind::emergencyStop;
    out.active = token[1][0] == '1';
  } else if (count == 4 && !strcmp(token[0], "vel")) {
    float values[3];
    for (size_t i = 0; i < 3; ++i) {
      errno = 0;
      char *end = nullptr;
      values[i] = strtof(token[i + 1], &end);
      if (end == token[i + 1] || *end || errno == ERANGE ||
          !isfinite(values[i])) return out;
    }
    out.velocity = {values[0], values[1], values[2]};
    out.kind = CommandKind::velocity;
  }
  return out;
}
} // namespace robot
