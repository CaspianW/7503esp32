/**
 * @file safety.h
 * @brief Robot safety management interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include <stdint.h>

namespace robot {

class Safety {
public:
  void begin();
  void noteVelocityCommand(uint32_t timestamp); // 单位 ms
  void setEmergencyStop(bool active);
  void update(uint32_t now); // 单位 ms
  bool motionAllowed() const;
  bool commandTimedOut() const;

private:
  uint32_t lastCommand_; // 单位 ms
  bool hasCommand_;
  bool emergencyStop_;
  bool commandTimedOut_;
};

} // robot 命名空间结束
