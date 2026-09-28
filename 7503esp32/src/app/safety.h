#pragma once

#include <stdint.h>

namespace robot {

class Safety {
public:
  void begin();
  void noteVelocityCommand(uint32_t timestampMs);
  void setEmergencyStop(bool active);
  void update(uint32_t nowMs);
  bool motionAllowed() const;
  bool commandTimedOut() const;

private:
  uint32_t lastCommandMs_;
  bool hasCommand_;
  bool emergencyStop_;
  bool commandTimedOut_;
};

} // namespace robot
