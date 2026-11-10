#pragma once
#include <stdint.h>
namespace robot {
class RobotController;
// Human-readable, read-only UART monitor. No binary command dispatch.
class MotorDebug {
public:
  void begin();
  void update(const RobotController &controller);
private:
  uint32_t lastPrintMs_ = 0;
};
}
