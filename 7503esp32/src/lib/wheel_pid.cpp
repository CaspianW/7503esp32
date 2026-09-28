#include "wheel_pid.h"

namespace robot {

WheelPid::WheelPid()
    : gains_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, integral_(0.0f),
      previousError_(0.0f), hasPreviousError_(false) {}

} // namespace robot

// Discrete PID implementation and output saturation to be added here.
