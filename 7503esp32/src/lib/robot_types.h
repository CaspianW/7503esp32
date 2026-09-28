#pragma once

#include <stdint.h>

namespace robot {

enum WheelId : uint8_t {
  kFrontLeft = 0,  // M1
  kRearLeft = 1,   // M2
  kFrontRight = 2, // M3
  kRearRight = 3,  // M4
  kWheelCount = 4
};

struct Twist2D {
  float vxMps;       // forward is positive
  float vyMps;       // left is positive
  float wzRadPerSec; // counter-clockwise is positive
};

// All wheel values use positive motion in the robot's configured forward
// wheel direction. The physical sign for each motor will be calibrated.
struct WheelValues {
  float value[kWheelCount];
};

struct ImuSample {
  float accelXMps2;
  float accelYMps2;
  float accelZMps2;
  float gyroXRadPerSec;
  float gyroYRadPerSec;
  float gyroZRadPerSec;
  float temperatureC;
  uint32_t timestampMs;
};

struct OdometryState {
  float xM;
  float yM;
  float yawRad;
  float vxMps;
  float vyMps;
  float wzRadPerSec;
  uint32_t timestampMs;
};

} // namespace robot
