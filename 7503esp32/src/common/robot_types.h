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
  float vxMps;       // 向前为正
  float vyMps;       // 向左为正
  float wzRadPerSec; // 逆时针为正
};

// 各车轮的正值均表示其配置的前进方向；每个电机的实际方向仍需校准。
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

} // robot 命名空间结束
