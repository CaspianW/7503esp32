/**
 * @file robot_types.h
 * @brief Shared robot data types.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include <stdint.h>

namespace robot {

enum WheelId : uint8_t {
  frontLeft = 0,  // M1
  rearLeft = 1,   // M2
  frontRight = 2, // M3
  rearRight = 3,  // M4
  wheelCount = 4
};

struct Twist2D {
  float vxMps;       // 向前为正
  float vyMps;       // 向左为正
  float wzRadPerSec; // 逆时针为正
};

enum class LiftPosition : uint8_t {
  lowered,
  raised
};

// 各车轮的正值均表示其配置的前进方向；每个电机的实际方向仍需校准。
struct WheelValues {
  float value[wheelCount];
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
