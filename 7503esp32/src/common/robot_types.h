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
  // 逻辑轮序固定为 FL、RL、FR、RR；实体电气通道映射见 robot_config::wheelChannel。
  frontLeft = 0,
  rearLeft = 1,
  frontRight = 2,
  rearRight = 3,
  wheelCount = 4
};

struct Twist2D {
  float vx; // 向前为正，单位 m/s
  float vy; // 向左为正，单位 m/s
  float wz; // 逆时针为正，单位 rad/s
};

enum class LiftPosition : uint8_t {
  lowered,
  raised
};

// 各车轮的正值均表示其配置的前进方向；每个电机的实际方向仍需校准。
struct WheelValues {
  float value[wheelCount]; // 轮速时单位 m/s，电机控制输出时为无量纲占空比
};

struct ImuSample {
  float accelX;       // X 轴加速度，单位 m/s²
  float accelY;       // Y 轴加速度，单位 m/s²
  float accelZ;       // Z 轴加速度，单位 m/s²
  float gyroX;        // X 轴角速度，单位 rad/s
  float gyroY;        // Y 轴角速度，单位 rad/s
  float gyroZ;        // Z 轴角速度，单位 rad/s
  float temperature;  // 温度，单位 °C
  uint32_t timestamp; // 时间戳，单位 ms
};

struct OdometryState {
  float x;            // 位置 X，单位 m
  float y;            // 位置 Y，单位 m
  float yaw;          // 航向角，单位 rad
  float vx;           // 向前速度，单位 m/s
  float vy;           // 向左速度，单位 m/s
  float wz;           // 逆时针角速度，单位 rad/s
  uint32_t timestamp; // 时间戳，单位 ms
};

} // robot 命名空间结束
