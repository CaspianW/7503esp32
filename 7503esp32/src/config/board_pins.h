#pragma once

#include <stdint.h>

namespace robot {
namespace board {

struct MotorPins {
  uint8_t pwmA;
  uint8_t pwmB;
  uint8_t encoderA;
  uint8_t encoderB;
};

// Yahboom Micro-ROS 控制板。数组顺序为 M1、M2、M3、M4。
// 电机线缆的白色接头连接到此控制板。
constexpr uint8_t kMotorCount = 4;
constexpr MotorPins kMotorPins[kMotorCount] = {
    {4, 5, 6, 7},     // M1：左前
    {15, 16, 47, 48}, // M2：左后
    {9, 10, 11, 12},  // M3：右前
    {13, 14, 1, 2},   // M4：右后
};

constexpr uint8_t kImuSda = 40;
constexpr uint8_t kImuScl = 39;
constexpr int8_t kImuInterrupt = 41;

constexpr uint8_t kServoS1 = 8;
constexpr uint8_t kServoS2 = 21;

constexpr uint8_t kBatterySense = 3;
constexpr uint8_t kBuzzer = 46;
constexpr uint8_t kStatusLed = 45;

// 控制板的 Type-C 串口使用 UART0：RX=GPIO43，TX=GPIO44。
// 机器人运行时，此串口留给 micro-ROS agent 连接使用。

} // board 命名空间结束
} // robot 命名空间结束
