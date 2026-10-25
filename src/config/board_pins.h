/**
 * @file board_pins.h
 * @brief Yahboom control board pin assignments.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

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
constexpr uint8_t motorCount = 4;
constexpr MotorPins motorPins[motorCount] = {
    {4, 5, 6, 7},     // M1：实际车轮位置待确认
    {15, 16, 47, 48}, // M2：实际车轮位置待确认
    {9, 10, 11, 12},  // M3：实际车轮位置待确认
    {13, 14, 1, 2},   // M4：实际车轮位置待确认
};

constexpr uint8_t imuSda = 40;
constexpr uint8_t imuScl = 39;
constexpr int8_t imuInterrupt = 41;

constexpr uint8_t servoS1 = 8;
constexpr uint8_t servoS2 = 21;

constexpr uint8_t batterySense = 3;
constexpr uint8_t buzzer = 46;
constexpr uint8_t statusLed = 45;
// MCU 蓝灯的点亮电平尚未实测；若上板后亮灭相反，改为 true。
constexpr bool statusLedActiveLow = false;

// 控制板的 Type-C 串口使用 UART0：TX=GPIO43，RX=GPIO44。
// 机器人运行时，此串口留给 micro-ROS agent 连接使用。

} // board 命名空间结束
} // robot 命名空间结束
