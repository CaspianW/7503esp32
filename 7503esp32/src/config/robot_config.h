/**
 * @file robot_config.h
 * @brief Robot and chassis configuration parameters.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include <stdint.h>

#include "../common/robot_types.h"

namespace robot {
namespace config {

constexpr float wheelDiameter = 0.065f; // 单位 m
constexpr float wheelRadius = wheelDiameter * 0.5f; // 单位 m

// 启用麦克纳姆轮运动前，需在组装好的底盘上实测以下尺寸。
constexpr float wheelbase = 0.0f;  // 前后轮中心距，单位 m
constexpr float trackWidth = 0.0f; // 左右轮中心距，单位 m

// Yahboom 标称编码器为 13 PPR、减速比为 1:20；其示例对 AB 两路的边沿计数，
// 因此输出轴每转约对应 1040 个计数。
constexpr float encoderCounts = 1040.0f; // 输出轴每转计数，单位 count/rev

constexpr uint32_t motorPwmFrequency = 25000; // 单位 Hz
constexpr uint32_t servoPwmFrequency = 50;    // 单位 Hz
constexpr uint16_t imuOutputRate = 100;       // 单位 Hz
constexpr uint16_t imuAccelFullScale = 16;    // 单位 g
constexpr uint16_t imuGyroFullScale = 2000;   // 单位 °/s

// 实车核对电机接线、车轮方向与急停后才能改为 true。
constexpr bool motorOutputsEnabled = false;
// 台架测试需同时打开此开关和 motorOutputsEnabled；默认上电绝不转动。
constexpr bool motorBenchEnabled = false;
constexpr uint8_t motorBenchChannel = 0; // 0～3 对应控制板 M1～M4，非车轮位置
constexpr float motorBenchDuty = 0.15f;
constexpr uint32_t motorBenchWait = 2000; // 单位 ms
constexpr uint32_t motorBenchRun = 500;   // 单位 ms
// 电机正向与编码器计数正向需在实车上逐轮校准。
constexpr int8_t motorDirection[wheelCount] = {1, 1, 1, 1};
constexpr int8_t encoderDirection[wheelCount] = {1, 1, 1, 1};

// MG996 舵机无外部位置反馈；以下是控制脉宽，不是编码器读数。
// 实车装配并确认机械限位后再标定，零值表示禁止启用升降控制。
constexpr uint16_t liftLoweredPulse = 0; // 单位 µs
constexpr uint16_t liftRaisedPulse = 0;  // 单位 µs

constexpr uint32_t controlPeriod = 10000;  // 单位 µs，100 Hz
constexpr uint32_t imuPeriod = 10000;      // 单位 µs，100 Hz
constexpr uint32_t odometryPeriod = 20000; // 单位 µs，50 Hz
constexpr uint32_t rosSpinPeriod = 1000;   // 单位 µs，最高 1 kHz

// 实测机器人后再整定。增益暂设为零，待电机和编码器特性确认后再启用车轮控制器。
constexpr float wheelPidKp = 0.0f;
constexpr float wheelPidKi = 0.0f;
constexpr float wheelPidKd = 0.0f;
constexpr float wheelPidOutputLimit = 1.0f;
constexpr float wheelPidIntegralLimit = 1.0f;

constexpr float maxLinearSpeed = 0.0f;  // 单位 m/s，车轮校准后设置
constexpr float maxAngularSpeed = 0.0f; // 单位 rad/s，底盘校准后设置
constexpr uint32_t commandTimeout = 500; // 单位 ms

} // config 命名空间结束
} // robot 命名空间结束
