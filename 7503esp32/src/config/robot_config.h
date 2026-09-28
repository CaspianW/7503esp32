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

constexpr float wheelDiameterM = 0.065f;
constexpr float wheelRadiusM = wheelDiameterM * 0.5f;

// 启用麦克纳姆轮运动前，需在组装好的底盘上实测以下尺寸。
constexpr float wheelbaseM = 0.0f;  // 前后轮中心距
constexpr float trackWidthM = 0.0f; // 左右轮中心距

// Yahboom 标称编码器为 13 PPR、减速比为 1:20；其示例对 AB 两路的边沿计数，
// 因此输出轴每转约对应 1040 个计数。
constexpr float encoderCountsPerWheelRev = 1040.0f;

constexpr uint32_t motorPwmFrequencyHz = 25000;
constexpr uint32_t servoPwmFrequencyHz = 50;
constexpr uint16_t imuOutputRateHz = 100;
constexpr uint16_t imuAccelFullScaleG = 16;
constexpr uint16_t imuGyroFullScaleDps = 2000;

// 实车核对电机接线、车轮方向与急停后才能改为 true。
constexpr bool motorOutputsEnabled = false;
// 电机正向与编码器计数正向需在实车上逐轮校准。
constexpr int8_t motorDirection[wheelCount] = {1, 1, 1, 1};
constexpr int8_t encoderDirection[wheelCount] = {1, 1, 1, 1};

// MG996 舵机无外部位置反馈；以下是控制脉宽，不是编码器读数。
// 实车装配并确认机械限位后再标定，零值表示禁止启用升降控制。
constexpr uint16_t liftLoweredPulseUs = 0;
constexpr uint16_t liftRaisedPulseUs = 0;

constexpr uint32_t controlPeriodUs = 10000; // 100 Hz
constexpr uint32_t imuPeriodUs = 10000;     // 100 Hz
constexpr uint32_t odometryPeriodUs = 20000; // 50 Hz
constexpr uint32_t rosSpinPeriodUs = 1000;  // 最高 1 kHz 的执行器轮询频率

// 实测机器人后再整定。增益暂设为零，待电机和编码器特性确认后再启用车轮控制器。
constexpr float wheelPidKp = 0.0f;
constexpr float wheelPidKi = 0.0f;
constexpr float wheelPidKd = 0.0f;
constexpr float wheelPidOutputLimit = 1.0f;
constexpr float wheelPidIntegralLimit = 1.0f;

constexpr float maxLinearSpeedMps = 0.0f; // 车轮校准后设置
constexpr float maxAngularSpeedRadps = 0.0f; // 底盘校准后设置
constexpr uint32_t commandTimeoutMs = 500;

} // config 命名空间结束
} // robot 命名空间结束
