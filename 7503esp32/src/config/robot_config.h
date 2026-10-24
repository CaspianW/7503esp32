/**
 * @file robot_config.h
 * @brief Robot and chassis configuration parameters.
 * @author Caspian Wu
 * @date 2026-10-22
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
constexpr float wheelbase = 0.142f;  // 前后轮中心距，单位 m
constexpr float trackWidth = 0.17f; // 左右轮中心距，单位 m

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
// 升降单独使能；端点必须标定为500～2500µs内不同的值。
constexpr bool liftOutputsEnabled = false;
constexpr uint8_t liftServoChannel = 0; // 0=S1(GPIO8)，1=S2(GPIO21)
constexpr uint32_t liftCommandTimeout = 500; // 单位 ms，失去心跳后停止PWM

constexpr uint32_t controlPeriod = 10000;  // 单位 µs，100 Hz
constexpr uint32_t imuPeriod = 10000;      // 单位 µs，100 Hz
constexpr uint32_t odometryPeriod = 20000; // 单位 µs，50 Hz
constexpr uint32_t rosSpinPeriod = 1000;   // 单位 µs，最高 1 kHz

// 实测机器人后再整定。增益暂设为零，待电机和编码器特性确认后再启用车轮控制器。
constexpr float wheelPidKp = 0.0f;
constexpr float wheelPidKi = 0.0f;
constexpr float wheelPidKd = 0.0f;
constexpr float wheelPidOutputLimit = 0.2f;
constexpr float wheelPidIntegralLimit = 1.0f;

constexpr float maxLinearSpeed = 0.0f;  // 单位 m/s，车轮校准后设置
constexpr float maxAngularSpeed = 0.0f; // 单位 rad/s，底盘校准后设置
constexpr uint32_t commandTimeout = 500; // 单位 ms

// 开环台架与闭环整车模式互斥；三个确认项均满足后才接受 arm/vel。
constexpr bool chassisControlEnabled = false;
constexpr bool wheelMappingConfirmed = false;
// 索引为逻辑轮序 FL、RL、FR、RR，值为电气通道 M1～M4 的 0～3。
constexpr uint8_t wheelChannel[wheelCount] = {0, 1, 2, 3};
constexpr float maxWheelSpeed = 0.0f; // 单位 m/s，需按实测电机能力设置
constexpr float maxWheelAcceleration = 0.5f; // 单位 m/s²
constexpr float motorDutyLimit = 0.2f; // 初次调试全局占空比上限
constexpr uint32_t maxControlGap = 50000; // 单位 µs，超时锁定停机
constexpr uint32_t telemetryPeriod = 100000; // 单位 µs，10 Hz
constexpr uint32_t imuFreshness = 100; // 单位 ms
// 未确认安装朝向前里程计使用编码器推算的角速度。
constexpr bool useImuYaw = false;
constexpr int8_t imuYawDirection = 1; // 芯片 Z 轴与车体 Z 轴同向为 +1
// 持续输出但编码器无反馈或方向相反时锁定停机；阈值仍需实车验证。
constexpr uint32_t feedbackTimeout = 500; // 单位 ms
constexpr float feedbackMinDuty = 0.1f;
constexpr float feedbackMinTarget = 0.02f; // 单位 m/s
constexpr float feedbackMinSpeed = 0.002f; // 单位 m/s
constexpr uint32_t serialBaud = 115200;
// 可选外接常闭急停：正常闭合到 GND，触发/断线为 HIGH；-1 表示未接入。
constexpr int8_t emergencyStopPin = -1;
constexpr bool emergencyStopActiveHigh = true;

} // config 命名空间结束
} // robot 命名空间结束
