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
constexpr float encoderCounts = 1040.0f; // 速度换算比例，单位 count/rev

constexpr uint32_t motorPwmFrequency = 25000; // 单位 Hz
constexpr uint32_t servoPwmFrequency = 50;    // 单位 Hz
constexpr uint16_t imuOutputRate = 100;       // 单位 Hz
constexpr uint16_t imuAccelFullScale = 16;    // 单位 g
constexpr uint16_t imuGyroFullScale = 2000;   // 单位 °/s

// 实车核对电机接线、车轮方向与急停后才能改为 true。
constexpr bool motorOutputsEnabled = true;
// 暂停台架测试；以后重启台架时按已测起动占空比测试。
constexpr bool motorBenchEnabled = false;
constexpr float motorBenchDuty = 0.7f; // 单位占空比，四电机台架测试输出
// 保留手动开环代码，需要重新标定硬件时可关闭底盘闭环后启用。
constexpr bool manualOpenLoopEnabled = true;
constexpr float manualDuty = 0.7f; // 单位占空比，键盘控制时活动车轮的固定输出
// 开环期间只采样编码器并回传计数/轮速，不参与电机控制。
constexpr bool encoderReadEnabled = true;
// 按电气通道M1～M4排列；已按前进测试补偿左右电机的镜像安装方向。
constexpr int8_t motorDirection[wheelCount] = {-1, -1, 1, 1};
// 按M1～M4排列；按小车正前方向手转时原始计数为+、+、-、-，统一修正为正。
constexpr int8_t encoderDirection[wheelCount] = {1, 1, -1, -1};

// MG996舵机无外部位置反馈；以下是控制脉宽，不是编码器读数。
constexpr uint16_t liftLoweredPulse = 1700; // 单位 µs，已测底部
constexpr uint16_t liftRaisedPulse = 500;   // 单位 µs，已测顶部
// 舵机上电即初始化PWM；端点须在500～2500µs内且互不相同。
constexpr bool liftOutputsEnabled = true;
constexpr uint8_t liftServoChannel = 1; // 0=S1(GPIO8)，1=S2(GPIO21)；当前接S2

constexpr uint32_t controlPeriod = 10000;  // 单位 µs，100 Hz
constexpr uint32_t imuPeriod = 10000;      // 单位 µs，100 Hz
constexpr uint32_t odometryPeriod = 20000; // 单位 µs，50 Hz
constexpr uint32_t rosSpinPeriod = 1000;   // 单位 µs，最高 1 kHz

// 根据70%占空比约0.63～0.69m/s的空载实测值设置首轮闭环整定参数。
// 首轮测试不使用D项，避免编码器量化噪声被放大。
constexpr float wheelPidKp = 0.4f;
constexpr float wheelPidKi = 0.5f;
constexpr float wheelPidKd = 0.0f;
constexpr float wheelPidOutputLimit = 0.7f; // 单位占空比，使用已验证的测试上限
constexpr float wheelPidIntegralLimit = 1.0f;
// 实测静摩擦和速度前馈：duty = sign(target) * wheelKs + wheelKv * target。
constexpr float wheelKs = 0.515f; // 单位占空比，按0.55～0.75实测拟合
constexpr float wheelKv = 0.275f; // 单位 (占空比)/(m/s)
constexpr float wheelMinDuty = 0.6f; // 单位占空比，非零轮速目标的最低输出

constexpr float maxLinearSpeed = 0.5f;  // 单位 m/s，低于当前实测空载轮速
constexpr float maxAngularSpeed = 2.0f; // 单位 rad/s
constexpr uint32_t commandTimeout = 500; // 单位 ms

// 开环台架与闭环整车模式互斥；三个确认项均满足后才接受 arm/vel。
constexpr bool chassisControlEnabled = true;
constexpr bool wheelMappingConfirmed = true;
// 索引为逻辑轮序 FL、RL、FR、RR，值为电气通道 M1～M4 的 0～3。
// 实车接线：FL=M4、RL=M3、FR=M2、RR=M1。
constexpr uint8_t wheelChannel[wheelCount] = {3, 2, 1, 0};
constexpr float maxWheelSpeed = 0.7f; // 单位 m/s，按四轮当前最大实测值取整
// 当前不人为延迟起转；100Hz控制下每周期最多变化0.5m/s，可覆盖全部平移测试。
constexpr float maxWheelAcceleration = 50.0f; // 单位 m/s²
constexpr float motorDutyLimit = 1.0f; // 单位占空比，闭环输出最大值
constexpr uint32_t maxControlGap = 50000; // 单位 µs，超时锁定停机
constexpr uint32_t telemetryPeriod = 100000; // 单位 µs，10 Hz
constexpr uint32_t imuFreshness = 100; // 单位 ms
constexpr uint32_t telemetrySampleMaxAge = 100; // 单位 ms，过期的 O/I 样本不上报
// 未确认安装朝向前里程计使用编码器推算的角速度。
constexpr bool useImuYaw = false;
constexpr int8_t imuYawDirection = 1; // 芯片 Z 轴与车体 Z 轴同向为 +1
// 持续输出但编码器无反馈或方向相反时锁定停机；阈值仍需实车验证。
constexpr uint32_t feedbackTimeout = 500; // 单位 ms
constexpr float feedbackMinDuty = 0.6f; // 单位占空比，低于实测起转值时不判定失速
constexpr float feedbackMinTarget = 0.02f; // 单位 m/s
constexpr float feedbackMinSpeed = 0.002f; // 单位 m/s
constexpr uint32_t serialBaud = 115200;
// 可选外接常闭急停：正常闭合到 GND，触发/断线为 HIGH；-1 表示未接入。
constexpr int8_t emergencyStopPin = -1;
constexpr bool emergencyStopActiveHigh = true;

} // config 命名空间结束
} // robot 命名空间结束
