#pragma once

#include <stdint.h>

namespace robot {
namespace config {

constexpr float kWheelDiameterM = 0.065f;
constexpr float kWheelRadiusM = kWheelDiameterM * 0.5f;

// 启用麦克纳姆轮运动前，需在组装好的底盘上实测以下尺寸。
constexpr float kWheelbaseM = 0.0f;  // 前后轮中心距
constexpr float kTrackWidthM = 0.0f; // 左右轮中心距

// Yahboom 标称编码器为 13 PPR、减速比为 1:20；其示例对 AB 两路的边沿计数，
// 因此输出轴每转约对应 1040 个计数。
constexpr float kEncoderCountsPerWheelRev = 1040.0f;

constexpr uint32_t kMotorPwmFrequencyHz = 25000;
constexpr uint32_t kServoPwmFrequencyHz = 50;

constexpr uint32_t kControlPeriodUs = 10000; // 100 Hz
constexpr uint32_t kImuPeriodUs = 10000;     // 100 Hz
constexpr uint32_t kOdometryPeriodUs = 20000; // 50 Hz
constexpr uint32_t kRosSpinPeriodUs = 1000;  // 最高 1 kHz 的执行器轮询频率

// 实测机器人后再整定。增益暂设为零，待电机和编码器特性确认后再启用车轮控制器。
constexpr float kWheelPidKp = 0.0f;
constexpr float kWheelPidKi = 0.0f;
constexpr float kWheelPidKd = 0.0f;
constexpr float kWheelPidOutputLimit = 1.0f;
constexpr float kWheelPidIntegralLimit = 1.0f;

constexpr float kMaxLinearSpeedMps = 0.0f; // 车轮校准后设置
constexpr float kMaxAngularSpeedRadps = 0.0f; // 底盘校准后设置
constexpr uint32_t kCommandTimeoutMs = 500;

} // config 命名空间结束
} // robot 命名空间结束
