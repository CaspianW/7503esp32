#pragma once

#include <stdint.h>

namespace robot {
namespace config {

constexpr float kWheelDiameterM = 0.065f;
constexpr float kWheelRadiusM = kWheelDiameterM * 0.5f;

// Measure these from the assembled chassis before enabling mecanum motion.
constexpr float kWheelbaseM = 0.0f;  // front-to-rear wheel center distance
constexpr float kTrackWidthM = 0.0f; // left-to-right wheel center distance

// Yahboom lists a 13 PPR encoder and 1:20 gearbox. Its example counts both
// AB edges, giving about 1040 counts per output-shaft revolution.
constexpr float kEncoderCountsPerWheelRev = 1040.0f;

constexpr uint32_t kMotorPwmFrequencyHz = 25000;
constexpr uint32_t kServoPwmFrequencyHz = 50;

constexpr uint32_t kControlPeriodUs = 10000; // 100 Hz
constexpr uint32_t kImuPeriodUs = 10000;     // 100 Hz
constexpr uint32_t kOdometryPeriodUs = 20000; // 50 Hz
constexpr uint32_t kRosSpinPeriodUs = 1000;  // 1 kHz maximum spin cadence

// Tune after measuring the real robot. Zero gains intentionally leave the
// wheel controllers inactive until the motors and encoders are characterized.
constexpr float kWheelPidKp = 0.0f;
constexpr float kWheelPidKi = 0.0f;
constexpr float kWheelPidKd = 0.0f;
constexpr float kWheelPidOutputLimit = 1.0f;
constexpr float kWheelPidIntegralLimit = 1.0f;

constexpr float kMaxLinearSpeedMps = 0.0f; // set after wheel calibration
constexpr float kMaxAngularSpeedRadps = 0.0f; // set after chassis calibration
constexpr uint32_t kCommandTimeoutMs = 500;

} // namespace config
} // namespace robot
