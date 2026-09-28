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

// Yahboom Micro-ROS Controller Board. Index order: M1, M2, M3, M4.
// The motor cable's white connector goes to this board.
constexpr uint8_t kMotorCount = 4;
constexpr MotorPins kMotorPins[kMotorCount] = {
    {4, 5, 6, 7},     // M1: front-left
    {15, 16, 47, 48}, // M2: rear-left
    {9, 10, 11, 12},  // M3: front-right
    {13, 14, 1, 2},   // M4: rear-right
};

constexpr uint8_t kImuSda = 40;
constexpr uint8_t kImuScl = 39;
constexpr int8_t kImuInterrupt = 41;

constexpr uint8_t kServoS1 = 8;
constexpr uint8_t kServoS2 = 21;

constexpr uint8_t kBatterySense = 3;
constexpr uint8_t kBuzzer = 46;
constexpr uint8_t kStatusLed = 45;

// The board's Type-C serial interface uses UART0: RX=GPIO43, TX=GPIO44.
// Reserve this port for the micro-ROS agent connection during robot operation.

} // namespace board
} // namespace robot
