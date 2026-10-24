/**
 * @file imu.cpp
 * @brief ICM-42670-P IMU driver.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../include/imu.h"

#include "../../config/board_pins.h"
#include "../../config/robot_config.h"

#include <Arduino.h>

namespace robot {
namespace {

constexpr float gravity = 9.80665f; // 单位 m/s²
constexpr float pi = 3.14159265358979323846f;
constexpr float rawScale = 32768.0f;

// 检测I2C地址
bool respondsAt(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

} // 匿名命名空间结束

bool Imu::begin() {
  if (initialized_) {
    return true;
  }
  if (!Wire.begin(board::imuSda, board::imuScl)) {
    return false;
  }

  // ICM42670P 的 AD0 接线决定地址为 0x68 或 0x69；自动检测两者。
  if (respondsAt(0x68)) {
    sensor_ = &sensorLowAddress_;
  } else if (respondsAt(0x69)) {
    sensor_ = &sensorHighAddress_;
  } else {
    return false;
  }
  if (sensor_->begin() != 0 ||
      sensor_->startAccel(config::imuOutputRate,
                          config::imuAccelFullScale) != 0 ||
      sensor_->startGyro(config::imuOutputRate,
                         config::imuGyroFullScale) != 0) {
    sensor_ = nullptr;
    return false;
  }
  Wire.setTimeOut(3); // 单次 I2C 事务超时，单位 ms
  delay(100);
  initialized_ = true;
  return true;
}

bool Imu::read(ImuSample &sample) {
  if (!initialized_ || sensor_ == nullptr) {
    return false;
  }
  inv_imu_sensor_event_t event{};
  if (sensor_->getDataFromRegisters(event) != 0 ||
      !sensor_->isAccelDataValid(&event) ||
      !sensor_->isGyroDataValid(&event)) {
    return false;
  }

  const float accelScale =
      config::imuAccelFullScale * gravity / rawScale; // 单位 m/s²/LSB
  const float gyroScale =
      config::imuGyroFullScale * (pi / 180.0f) / rawScale; // 单位 rad/s/LSB
  sample.accelX = event.accel[0] * accelScale;
  sample.accelY = event.accel[1] * accelScale;
  sample.accelZ = event.accel[2] * accelScale;
  sample.gyroX = event.gyro[0] * gyroScale - gyroBias_[0];
  sample.gyroY = event.gyro[1] * gyroScale - gyroBias_[1];
  sample.gyroZ = event.gyro[2] * gyroScale - gyroBias_[2];
  sample.temperature = event.temperature / 128.0f + 25.0f;
  sample.timestamp = millis();
  // 这里仍是芯片自身坐标轴；装板方向需实测后再转换到 ROS base_link。
  return true;
}

bool Imu::calibrateGyro(uint16_t sampleCount) {
  if (!initialized_ || sampleCount == 0) {
    return false;
  }
  const float previousBias[3] = {gyroBias_[0], gyroBias_[1], gyroBias_[2]}; // rad/s
  double sum[3] = {}; // 单位 rad/s
  for (uint16_t i = 0; i < sampleCount; ++i) {
    ImuSample sample{};
    if (!read(sample)) {
      return false;
    }
    sum[0] += sample.gyroX + previousBias[0];
    sum[1] += sample.gyroY + previousBias[1];
    sum[2] += sample.gyroZ + previousBias[2];
    delay(1000 / config::imuOutputRate);
  }
  for (uint8_t axis = 0; axis < 3; ++axis) {
    gyroBias_[axis] = static_cast<float>(sum[axis] / sampleCount);
  }
  return true;
}

} // robot 命名空间结束
