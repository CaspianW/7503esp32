/**
 * @file ICM42670P.h
 * @brief Minimal host-test substitute for the IMU library header.
 * @author Caspian Wu
 * @date 2026-10-04
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

/** @brief 主机测试用I2C总线替身。 */
struct FakeWire {
  bool begin(int, int) { return true; }
  void setTimeOut(uint16_t) {}
  void beginTransmission(uint8_t address) { lastAddress = address; }
  uint8_t endTransmission() { return 0; }

  uint8_t lastAddress = 0;
};
extern FakeWire Wire;

struct inv_imu_sensor_event_t {
  int16_t accel[3] = {};
  int16_t gyro[3] = {};
  int16_t temperature = 0;
  int sensor_mask = 0;
};

extern inv_imu_sensor_event_t fakeImuEvent;
extern bool fakeImuDataReady;
extern int fakeImuReadStatus;

class ICM42670 {
public:
  ICM42670(FakeWire &, bool) {}

  int begin() { return 0; }
  int startAccel(uint16_t, uint16_t) { return 0; }
  int startGyro(uint16_t, uint16_t) { return 0; }

  int getDataFromRegisters(inv_imu_sensor_event_t &event) {
    if (fakeImuReadStatus != 0) return fakeImuReadStatus;
    if (fakeImuDataReady) {
      event = fakeImuEvent;
      // 寄存器輪詢接口不提供FIFO的sensor_mask。
      event.sensor_mask = 0;
      fakeImuDataReady = false;
    }
    return 0;
  }

  bool isAccelDataValid(inv_imu_sensor_event_t *event) {
    return (event->sensor_mask & 1) != 0;
  }
  bool isGyroDataValid(inv_imu_sensor_event_t *event) {
    return (event->sensor_mask & 2) != 0;
  }
};
