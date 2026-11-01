/**
 * @file imu_driver_test.cpp
 * @brief IMU寄存器轮询样本更新测试。
 * @author Caspian Wu
 * @date 2026-10-24
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../../src/drivers/include/imu.h"

#include <cassert>
#include <cmath>

int main() {
  robot::Imu imu;
  assert(imu.begin());

  fakeImuEvent = {};
  fakeImuEvent.accel[2] = 2048;
  fakeImuEvent.gyro[2] = 16384;
  fakeImuEvent.temperature = 128;
  fakeImuDataReady = true;

  robot::ImuSample sample{};
  assert(imu.read(sample));
  assert(std::fabs(sample.accelZ - 9.80665f) < 0.001f);
  assert(std::fabs(sample.gyroZ - 17.45329f) < 0.001f);
  assert(std::fabs(sample.temperature - 26.0f) < 0.001f);
  const uint32_t timestamp = sample.timestamp;

  // 本次没有DRDY时不能把零样本伪装成新数据。
  assert(!imu.read(sample));
  assert(sample.timestamp == timestamp);

  // 新一帧寄存器数据应再次更新样本，即使FIFO sensor_mask为零。
  fakeImuEvent.accel[0] = 1024;
  fakeImuEvent.accel[2] = 0;
  fakeImuEvent.gyro[2] = 0;
  fakeImuDataReady = true;
  assert(imu.read(sample));
  assert(std::fabs(sample.accelX - 4.903325f) < 0.001f);
  assert(sample.timestamp >= timestamp);

  fakeImuReadStatus = -1;
  assert(!imu.read(sample));
}
