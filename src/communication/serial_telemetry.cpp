/**
 * @file serial_telemetry.cpp
 * @brief Encode MCU status and measured data using the S/E/O/I/L wire contract.
 */
#include "serial_telemetry.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

namespace robot {
namespace {
class Writer {
public:
  Writer(char *buffer, size_t capacity) : buffer_(buffer), capacity_(capacity) {}
  void append(const char *format, ...) {
    if (!valid_) return;
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(buffer_ + size_, capacity_ - size_, format, args);
    va_end(args);
    if (length < 0 || static_cast<size_t>(length) >= capacity_ - size_) {
      valid_ = false;
      return;
    }
    size_ += static_cast<size_t>(length);
  }
  size_t finish() {
    if (!valid_) { buffer_[0] = '\0'; return 0; }
    return size_;
  }
private:
  char *buffer_;
  size_t capacity_;
  size_t size_ = 0;
  bool valid_ = true;
};

bool fresh(uint32_t now, uint32_t timestamp, uint32_t maxAge) {
  // Timestamp 0 denotes a struct that has never received a measurement.
  return timestamp != 0 && static_cast<uint32_t>(now - timestamp) < maxAge;
}
bool finiteOdometry(const OdometryState &o) {
  return isfinite(o.x) && isfinite(o.y) && isfinite(o.yaw) &&
    isfinite(o.vx) && isfinite(o.vy) && isfinite(o.wz);
}
bool finiteImu(const ImuSample &i) {
  return isfinite(i.accelX) && isfinite(i.accelY) && isfinite(i.accelZ) &&
    isfinite(i.gyroX) && isfinite(i.gyroY) && isfinite(i.gyroZ) && isfinite(i.temperature);
}
} // namespace

size_t formatSerialTelemetry(char *buffer, size_t capacity,
                             const SerialTelemetry &f, uint32_t maxSampleAgeMs) {
  if (!buffer || !capacity) return 0;
  const auto &o = f.odometry;
  const auto &i = f.imu;
  const bool odomValid = f.initialized && f.encoderReady && f.speedReady &&
    fresh(f.uptimeMs, o.timestamp, maxSampleAgeMs) && finiteOdometry(o);
  const bool imuValid = f.initialized && f.imuReady &&
    fresh(f.uptimeMs, i.timestamp, maxSampleAgeMs) && finiteImu(i);
  Writer out(buffer, capacity);
  out.append("S %lu %u %u %u %u %u %u %u %u %u %u %u %u\n",
    static_cast<unsigned long>(f.uptimeMs), static_cast<unsigned>(f.initialized),
    static_cast<unsigned>(f.armed), static_cast<unsigned>(f.motionAllowed),
    static_cast<unsigned>(f.emergencyStopped), static_cast<unsigned>(f.fault),
    static_cast<unsigned>(f.encoderReady), static_cast<unsigned>(imuValid),
    static_cast<unsigned>(f.speedReady), static_cast<unsigned>(f.benchMode),
    static_cast<unsigned>(f.motorOutputsEnabled), static_cast<unsigned>(f.chassisControlEnabled),
    static_cast<unsigned>(f.manualMode));
  out.append("E %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.4f %.4f %.4f %.4f\n",
    f.wheelTarget.value[0], f.wheelTarget.value[1], f.wheelTarget.value[2], f.wheelTarget.value[3],
    f.wheelSpeed.value[0], f.wheelSpeed.value[1], f.wheelSpeed.value[2], f.wheelSpeed.value[3],
    f.duty.value[0], f.duty.value[1], f.duty.value[2], f.duty.value[3]);
  if (odomValid)
    out.append("O %lu %.5f %.5f %.5f %.5f %.5f %.5f\n",
      static_cast<unsigned long>(o.timestamp), o.x, o.y, o.yaw, o.vx, o.vy, o.wz);
  if (imuValid)
    out.append("I %lu %.5f %.5f %.5f %.5f %.5f %.5f %.3f\n",
      static_cast<unsigned long>(i.timestamp), i.accelX, i.accelY, i.accelZ,
      i.gyroX, i.gyroY, i.gyroZ, i.temperature);
  out.append("L %u %u %u %u %u %u\n",
    static_cast<unsigned>(f.liftOutputsEnabled), static_cast<unsigned>(f.liftConfigured),
    static_cast<unsigned>(f.liftReady), static_cast<unsigned>(f.liftActive),
    static_cast<unsigned>(f.liftTarget), static_cast<unsigned>(f.liftPulseUs));
  return out.finish();
}
} // namespace robot
