#include "serial_telemetry.h"
#include "serial_protocol.h"
#include <math.h>

namespace robot {
namespace {
bool fresh(uint32_t now, uint32_t stamp, uint32_t maxAge) {
  return stamp != 0 && static_cast<uint32_t>(now - stamp) < maxAge;
}
}
size_t encodeSerialTelemetry(uint8_t *out, size_t capacity,
                              const SerialTelemetry &f, uint32_t maxSampleAgeMs) {
  if (!out) return 0;
  const auto &o = f.odometry;
  const auto &i = f.imu;
  const bool odomValid = f.initialized && f.encoderReady && f.speedReady &&
    fresh(f.uptimeMs, o.timestamp, maxSampleAgeMs) &&
    isfinite(o.x) && isfinite(o.y) && isfinite(o.yaw) &&
    isfinite(o.vx) && isfinite(o.vy) && isfinite(o.wz);
  const bool imuValid = f.initialized && f.imuReady &&
    fresh(f.uptimeMs, i.timestamp, maxSampleAgeMs) &&
    isfinite(i.accelX) && isfinite(i.accelY) && isfinite(i.accelZ) &&
    isfinite(i.gyroX) && isfinite(i.gyroY) && isfinite(i.gyroZ);
  uint8_t payload[28] = {};
  payload[0] = (f.initialized ? 1 : 0) | (f.armed ? 2 : 0) |
    (f.emergencyStopped ? 4 : 0) | (odomValid ? 8 : 0) |
    (imuValid ? 16 : 0) | (f.liftStateKnown ? 32 : 0);
  payload[1] = f.fault;
  size_t used = encodeSerialPacket(out, capacity, PacketType::status, payload, 2);
  if (!used) return 0;
  const auto append = [&](PacketType type, size_t length) -> bool {
    const size_t size = encodeSerialPacket(out + used, capacity - used, type, payload, length);
    if (!size) return false;
    used += size;
    return true;
  };
  if (odomValid) {
    writeU32LE(payload, o.timestamp);
    const float values[] = {o.x, o.y, o.yaw, o.vx, o.vy, o.wz};
    for (uint8_t n = 0; n < 6; ++n) writeFloatLE(payload + 4 + n * 4, values[n]);
    if (!append(PacketType::odometry, 28)) return 0;
  }
  if (imuValid) {
    writeU32LE(payload, i.timestamp);
    const float values[] = {i.accelX, i.accelY, i.accelZ, i.gyroX, i.gyroY, i.gyroZ};
    for (uint8_t n = 0; n < 6; ++n) writeFloatLE(payload + 4 + n * 4, values[n]);
    if (!append(PacketType::imu, 28)) return 0;
  }
  if (f.liftStateKnown) {
    payload[0] = f.liftState;
    if (!append(PacketType::liftState, 1)) return 0;
  }
  return used;
}
} // namespace robot
