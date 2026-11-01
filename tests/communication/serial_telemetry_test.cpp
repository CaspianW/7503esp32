#include "serial_test_helpers.h"
#include "../../src/communication/serial_telemetry.h"
#include <limits>

static std::vector<robot::SerialPacket> encode(const robot::SerialTelemetry &f) {
  uint8_t out[robot::serialTelemetryCapacity];
  const size_t size = robot::encodeSerialTelemetry(out, sizeof(out), f, 100);
  assert(size);
  return packets(std::string(reinterpret_cast<char *>(out), size));
}
int main() {
  using robot::PacketType;
  robot::SerialTelemetry f;
  f.uptimeMs = 1000; f.initialized = f.encoderReady = f.speedReady = f.imuReady = true;
  f.odometry = {1, -2, .25f, .1f, -.2f, .3f, 990};
  f.imu = {1, -2, 9.80665f, -.1f, .2f, -.3f, 25, 995};
  f.liftStateKnown = true; f.liftState = 1;
  auto all = encode(f);
  assert(all.size() == 4);
  const auto *o = findPacket(all, PacketType::odometry);
  const auto *i = findPacket(all, PacketType::imu);
  const auto *l = findPacket(all, PacketType::liftState);
  const auto *s = findPacket(all, PacketType::status);
  assert(o && o->length == 28 && robot::readU32LE(o->data) == 990);
  assert(robot::readFloatLE(o->data + 4) == 1 && robot::readFloatLE(o->data + 8) == -2);
  assert(robot::readFloatLE(o->data + 24) == .3f);
  assert(i && i->length == 28 && robot::readU32LE(i->data) == 995);
  assert(robot::readFloatLE(i->data + 12) == 9.80665f);
  assert(robot::readFloatLE(i->data + 24) == -.3f); // No temperature field.
  assert(l && l->length == 1 && l->data[0] == 1);
  assert(s && s->length == 2 && s->data[0] == (1 | 8 | 16 | 32) && s->data[1] == 0);

  f.imu.temperature = std::numeric_limits<float>::quiet_NaN();
  all = encode(f);
  assert(findPacket(all, PacketType::imu));
  f.uptimeMs = 1020;
  all = encode(f);
  assert(robot::readU32LE(findPacket(all, PacketType::imu)->data) == 995);
  f.uptimeMs = 1095;
  all = encode(f);
  assert(!findPacket(all, PacketType::odometry) && !findPacket(all, PacketType::imu));
  assert(findPacket(all, PacketType::liftState));
  assert(findPacket(all, PacketType::status)->data[0] == (1 | 32));

  f.uptimeMs = 1000;
  f.odometry.timestamp = f.imu.timestamp = 0;
  assert(encode(f).size() == 2);
  f.odometry.timestamp = f.imu.timestamp = 990;
  f.initialized = false;
  assert(encode(f).size() == 2);
  f.initialized = true; f.encoderReady = f.imuReady = false;
  assert(encode(f).size() == 2);
  f.encoderReady = f.imuReady = true;
  f.odometry.x = std::numeric_limits<float>::quiet_NaN();
  f.imu.gyroZ = std::numeric_limits<float>::infinity();
  assert(encode(f).size() == 2);
  f.odometry.x = 1; f.imu.gyroZ = -.3f;
  f.uptimeMs = 20; f.odometry.timestamp = f.imu.timestamp = UINT32_MAX - 10;
  assert(encode(f).size() == 4);
  f.odometry.timestamp = f.imu.timestamp = 30;
  assert(encode(f).size() == 2);
  f.liftStateKnown = false;
  assert(encode(f).size() == 1);
  uint8_t small[5];
  assert(robot::encodeSerialTelemetry(small, sizeof(small), f, 100) == 0);
  assert(robot::encodeSerialTelemetry(nullptr, 100, f, 100) == 0);
}
