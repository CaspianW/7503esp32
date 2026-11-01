#pragma once
#include "../../src/communication/serial_protocol.h"
#include <cassert>
#include <initializer_list>
#include <string>
#include <vector>

inline std::string wire(robot::PacketType type, const uint8_t *data, size_t size) {
  uint8_t out[robot::serialMaxPacket];
  const size_t count = robot::encodeSerialPacket(out, sizeof(out), type, data, size);
  assert(count);
  return std::string(reinterpret_cast<const char *>(out), count);
}
inline std::string wire(robot::PacketType type, std::initializer_list<uint8_t> data = {}) {
  return wire(type, data.begin(), data.size());
}
inline std::string velocityWire(float vx, float vy, float wz) {
  uint8_t data[12];
  robot::writeFloatLE(data, vx); robot::writeFloatLE(data + 4, vy);
  robot::writeFloatLE(data + 8, wz);
  return wire(robot::PacketType::velocity, data, sizeof(data));
}
inline std::vector<robot::SerialPacket> packets(const std::string &bytes) {
  robot::SerialPacketParser parser;
  robot::SerialPacket packet;
  std::vector<robot::SerialPacket> result;
  for (unsigned char byte : bytes)
    if (parser.feed(byte, packet)) {
      result.push_back(packet);
      while (parser.take(packet)) result.push_back(packet);
    }
  return result;
}
inline const robot::SerialPacket *findPacket(const std::vector<robot::SerialPacket> &values,
                                            robot::PacketType type) {
  for (const auto &value : values) if (value.type == type) return &value;
  return nullptr;
}
inline bool acknowledged(const std::string &bytes, robot::PacketType type, uint8_t result) {
  for (const auto &p : packets(bytes))
    if (p.type == robot::PacketType::ack && p.length == 2 &&
        p.data[0] == static_cast<uint8_t>(type) && p.data[1] == result) return true;
  return false;
}
