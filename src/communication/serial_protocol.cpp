/**
 * @file serial_protocol.cpp
 * @brief Binary UART encoding, CRC and stream recovery.
 * @author Caspian Wu
 * @date 2026-11-01
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "serial_protocol.h"
#include <limits>
#include <string.h>

namespace robot {
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "Serial protocol requires IEEE-754 float32");

uint16_t serialCrc16(const uint8_t *data, size_t size) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : crc >> 1;
  }
  return crc;
}
void writeU32LE(uint8_t *out, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(value >> (8 * i));
}
uint32_t readU32LE(const uint8_t *data) {
  uint32_t value = 0;
  for (uint8_t i = 0; i < 4; ++i) value |= static_cast<uint32_t>(data[i]) << (8 * i);
  return value;
}
void writeFloatLE(uint8_t *out, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  writeU32LE(out, bits);
}
float readFloatLE(const uint8_t *data) {
  const uint32_t bits = readU32LE(data);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}
size_t encodeSerialPacket(uint8_t *out, size_t capacity, PacketType type,
                          const uint8_t *data, size_t size) {
  if (!out || size > serialMaxPayload || capacity < size + 8 || (size && !data)) return 0;
  out[0] = 0xAA; out[1] = 0x55;
  out[2] = static_cast<uint8_t>(type); out[3] = static_cast<uint8_t>(size);
  if (size) memcpy(out + 4, data, size);
  const uint16_t crc = serialCrc16(out + 2, size + 2);
  out[4 + size] = static_cast<uint8_t>(crc);
  out[5 + size] = static_cast<uint8_t>(crc >> 8);
  out[6 + size] = 0x0D; out[7 + size] = 0x0A;
  return size + 8;
}
void SerialPacketParser::discard(size_t count) {
  size_ -= count;
  if (size_) memmove(buffer_, buffer_ + count, size_);
}
bool SerialPacketParser::feed(uint8_t byte, SerialPacket &packet) {
  if (size_ == sizeof(buffer_)) discard(1);
  buffer_[size_++] = byte;
  return take(packet);
}
bool SerialPacketParser::take(SerialPacket &packet, bool expired) {
  while (size_) {
    if (buffer_[0] != 0xAA) { discard(1); continue; }
    if (size_ < 2) {
      if (expired) { discard(1); continue; }
      return false;
    }
    if (buffer_[1] != 0x55) { discard(1); continue; }
    if (size_ < 4) {
      if (expired) { discard(1); continue; }
      return false;
    }
    const uint8_t length = buffer_[3];
    if (length > serialMaxPayload) { discard(1); continue; }
    const size_t total = length + 8;
    if (size_ < total) {
      if (expired) { discard(1); continue; }
      return false;
    }
    const uint16_t crc = static_cast<uint16_t>(buffer_[4 + length]) |
                         static_cast<uint16_t>(buffer_[5 + length]) << 8;
    if (buffer_[6 + length] != 0x0D || buffer_[7 + length] != 0x0A ||
        serialCrc16(buffer_ + 2, length + 2) != crc) {
      discard(1); continue;
    }
    packet.type = static_cast<PacketType>(buffer_[2]);
    packet.length = length;
    if (length) memcpy(packet.data, buffer_ + 4, length);
    discard(total);
    return true;
  }
  return false;
}
} // namespace robot
