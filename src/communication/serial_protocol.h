/**
 * @file serial_protocol.h
 * @brief Binary UART packet definitions and timeout-aware stream parser.
 * @author Caspian Wu
 * @date 2026-11-01
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once
#include <stddef.h>
#include <stdint.h>

namespace robot {
enum class PacketType : uint8_t {
  velocity = 0x01, lift = 0x02, arm = 0x03, emergencyStop = 0x04,
  stop = 0x05, statusRequest = 0x06,
  odometry = 0x81, imu = 0x82, liftState = 0x83, status = 0x84, ack = 0xF0
};
constexpr size_t serialMaxPayload = 32;
constexpr size_t serialMaxPacket = serialMaxPayload + 8;
struct SerialPacket {
  PacketType type = PacketType::statusRequest;
  uint8_t length = 0;
  uint8_t data[serialMaxPayload] = {};
};

// AA 55 TYPE LEN DATA CRC16_LO CRC16_HI 0D 0A.
// CRC-16/Modbus covers TYPE, LEN and DATA. Integers/floats use little endian.
uint16_t serialCrc16(const uint8_t *data, size_t size);
size_t encodeSerialPacket(uint8_t *out, size_t capacity, PacketType type,
                          const uint8_t *data, size_t size);
void writeU32LE(uint8_t *out, uint32_t value);
uint32_t readU32LE(const uint8_t *data);
void writeFloatLE(uint8_t *out, float value);
float readFloatLE(const uint8_t *data);

class SerialPacketParser {
public:
  bool feed(uint8_t byte, SerialPacket &packet);
  // 超时后跳过不完整候选帧，保留后面 CRC 正确的完整包。
  bool take(SerialPacket &packet, bool expired = false);
  void reset() { size_ = 0; }
  bool pending() const { return size_ != 0; }
private:
  void discard(size_t count);
  uint8_t buffer_[serialMaxPacket] = {};
  size_t size_ = 0;
};
} // namespace robot
