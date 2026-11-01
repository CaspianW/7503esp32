#include "serial_test_helpers.h"
#include <cstring>
#include <limits>

int main() {
  using robot::PacketType;
  const uint8_t reference[] = {'1','2','3','4','5','6','7','8','9'};
  assert(robot::serialCrc16(reference, sizeof(reference)) == 0x4B37);
  uint8_t bytes[4];
  robot::writeU32LE(bytes, 0x78563412);
  assert(bytes[0] == 0x12 && bytes[1] == 0x34 && bytes[2] == 0x56 && bytes[3] == 0x78);
  assert(robot::readU32LE(bytes) == 0x78563412);
  robot::writeFloatLE(bytes, 1.0f);
  const uint8_t one[] = {0, 0, 0x80, 0x3F};
  assert(!memcmp(bytes, one, 4));
  assert(robot::readFloatLE(bytes) == 1.0f);
  robot::writeFloatLE(bytes, -0.25f);
  assert(robot::readFloatLE(bytes) == -0.25f);

  const std::string valid = wire(PacketType::lift, {1});
  const uint8_t golden[] = {0xAA, 0x55, 0x02, 0x01, 0x01, 0x10, 0x50, 0x0D, 0x0A};
  assert(valid.size() == sizeof(golden) && !memcmp(valid.data(), golden, sizeof(golden)));
  assert(valid.size() == 9);
  assert(static_cast<uint8_t>(valid[0]) == 0xAA && valid[1] == 0x55);
  assert(valid[2] == 2 && valid[3] == 1 && valid[7] == 0x0D && valid[8] == 0x0A);
  auto decoded = packets(valid);
  assert(decoded.size() == 1 && decoded[0].type == PacketType::lift && decoded[0].data[0] == 1);
  assert(packets(valid.substr(0, 5)).empty());
  assert(packets(valid + valid).size() == 2);
  assert(packets(std::string("boot log\r\n") + valid).size() == 1);

  // Corrupt every byte, then ensure the next valid packet is still recovered.
  for (size_t n = 0; n < valid.size(); ++n) {
    std::string bad = valid;
    bad[n] ^= 0x80;
    assert(packets(bad).empty());
    const auto recovered = packets(bad + valid);
    assert(recovered.size() == 1 && recovered[0].data[0] == 1);
  }
  // Embedded headers/tails are data, not packet boundaries.
  const uint8_t payload[] = {0xAA, 0x55, 0x0D, 0x0A, 0, 0xFF};
  decoded = packets(wire(PacketType::imu, payload, sizeof(payload)));
  assert(decoded.size() == 1 && !memcmp(decoded[0].data, payload, sizeof(payload)));

  std::string oversized("\xAA\x55\x01\xFF", 4);
  assert(packets(oversized + valid).size() == 1);
  // False long frame can contain two real short frames; drain both after resync.
  std::string broken("\xAA\x55\x01\x14", 4);
  const auto recovered = packets(broken + valid + valid + std::string(6, '\0'));
  assert(recovered.size() == 2);

  robot::SerialPacketParser parser;
  robot::SerialPacket p;
  assert(!parser.feed(0xAA, p) && parser.pending());
  parser.reset();
  assert(!parser.pending());
  for (uint8_t value : {0xAA, 0xAA, 0x55}) parser.feed(value, p);
  assert(parser.pending());
  parser.reset();

  uint8_t out[robot::serialMaxPacket];
  assert(robot::encodeSerialPacket(nullptr, sizeof(out), PacketType::lift, payload, 1) == 0);
  assert(robot::encodeSerialPacket(out, 8, PacketType::lift, payload, 1) == 0);
  assert(robot::encodeSerialPacket(out, sizeof(out), PacketType::lift, nullptr, 1) == 0);
  assert(robot::encodeSerialPacket(out, sizeof(out), PacketType::lift, payload, 33) == 0);
}
