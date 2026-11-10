#include "motor_debug.h"
#include "../app/robot_controller.h"
#include "../config/board_pins.h"
#include "../config/robot_config.h"
#include <Arduino.h>
#include <stdio.h>

namespace robot {
void MotorDebug::begin() {
  Serial.setTxBufferSize(2048);
  Serial.begin(config::serialBaud, SERIAL_8N1, board::serialRx, board::serialTx);
  lastPrintMs_ = millis();
}

void MotorDebug::update(const RobotController &c) {
  // Discard input with a bounded budget; visual commands cannot arm the robot.
  for (uint8_t budget = 64; budget && Serial.available(); --budget) Serial.read();
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - lastPrintMs_) < 100) return;
  lastPrintMs_ = now;
  char buffer[768];
  int count = snprintf(buffer, sizeof(buffer),
    "ms=%lu init=%u armed=%u fault=%u encoder_ok=%u speed_ok=%u\r\n",
    static_cast<unsigned long>(now), unsigned(c.initialized()), unsigned(c.armed()),
    unsigned(c.fault()), unsigned(c.encoderReady()), unsigned(c.speedReady()));
  if (count < 0 || static_cast<size_t>(count) >= sizeof(buffer)) return;
  size_t used = static_cast<size_t>(count);
  const char *names[] = {"FL", "RL", "FR", "RR"};
  for (uint8_t i = 0; i < wheelCount; ++i) {
    const uint8_t channel = config::wheelChannel[i];
    count = snprintf(buffer + used, sizeof(buffer) - used,
      "%s(M%u) ticks=%lld speed=%.4f m/s target=%.4f m/s duty=%.1f%%\r\n",
      names[i], unsigned(channel + 1), static_cast<long long>(c.ticks()[channel]),
      static_cast<double>(c.wheelSpeed().value[i]),
      static_cast<double>(c.wheelTarget().value[i]),
      static_cast<double>(c.duty().value[i] * 100.0f));
    if (count < 0 || static_cast<size_t>(count) >= sizeof(buffer) - used) return;
    used += static_cast<size_t>(count);
  }
  // Skip a sample if UART is congested instead of delaying the control loop.
  if (Serial.availableForWrite() >= static_cast<int>(used))
    Serial.write(reinterpret_cast<const uint8_t *>(buffer), used);
}
}
