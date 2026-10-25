/**
 * @file Arduino.h
 * @brief Arduino hardware and serial boundaries for host tests.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>

constexpr int INPUT_PULLUP = 2;
constexpr int OUTPUT = 1;
constexpr int HIGH = 1;
constexpr int LOW = 0;
extern uint32_t fakeNow;
extern uint32_t fakePwm[8];
extern uint32_t fakeFreq[8];
extern int fakePin[49];
extern int fakeAttached[49];
extern int fakeSetupFailure;
inline uint32_t millis() { return fakeNow / 1000; }
inline uint32_t micros() { return fakeNow; }
inline void delay(unsigned value) { fakeNow += value * 1000; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int value) { fakePin[pin] = value; }
inline int digitalRead(int pin) { return fakePin[pin]; }
inline double ledcSetup(uint8_t channel, double frequency, uint8_t) {
  fakeFreq[channel] = channel == fakeSetupFailure ? 0 : frequency;
  return channel == fakeSetupFailure ? 0 : frequency;
}
inline uint32_t ledcRead(uint8_t channel) { return fakePwm[channel]; }
inline uint32_t ledcReadFreq(uint8_t channel) { return fakePwm[channel] ? fakeFreq[channel] : 0; }
inline void ledcWrite(uint8_t channel, uint32_t duty) { fakePwm[channel] = duty; }
inline void ledcAttachPin(uint8_t pin, uint8_t channel) { fakeAttached[pin] = channel; }
inline void ledcDetachPin(uint8_t pin) { fakeAttached[pin] = -1; }
struct FakeSerial {
  std::string input;
  std::string output;
  int capacity = 2048;
  void setTxBufferSize(size_t) {}
  void begin(uint32_t) {}
  int availableForWrite() { return capacity; }
  int available() { return static_cast<int>(input.size()); }
  int read() { if (input.empty()) return -1; int c = input[0]; input.erase(0, 1); return c; }
  size_t write(const uint8_t *data, size_t size) {
    output.append(reinterpret_cast<const char *>(data), size); return size;
  }
};
extern FakeSerial Serial;
