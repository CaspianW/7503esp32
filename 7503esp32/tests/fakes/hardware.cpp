/**
 * @file hardware.cpp
 * @brief Shared host-side clock, PCNT and PWM state.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "Arduino.h"
#include "ICM42670P.h"
#include <driver/pcnt.h>
uint32_t fakeNow = 0;
uint32_t fakePwm[8] = {};
int fakePin[49] = {};
int fakeAttached[49] = {};
int fakeSetupFailure = -1;
int16_t fakePcnt[4] = {};
int fakePcntFailure = 0;
FakeWire Wire;
FakeSerial Serial;

namespace {
struct ResetPins {
  ResetPins() { for (int &pin : fakeAttached) pin = -1; }
} resetPins;
}
