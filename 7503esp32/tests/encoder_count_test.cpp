/**
 * @file encoder_count_test.cpp
 * @brief Host tests for long-running encoder accumulation.
 * @author Caspian Wu
 * @date 2026-10-20
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../src/drivers/include/encoder.h"

#include <driver/pcnt.h>

#include <cassert>

int16_t fakePcnt[4] = {};

int main() {
  robot::Encoder encoder;
  assert(encoder.begin());

  // 每次读数低于 PCNT 上限 30000 count，累计跨越旧 int32 上限。
  for (int i = 0; i < 71585; ++i) {
    fakePcnt[0] = 29999;
    encoder.readTicks(robot::frontLeft);
  }
  fakePcnt[0] = 29999;
  assert(encoder.readTicks(robot::frontLeft) == 2147508414LL);
  fakePcnt[0] = -29999;
  assert(encoder.readTicks(robot::frontLeft) == 2147478415LL);

  int64_t ticks[robot::wheelCount] = {};
  encoder.readAllTicks(ticks);
  assert(ticks[0] == 2147478415LL);
  for (uint8_t i = 1; i < robot::wheelCount; ++i) assert(ticks[i] == 0);

  encoder.resetAll();
  assert(encoder.readTicks(robot::frontLeft) == 0);
}
