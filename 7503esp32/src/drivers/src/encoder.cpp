/**
 * @file encoder.cpp
 * @brief Four-channel quadrature encoder driver.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../include/encoder.h"

#include "../../config/board_pins.h"
#include "../../config/robot_config.h"

#include <Arduino.h>
#include <driver/pcnt.h>

namespace robot {
namespace {

constexpr int16_t counterLimit = 30000;
constexpr uint16_t glitchFilter = 100; // 单位 APB 时钟周期

bool configureChannel(pcnt_unit_t unit, pcnt_channel_t channel,
                      int pulsePin, int directionPin, bool isChannelA) {
  pcnt_config_t settings{};
  settings.pulse_gpio_num = pulsePin;
  settings.ctrl_gpio_num = directionPin;
  settings.lctrl_mode = PCNT_MODE_KEEP;
  settings.hctrl_mode = PCNT_MODE_REVERSE;
  // 与 Yahboom 示例一致：车轮正向旋转时累计计数为正。
  settings.pos_mode = isChannelA ? PCNT_COUNT_DEC : PCNT_COUNT_INC;
  settings.neg_mode = isChannelA ? PCNT_COUNT_INC : PCNT_COUNT_DEC;
  settings.counter_h_lim = counterLimit;
  settings.counter_l_lim = -counterLimit;
  settings.unit = unit;
  settings.channel = channel;
  return pcnt_unit_config(&settings) == ESP_OK;
}

} // 匿名命名空间结束

bool Encoder::begin() {
  if (initialized_) {
    return true;
  }

  healthy_ = false;
  for (uint8_t i = 0; i < wheelCount; ++i) {
    if (config::encoderDirection[i] != 1 && config::encoderDirection[i] != -1)
      return false;
    const board::MotorPins &pins = board::motorPins[i];
    pinMode(pins.encoderA, INPUT_PULLUP);
    pinMode(pins.encoderB, INPUT_PULLUP);
    const pcnt_unit_t unit = static_cast<pcnt_unit_t>(i);
    if (!configureChannel(unit, PCNT_CHANNEL_0, pins.encoderA, pins.encoderB,
                          true) ||
        !configureChannel(unit, PCNT_CHANNEL_1, pins.encoderB, pins.encoderA,
                          false) ||
        pcnt_set_filter_value(unit, glitchFilter) != ESP_OK ||
        pcnt_filter_enable(unit) != ESP_OK ||
        pcnt_counter_pause(unit) != ESP_OK ||
        pcnt_counter_clear(unit) != ESP_OK ||
        pcnt_counter_resume(unit) != ESP_OK) {
      return false;
    }
    accumulatedTicks_[i] = 0;
  }
  healthy_ = true;
  initialized_ = true;
  return true;
}

int64_t Encoder::readTicks(WheelId wheel) {
  const uint8_t index = static_cast<uint8_t>(wheel);
  if (!initialized_ || index >= wheelCount) {
    return 0;
  }
  const pcnt_unit_t unit = static_cast<pcnt_unit_t>(index);
  int16_t delta = 0;
  // PCNT 计数器只有 16 位；每个控制周期读取并清零，软件累加为 64 位。
  // 调用间隔必须足够短，避免计数达到硬件上下限。
  if (!healthy_) return accumulatedTicks_[index];
  const bool paused = pcnt_counter_pause(unit) == ESP_OK;
  const bool read = paused && pcnt_get_counter_value(unit, &delta) == ESP_OK;
  const bool cleared = read && pcnt_counter_clear(unit) == ESP_OK;
  const bool resumed = pcnt_counter_resume(unit) == ESP_OK;
  if (!paused || !read || !cleared || !resumed ||
      delta >= counterLimit || delta <= -counterLimit) {
    healthy_ = false;
    return accumulatedTicks_[index];
  }
  const int64_t step = static_cast<int64_t>(delta) * config::encoderDirection[index];
  if ((step > 0 && accumulatedTicks_[index] > INT64_MAX - step) ||
      (step < 0 && accumulatedTicks_[index] < INT64_MIN - step)) {
    healthy_ = false;
    return accumulatedTicks_[index];
  }
  accumulatedTicks_[index] += step;
  return accumulatedTicks_[index];
}

bool Encoder::readAllTicks(int64_t ticks[wheelCount]) {
  if (ticks == nullptr) return false;
  for (uint8_t i = 0; i < wheelCount; ++i) {
    ticks[i] = readTicks(static_cast<WheelId>(i));
  }
  return healthy();
}

void Encoder::resetAll() {
  for (uint8_t i = 0; i < wheelCount; ++i) {
    accumulatedTicks_[i] = 0;
    if (initialized_) {
      const pcnt_unit_t unit = static_cast<pcnt_unit_t>(i);
      if (pcnt_counter_pause(unit) != ESP_OK ||
          pcnt_counter_clear(unit) != ESP_OK ||
          pcnt_counter_resume(unit) != ESP_OK) healthy_ = false;
    }
  }
}

} // robot 命名空间结束
