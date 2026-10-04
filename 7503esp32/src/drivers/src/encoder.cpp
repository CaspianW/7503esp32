/**
 * @file encoder.cpp
 * @brief Four-channel quadrature encoder driver.
 * @author Caspian Wu
 * @date 2026-09-28
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

  for (uint8_t i = 0; i < wheelCount; ++i) {
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
  initialized_ = true;
  return true;
}

int32_t Encoder::readTicks(WheelId wheel) {
  const uint8_t index = static_cast<uint8_t>(wheel);
  if (!initialized_ || index >= wheelCount) {
    return 0;
  }
  const pcnt_unit_t unit = static_cast<pcnt_unit_t>(index);
  int16_t delta = 0;
  // PCNT 计数器只有 16 位；每个控制周期读取并清零，软件累加为 32 位。
  // 调用间隔必须足够短，避免计数达到硬件上下限。
  if (pcnt_counter_pause(unit) == ESP_OK) {
    if (pcnt_get_counter_value(unit, &delta) == ESP_OK &&
        pcnt_counter_clear(unit) == ESP_OK) {
      accumulatedTicks_[index] +=
          static_cast<int32_t>(delta) * config::encoderDirection[index];
    }
    pcnt_counter_resume(unit);
  }
  return accumulatedTicks_[index];
}

void Encoder::readAllTicks(int32_t ticks[wheelCount]) {
  if (ticks == nullptr) {
    return;
  }
  for (uint8_t i = 0; i < wheelCount; ++i) {
    ticks[i] = readTicks(static_cast<WheelId>(i));
  }
}

void Encoder::resetAll() {
  for (uint8_t i = 0; i < wheelCount; ++i) {
    accumulatedTicks_[i] = 0;
    if (initialized_) {
      const pcnt_unit_t unit = static_cast<pcnt_unit_t>(i);
      pcnt_counter_pause(unit);
      pcnt_counter_clear(unit);
      pcnt_counter_resume(unit);
    }
  }
}

} // robot 命名空间结束
