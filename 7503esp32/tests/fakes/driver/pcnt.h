/**
 * @file pcnt.h
 * @brief Minimal PCNT boundary for host-side encoder tests.
 * @author Caspian Wu
 * @date 2026-10-20
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include <stdint.h>

using pcnt_unit_t = int;
using pcnt_channel_t = int;

constexpr int ESP_OK = 0;
constexpr int PCNT_MODE_KEEP = 0;
constexpr int PCNT_MODE_REVERSE = 1;
constexpr int PCNT_COUNT_DEC = -1;
constexpr int PCNT_COUNT_INC = 1;
constexpr int PCNT_CHANNEL_0 = 0;
constexpr int PCNT_CHANNEL_1 = 1;

struct pcnt_config_t {
  int pulse_gpio_num;
  int ctrl_gpio_num;
  int lctrl_mode;
  int hctrl_mode;
  int pos_mode;
  int neg_mode;
  int counter_h_lim;
  int counter_l_lim;
  pcnt_unit_t unit;
  pcnt_channel_t channel;
};

extern int16_t fakePcnt[4];

inline int pcnt_unit_config(const pcnt_config_t *) { return ESP_OK; }
inline int pcnt_set_filter_value(pcnt_unit_t, uint16_t) { return ESP_OK; }
inline int pcnt_filter_enable(pcnt_unit_t) { return ESP_OK; }
inline int pcnt_counter_pause(pcnt_unit_t) { return ESP_OK; }
inline int pcnt_counter_resume(pcnt_unit_t) { return ESP_OK; }
inline int pcnt_counter_clear(pcnt_unit_t unit) {
  fakePcnt[unit] = 0;
  return ESP_OK;
}
inline int pcnt_get_counter_value(pcnt_unit_t unit, int16_t *value) {
  *value = fakePcnt[unit];
  return ESP_OK;
}
