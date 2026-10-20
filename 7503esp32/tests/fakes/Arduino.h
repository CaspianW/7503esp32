/**
 * @file Arduino.h
 * @brief Minimal Arduino clock declaration for host-side controller tests.
 * @author Caspian Wu
 * @date 2026-10-19
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

#include <stdint.h>

constexpr int INPUT_PULLUP = 2;
inline void pinMode(int, int) {}

uint32_t millis();
