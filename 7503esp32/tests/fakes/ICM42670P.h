/**
 * @file ICM42670P.h
 * @brief Minimal host-test substitute for the IMU library header.
 * @author Caspian Wu
 * @date 2026-10-04
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

struct FakeWire {};
extern FakeWire Wire;

class ICM42670 {
public:
  ICM42670(FakeWire &, bool) {}
};
