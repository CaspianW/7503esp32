/**
 * @file motor_bench_test.cpp
 * @brief Host tests for the one-shot motor bench schedule.
 * @author Caspian Wu
 * @date 2026-10-04
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../src/app/robot_controller.h"

#include <cassert>

int main() {
  using robot::MotorBenchPhase;
  using robot::motorBenchPhase;

  // 启动等待、运行、停机各阶段在边界时刻正确切换。
  assert(motorBenchPhase(100, 100, 2000, 500) == MotorBenchPhase::wait);
  assert(motorBenchPhase(100, 2099, 2000, 500) == MotorBenchPhase::wait);
  assert(motorBenchPhase(100, 2100, 2000, 500) == MotorBenchPhase::run);
  assert(motorBenchPhase(100, 2599, 2000, 500) == MotorBenchPhase::run);
  assert(motorBenchPhase(100, 2600, 2000, 500) == MotorBenchPhase::done);
  assert(motorBenchPhase(100, 5000, 2000, 500) == MotorBenchPhase::done);

  // millis() 溢出后仍按无符号时间差计算。
  assert(motorBenchPhase(0xFFFFFF00U, 0x00000010U, 200, 500) ==
         MotorBenchPhase::run);
}
