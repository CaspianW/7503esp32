/**
 * @file pid_test.cpp
 * @brief PID比例、积分、微分、限幅和抗积分饱和测试。
 * @author Caspian Wu
 * @date 2026-10-25
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "../../src/control/include/pid.h"

#include <cassert>
#include <cmath>

namespace {
bool near(float actual, float expected) {
  return std::fabs(actual - expected) < 0.00001f;
}
}

int main() {
  robot::Pid p;
  p.configure(robot::PidParam(2.0f, 0.0f, 0.0f, 1.0f, 0.0f));
  assert(near(p.update(0.2f, 0.0f, 0.01f), 0.4f));
  assert(near(p.update(-0.8f, 0.0f, 0.01f), -1.0f));

  robot::Pid i;
  i.configure(robot::PidParam(0.0f, 1.0f, 0.0f, 1.0f, 0.2f));
  assert(near(i.update(1.0f, 0.0f, 0.1f), 0.1f));
  assert(near(i.update(1.0f, 0.0f, 0.1f), 0.2f));
  assert(near(i.update(1.0f, 0.0f, 0.1f), 0.2f));

  robot::Pid d;
  d.configure(robot::PidParam(0.0f, 0.0f, 0.5f, 10.0f, 0.0f, true));
  assert(near(d.update(1.0f, 0.0f, 0.1f), 0.0f));
  assert(near(d.update(1.0f, 0.2f, 0.1f), -1.0f));

  robot::Pid antiWindup;
  antiWindup.configure(robot::PidParam(1.0f, 1.0f, 0.0f, 0.5f, 10.0f));
  assert(near(antiWindup.update(1.0f, 0.0f, 1.0f), 0.5f));
  assert(near(antiWindup.update(0.0f, 0.0f, 1.0f), 0.0f));

  robot::Pid invalid;
  invalid.configure(robot::PidParam(1.0f, 0.0f, 1.0f, 1.0f, 0.0f));
  assert(near(invalid.update(1.0f, 0.0f, 0.01f), 1.0f));
  assert(near(invalid.update(1.0f, 0.0f, 0.0f), 0.0f));
  assert(near(invalid.update(1.0f, 0.0f, 0.01f), 1.0f));

  // 含前馈的上饱和抗积分：总输出超过 outMax 时停止正向积分累积。
  robot::Pid ffUpper;
  ffUpper.configure(robot::PidParam(1.0f, 1.0f, 0.0f, 0.8f, 10.0f));
  assert(near(ffUpper.update(1.0f, 0.0f, 1.0f, 0.6f, 0.0f), 0.8f));
  assert(near(ffUpper.update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f), 0.0f));

  // 含 minDuty 的下饱和抗积分：总输出触及正向下限时停止负向积分累积。
  robot::Pid ffLower;
  ffLower.configure(robot::PidParam(1.0f, 1.0f, 0.0f, 0.8f, 10.0f));
  assert(near(ffLower.update(0.4f, 0.8f, 1.0f, 0.6f, 0.5f), 0.5f));
  assert(near(ffLower.update(0.4f, 0.3f, 0.1f, 0.6f, 0.5f), 0.71f));

  // minDuty=0 时保持与目标同向（不反向制动），超速被截断到 0 且不产生负向积分饱和。
  robot::Pid sameSignPos;
  sameSignPos.configure(robot::PidParam(1.0f, 1.0f, 0.0f, 1.0f, 1.0f));
  assert(near(sameSignPos.update(0.4f, 0.8f, 0.1f, 0.0f, 0.0f), 0.0f));
  assert(near(sameSignPos.update(0.4f, 0.3f, 0.1f, 0.0f, 0.0f), 0.11f));

  robot::Pid sameSignNeg;
  sameSignNeg.configure(robot::PidParam(1.0f, 1.0f, 0.0f, 1.0f, 1.0f));
  assert(near(sameSignNeg.update(-0.4f, -0.8f, 0.1f, 0.0f, 0.0f), 0.0f));
  assert(near(sameSignNeg.update(-0.4f, -0.3f, 0.1f, 0.0f, 0.0f), -0.11f));
}
