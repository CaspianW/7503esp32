/**
 * @file serial_console_test.cpp
 * @brief Serial framing, partial timeout, emergency stop and blocked-output tests.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include "../src/communication/serial_console.h"
#include "../src/app/robot_controller.h"
#include "../src/config/robot_config.h"
#include <driver/pcnt.h>
#include <cassert>
int main() {
  robot::RobotController c;
  robot::SerialConsole console;
  c.begin(); console.begin();
  assert(c.encoderReady() ==
         (robot::config::encoderReadEnabled || robot::config::chassisControlEnabled));
  Serial.input = "status\n"; console.update(c);
  assert(Serial.output.find("S ") != std::string::npos);
  assert(Serial.output.find("E 0.00000 0.00000 0.00000 0.00000 ") != std::string::npos);
  if (robot::config::encoderReadEnabled) {
    // 正前方向手转时原始M1～M4计数为 +、+、-、-；归一后都应为正。
    fakePcnt[0] = 5;
    fakePcnt[1] = 5;
    fakePcnt[2] = -5;
    fakePcnt[3] = -5;
    fakeNow += 10000;
    c.update();
    for (uint8_t i = 0; i < robot::wheelCount; ++i) assert(c.ticks()[i] == 5);
    assert(c.speedReady());
  }
  Serial.output.clear();
  Serial.input = "status\n"; console.update(c);
  if (robot::config::encoderReadEnabled)
    assert(Serial.output.find("E 0.00000 0.00000 0.00000 0.00000 0.09817 0.09817 0.09817 0.09817 ") != std::string::npos);
  else
    assert(Serial.output.find("E 0.00000 0.00000 0.00000 0.00000 ") != std::string::npos);
  assert(Serial.output.find("S ") != std::string::npos);
  assert(Serial.output.find("E ") != std::string::npos);
  assert(Serial.output.find("OK\n") != std::string::npos);
  if (robot::config::encoderReadEnabled) {
    fakeNow += 10000;
    c.update();
    Serial.output.clear(); Serial.input = "status\n"; console.update(c);
    assert(Serial.output.find("E 0.00000 0.00000 0.00000 0.00000 0.00000 0.00000 0.00000 0.00000 ") != std::string::npos);
  }
  if (c.benchMode()) {
    fakeNow += 10000; c.update();
    // 模拟外设回读与目标不符，P行必须暴露差异，不能复制E行目标值。
    fakePwm[2] = 123;
    Serial.output.clear(); Serial.input = "status\n"; console.update(c);
    assert(Serial.output.find("P 3 10 123 25000 -1 -1 -1 -1\n") != std::string::npos);
  }
  Serial.output.clear(); Serial.input = "manual arm\n"; console.update(c);
  assert(Serial.output.find("OK\n") != std::string::npos);
  assert(c.armed() && !c.motionAllowed());
  assert(c.encoderReady() ==
         (robot::config::encoderReadEnabled || robot::config::chassisControlEnabled));
  assert(c.arm());
  assert(!c.manualMode());
  Serial.input = "manual arm\n"; console.update(c);
  assert(c.manualMode());
  Serial.output.clear(); Serial.input = "manual duty 0.55\n"; console.update(c);
  assert(Serial.output.find("OK\n") != std::string::npos);
  Serial.output.clear(); Serial.input = "manual vel 0.05 0 0\n"; console.update(c);
  assert(Serial.output.find("OK\n") != std::string::npos);
  fakeNow += 10000; c.update();
  for (uint8_t channel = 0; channel < robot::wheelCount; ++channel)
    assert(fakePwm[channel] == 562); // 55%输出，4个逻辑轮映射到M1～M4
  Serial.input = "manual vel 0 0 0\n"; console.update(c);
  assert(c.armed() && c.motionAllowed());
  for (uint8_t channel = 0; channel < robot::wheelCount; ++channel) assert(fakePwm[channel] == 0);
  Serial.input = "manual arm\n"; console.update(c);
  assert(c.armed());
  Serial.input = "manual vel 0 0.05 0\n"; console.update(c);
  fakeNow += 10000; c.update();
  // 左移：FL、RR反转，RL、FR正转；motorDirection补偿电机安装方向。
  assert(fakeAttached[5] == 0);  // M1/RR，补偿后接PWM B
  assert(fakeAttached[15] == 1); // M2/FR，补偿后接PWM A
  assert(fakeAttached[10] == 2); // M3/RL，正向接PWM B
  assert(fakeAttached[13] == 3); // M4/FL，反向接PWM A
  Serial.input = "manual stop\n"; console.update(c);
  assert(!c.armed());
  for (uint8_t channel = 0; channel < robot::wheelCount; ++channel) assert(fakePwm[channel] == 0);
  Serial.input = "manual arm\n"; console.update(c);
  Serial.input = "manual vel 0.05 0 0\n"; console.update(c);
  fakeNow += 10000; c.update();
  for (int i = 0; i < 50; ++i) { fakeNow += 10000; c.update(); }
  assert(!c.armed()); // 500ms没有刷新指令即解除使能
  for (uint8_t channel = 0; channel < robot::wheelCount; ++channel) assert(fakePwm[channel] == 0);
  Serial.output.clear(); Serial.input = "estop 1\n"; console.update(c);
  assert(c.emergencyStopped());
  Serial.input = "estop 0\n"; console.update(c);
  assert(!c.emergencyStopped());
  assert(!c.armed()); // 解急停不会自动恢复使能

  // 串口触发急停时正在运动，必须立即清零四路PWM并回复OK。
  Serial.input = "manual arm\n"; console.update(c);
  Serial.input = "manual vel 0.05 0 0\n"; console.update(c);
  fakeNow += 10000;
  c.update();
  assert(c.motionAllowed());
  for (uint8_t channel = 0; channel < robot::wheelCount; ++channel)
    assert(fakePwm[channel] != 0);
  Serial.input = "lift up\n"; console.update(c);
  assert(c.liftActive() && fakePwm[4] != 0);
  Serial.output.clear();
  Serial.input = "estop 1\n"; console.update(c);
  assert(c.emergencyStopped() && !c.armed() && !c.motionAllowed());
  assert(Serial.output.find("OK\n") != std::string::npos);
  for (uint8_t channel = 0; channel < robot::wheelCount; ++channel)
    assert(fakePwm[channel] == 0);
  assert(c.liftActive() && fakePwm[4] != 0); // 急停不关闭升降PWM
  Serial.input = "lift down\n"; console.update(c);
  assert(c.liftPulse() == robot::config::liftLoweredPulse);
  assert(c.liftActive() && fakePwm[4] != 0); // 急停时仍可控制升降
  Serial.input = "estop 0\n"; console.update(c);
  assert(!c.emergencyStopped() && !c.armed());
  assert(c.liftActive() && fakePwm[4] != 0);

  // 桥接断开或显式 disarm 只撤销底盘，不能释放升降保持力。
  assert(c.arm());
  assert(c.commandVelocity({0.05f, 0, 0}));
  fakeNow += 10000; c.update();
  const uint32_t holdingPwm = fakePwm[4];
  Serial.output.clear(); Serial.input = "disarm\n"; console.update(c);
  assert(Serial.output.find("OK\n") != std::string::npos);
  assert(!c.armed() && !c.motionAllowed());
  assert(!c.commandVelocity({0.05f, 0, 0}));
  for (uint8_t channel = 0; channel < robot::wheelCount; ++channel)
    assert(fakePwm[channel] == 0);
  assert(c.liftActive() && fakePwm[4] == holdingPwm && holdingPwm != 0);

  Serial.input = std::string(130, 'a') + "\narm\n";
  while (Serial.available()) console.update(c);
  assert(Serial.output.find("ERR frame") != std::string::npos);
  Serial.output.clear(); Serial.input = "estop "; console.update(c);
  fakeNow += 101000; Serial.input = "1\n"; console.update(c);
  assert(!c.emergencyStopped());
  assert(Serial.output.find("ERR frame") != std::string::npos);
  Serial.capacity = 0; Serial.input = "status\n"; console.update(c); // 输出堵塞不阻塞控制
}
