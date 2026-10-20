/**
 * @file robot_controller_safety_test.cpp
 * @brief Host tests for safe controller startup and motor shutdown.
 * @author Caspian Wu
 * @date 2026-10-19
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "fakes/Arduino.h"
#include "../src/app/robot_controller.h"

#include <cassert>

FakeWire Wire;

namespace {
uint32_t now = 0; // 单位 ms
float duty[robot::wheelCount] = {0.3f, 0.3f, 0.3f, 0.3f};
} // 匿名命名空间结束

uint32_t millis() { return now; }

namespace robot {

// 仅替换硬件输出边界；测试观察的是四路电机最终占空比。
bool MotorDriver::begin() { return true; }

void MotorDriver::setDuty(WheelId wheel, float value) {
  duty[static_cast<uint8_t>(wheel)] = value;
}

void MotorDriver::stopAll(bool) {
  for (float &value : duty) value = 0.0f;
}

} // robot 命名空间结束

int main() {
  robot::RobotController controller;
  assert(controller.begin());
  for (float value : duty) assert(value == 0.0f);

  // 无指令时，正常循环会清除任何遗留的电机输出。
  for (float &value : duty) value = 0.3f;
  controller.update();
  for (float value : duty) assert(value == 0.0f);

  // 收到指令后，超时前仅更新许可；当前正常模式始终保持停机。
  controller.noteCmdVel(100);
  now = 599;
  for (float &value : duty) value = 0.3f;
  controller.update();
  assert(controller.motionAllowed());
  for (float value : duty) assert(value == 0.0f);
  now = 600;
  controller.update();
  assert(!controller.motionAllowed());
  for (float value : duty) assert(value == 0.0f);

  // 超时后收到新指令可恢复运动许可，但当前版本仍不产生电机输出。
  controller.noteCmdVel(700);
  now = 700;
  controller.update();
  assert(controller.motionAllowed());
  for (float value : duty) assert(value == 0.0f);

  // 急停调用当下就清除电机输出，不能等下一个 update()。
  for (float &value : duty) value = 0.3f;
  controller.setEmergencyStop(true);
  assert(!controller.motionAllowed());
  for (float value : duty) assert(value == 0.0f);
  // 同一控制器实例再次调用 begin()，不能清掉急停并重启台架流程。
  assert(!controller.begin());

  // 急停期间的速度指令无效；解除后必须重新收到指令。
  controller.noteCmdVel(710);
  controller.setEmergencyStop(false);
  assert(!controller.begin());
  now = 711;
  controller.update();
  assert(!controller.motionAllowed());
  controller.noteCmdVel(720);
  now = 720;
  controller.update();
  assert(controller.motionAllowed());

  // 在 begin() 之前触发的软件急停也不能被初始化清除。
  robot::RobotController stoppedBeforeBegin;
  stoppedBeforeBegin.setEmergencyStop(true);
  assert(!stoppedBeforeBegin.begin());
}
