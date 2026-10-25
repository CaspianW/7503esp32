/**
 * @file pid.h
 * @brief Generic PID controller interface.
 * @author Caspian Wu
 * @date 2026-09-28
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#pragma once

namespace robot {

struct PidParam {
  float kp;
  float ki;
  float kd;
  float outMax; // 输出绝对值上限
  float iMax;   // 积分累加状态绝对值上限
  bool dMeas; // true 时对测量值微分，避免设定值突变引起微分冲击

  // 构造函数兼容 Arduino-ESP32 常用的 C++11 编译标准。
  PidParam(float pGain = 0.0f, float iGain = 0.0f,
           float dGain = 0.0f, float maxOut = 0.0f,
           float maxI = 0.0f, bool onMeas = false)
      : kp(pGain), ki(iGain), kd(dGain), outMax(maxOut), iMax(maxI),
        dMeas(onMeas) {}
};

// 不依赖具体电机或传感器的通用 PID 计算器。
class Pid {
public:
  Pid();
  // 配置增益及限幅；重新配置会清除积分和微分历史。
  void configure(const PidParam &param);
  // dt 为采样周期，单位为秒；首次有效更新时微分项为零。
  float update(float target, float measured, float dt);
  void reset();

private:
  PidParam param_;
  float iSum_;
  float prevErr_;
  float prevMeas_;
  bool hasPrev_;
};

} // robot 命名空间结束
