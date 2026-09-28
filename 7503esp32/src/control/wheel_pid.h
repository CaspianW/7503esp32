#pragma once

namespace robot {

struct PidGains {
  float kp;
  float ki;
  float kd;
  float outputLimit;
  float integralLimit;
};

class WheelPid {
public:
  WheelPid();
  void configure(const PidGains &gains);
  float update(float targetMps, float measuredMps, float dtSeconds);
  void reset();

private:
  PidGains gains_;
  float integral_;
  float previousError_;
  bool hasPreviousError_;
};

} // robot 命名空间结束
