#include "wheel_pid.h"

namespace robot {

WheelPid::WheelPid()
    : gains_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, integral_(0.0f),
      previousError_(0.0f), hasPreviousError_(false) {}

} // robot 命名空间结束

// 在这里实现离散 PID 计算和输出限幅。
