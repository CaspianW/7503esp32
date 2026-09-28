#include "mecanum.h"

namespace robot {

MecanumKinematics::MecanumKinematics()
    : wheelRadiusM_(0.0f), halfWheelbaseM_(0.0f), halfTrackWidthM_(0.0f),
      configured_(false) {}

} // robot 命名空间结束

// 确认实车的滚轮朝向和电机转向后，再实现麦克纳姆轮正、逆运动学解算。
