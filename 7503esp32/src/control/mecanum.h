#pragma once

#include "../common/robot_types.h"

namespace robot {

class MecanumKinematics {
public:
  MecanumKinematics();
  bool configure(float wheelRadiusM, float wheelbaseM, float trackWidthM);

  // 车轮顺序：M1 左前、M2 左后、M3 右前、M4 右后。
  bool inverse(const Twist2D &bodyVelocity, WheelValues &wheelLinearMps) const;
  bool forward(const WheelValues &wheelLinearMps, Twist2D &bodyVelocity) const;
  static void scaleToLimit(WheelValues &wheelValues, float maxAbsValue);

private:
  float wheelRadiusM_;
  float halfWheelbaseM_;
  float halfTrackWidthM_;
  bool configured_;
};

} // robot 命名空间结束
