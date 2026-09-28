#pragma once

#include "robot_types.h"

namespace robot {

class MecanumKinematics {
public:
  MecanumKinematics();
  bool configure(float wheelRadiusM, float wheelbaseM, float trackWidthM);

  // Wheel order is M1 front-left, M2 rear-left, M3 front-right, M4 rear-right.
  bool inverse(const Twist2D &bodyVelocity, WheelValues &wheelLinearMps) const;
  bool forward(const WheelValues &wheelLinearMps, Twist2D &bodyVelocity) const;
  static void scaleToLimit(WheelValues &wheelValues, float maxAbsValue);

private:
  float wheelRadiusM_;
  float halfWheelbaseM_;
  float halfTrackWidthM_;
  bool configured_;
};

} // namespace robot
