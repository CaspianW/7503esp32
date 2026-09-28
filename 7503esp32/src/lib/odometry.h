#pragma once

#include "robot_types.h"

namespace robot {

class Odometry {
public:
  void reset(float xM = 0.0f, float yM = 0.0f, float yawRad = 0.0f);
  void update(const Twist2D &bodyVelocity, float imuYawRateRadPerSec,
              float dtSeconds, uint32_t timestampMs);
  const OdometryState &state() const;

private:
  OdometryState state_;
};

} // namespace robot
