#pragma once

#include "../common/robot_types.h"

namespace robot {

class RosBridge {
public:
  bool begin();
  void spinOnce();
  bool takeVelocityCommand(Twist2D &command);
  void publishOdometry(const OdometryState &odometry);
  void publishImu(const ImuSample &sample);
  bool connected() const;
};

} // robot 命名空间结束
