#include "mecanum.h"

namespace robot {

MecanumKinematics::MecanumKinematics()
    : wheelRadiusM_(0.0f), halfWheelbaseM_(0.0f), halfTrackWidthM_(0.0f),
      configured_(false) {}

} // namespace robot

// Mecanum inverse/forward kinematics to be implemented after confirming wheel
// roller orientation and motor direction on the assembled chassis.
