// Timed completion is nonblocking, idempotent and safe across uint32 millis rollover.
#include "fakes/Arduino.h"
#include "../../src/control/include/lift_controller.h"
#include "../../src/config/board_pins.h"
#include "../../src/config/robot_config.h"
#include <cassert>

int main() {
  robot::ServoDriver servo;
  robot::LiftController lift;
  const auto begin = [&]() {
    assert(servo.begin(robot::board::servoS2, robot::config::servoPwmFrequency));
    assert(lift.begin(servo, robot::config::liftLoweredPulse, robot::config::liftRaisedPulse));
  };
  begin();
  assert(!lift.completionKnown());
  lift.request(robot::LiftPosition::lowered, 0);
  assert(fakePwm[4] != 0);
  lift.update(2999);
  assert(!lift.completionKnown());
  lift.update(3000);
  assert(lift.completionKnown() && lift.completedState() == 0);

  lift.request(robot::LiftPosition::raised, 3100);
  const uint32_t raisedDuty = fakePwm[4];
  assert(raisedDuty != 0 && lift.completedState() == 0);
  lift.request(robot::LiftPosition::raised, 5000); // 重发不得重新等待3s。
  lift.update(6099);
  assert(lift.completedState() == 0);
  lift.update(6100);
  assert(lift.completedState() == 1 && fakePwm[4] == raisedDuty);

  lift.request(robot::LiftPosition::lowered, 7000);
  lift.request(robot::LiftPosition::raised, 8000); // 中途反向取消下降的完成事件。
  lift.update(10000);
  assert(lift.completedState() == 1);
  lift.update(11000);
  assert(lift.completedState() == 1);

  lift.request(robot::LiftPosition::lowered, 12000);
  lift.stop();
  lift.update(20000);
  assert(lift.completedState() == 1 && fakePwm[4] == 0);
  begin();
  lift.request(robot::LiftPosition::lowered, 21000);
  lift.update(23999);
  assert(lift.completedState() == 1);
  lift.update(24000);
  assert(lift.completedState() == 0);

  lift.request(robot::LiftPosition::raised, UINT32_MAX - 1000);
  lift.update(1998);
  assert(lift.completedState() == 0);
  lift.update(1999);
  assert(lift.completedState() == 1);
}
