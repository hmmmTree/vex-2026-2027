#include "robot/hardware.hpp"

namespace robot {

Hardware::Hardware()
    : master(pros::E_CONTROLLER_MASTER),
      // Port numbers and reversal flags are documented in hardware.hpp.
      left({LEFT_FRONT_PORT, LEFT_MID_PORT, LEFT_BACK_PORT}),
      right({RIGHT_FRONT_PORT, RIGHT_MID_PORT, RIGHT_BACK_PORT}),
      inertial(IMU_PORT),
      xrot(XROT_PORT),
      yrot(YROT_PORT),
      aivision(AIVISION_PORT) {}

void Hardware::configure() {
    left.move_velocity(0);
    right.move_velocity(0);
    left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    xrot.reset();
    yrot.reset();
    yrot.set_reversed(true);
}

}
