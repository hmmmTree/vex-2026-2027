#include "robot/hardware.hpp"

namespace robot {

Hardware::Hardware()
    : master(pros::E_CONTROLLER_MASTER),
      left({1, 6, 9}),
      right({-18, -19, -20}),
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
