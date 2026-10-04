#include "robot/hardware.hpp"

namespace robot {

Hardware::Hardware()
    : master(pros::E_CONTROLLER_MASTER),
      left(std::vector<std::int8_t>(LEFT_PORTS.begin(),  LEFT_PORTS.end())),
      right(std::vector<std::int8_t>(RIGHT_PORTS.begin(), RIGHT_PORTS.end())),
      inertial(IMU_PORT),
      xrot(XROT_PORT),
      yrot(YROT_PORT),
      lift_r(LIFT_ROT_PORT),
      aivision(AIVISION_PORT),
      top_roller(TOP_ROLLER),
      bottom_roller(BOTTOM_ROLLER),
      lift(LIFT_PORT),
      claw_flip(CLAW_FLIP, false, false),
      flipper_claw(CLAW, false, false),
      intake_slider(INTAKE_SLIDER, false, false),
      claw(CLAW, false, false) {}


void Hardware::configure() {
    left.move_velocity(0);
    right.move_velocity(0);
    left.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    xrot.reset();
    yrot.reset();
    lift_r.reset();
    yrot.set_reversed(false);

    top_roller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    bottom_roller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    claw_flip.retract();
    claw.retract();
    intake_slider.retract();
    flipper_claw.retract();
}

}
