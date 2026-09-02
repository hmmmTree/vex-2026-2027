#include "robot/robot.hpp"

namespace robot {

Robot& Robot::instance() {
    static Robot robot;
    return robot;
}

Robot::Robot()
    : odometry_(hardware_),
      drivetrain_(hardware_, odometry_, interrupts_),
      vision_(hardware_, odometry_),
      diagnostics_(hardware_) {}

}
