#include "robot/robot.hpp"

namespace robot {

Robot& Robot::instance() {
    static Robot robot;
    return robot;
}

Robot::Robot()
    : intake_(hardware_),
      odometry_(hardware_),
      lift_(hardware_, odometry_),
      drivetrain_(hardware_, odometry_, interrupts_),
      vision_(hardware_, odometry_),
      diagnostics_(hardware_) {}

}
