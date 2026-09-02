#pragma once

#include "robot/diagnostics.hpp"
#include "robot/drive.hpp"
#include "robot/hardware.hpp"
#include "robot/interrupt.hpp"
#include "robot/odometry.hpp"
#include "robot/vision.hpp"

namespace robot {

class Robot {
public:
    static Robot& instance();

    Robot(const Robot&)            = delete;
    Robot& operator=(const Robot&) = delete;

    Hardware&          hardware()    { return hardware_; }
    Interrupts&        interrupts()  { return interrupts_; }
    Odometry&          odometry()    { return odometry_; }
    Drivetrain&        drivetrain()  { return drivetrain_; }
    AprilTagLocalizer& vision()      { return vision_; }
    Diagnostics&       diagnostics() { return diagnostics_; }

private:
    Robot();

    // Declaration order is construction order, and the constructors below take
    // references to each other do not reordr these.
    Hardware          hardware_;
    Interrupts        interrupts_;
    Odometry          odometry_;
    Drivetrain        drivetrain_;
    AprilTagLocalizer vision_;
    Diagnostics       diagnostics_;
};

}
