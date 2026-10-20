#pragma once

#include "robot/hardware.hpp"
#include "robot/odometry.hpp"
#include "robot/interrupt.hpp"

namespace robot {


class Lift {
public:
    Lift(Hardware& hw, Odometry& odometry);
    
    void up(double speed);
    void down(double speed);
    void toggle(LiftGains gains, double timeout);
    void drop(LiftGains gains, double timeout);

    void armtoggle(); // 90 mech via pnumtics
    void clawtoggle();//180 mech via pnumatics
    void grippertoggle(); // gripper via pnumatics
   

    [[nodiscard]] MotionResult move_to(double inches, LiftGains gains, double timeout_s);

private:
    Hardware& hw_;
    Odometry& odometry_;
};
}