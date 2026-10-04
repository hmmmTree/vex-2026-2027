#pragma once

#include "robot/hardware.hpp"

namespace robot {

class Lift {
public:
    explicit Lift(Hardware& hw);
    
    void up(double speed);
    void down(double speed);
    void toggle(int stage);
    void drop();

    void armtoggle(); // 90 mech via pnumtics
    void clawtoggle();//180 mech via pnumatics
    void grippertoggle(); // gripper via pnumatics

    // Each call moves to the next step: up slowly -> stop -> down slowly -> stop -> repeat
    void claw_cycle();

private:
    Hardware& hw_;
    int       claw_step_ = 0;   // which step claw_cycle() runs next (0 to 3)
};
}