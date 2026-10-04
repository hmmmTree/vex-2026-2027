#include "robot/lift.hpp"

namespace robot {

namespace {
constexpr int CLAW_SLOW_RPM = 75;   // very slow; raise it if the claw is too weak to move
}

Lift::Lift(Hardware& hw) : hw_(hw) {}
    void Lift::up(double speed) {
        hw_.lift.move_velocity(speed);
    }
    void Lift::down(double speed) {
        hw_.lift.move_velocity(-speed);
    }
    void Lift::toggle(int stage){
        //logic i rly dont wanna deal with it rn
    }
    void Lift::drop() {
        //logic i rly dont wanna deal with it rn
    }

    void Lift::armtoggle() {
        if (hw_.claw_flip.is_extended() == true) {
            hw_.claw.retract();
        } else {
            hw_.claw_flip.extend();
        }
    }
    void Lift::clawtoggle() {
        if (hw_.flipper_claw.is_extended() == true) {
            hw_.flipper_claw.retract();
        } else {
            hw_.flipper_claw.extend();
        }
    } 
    void Lift::grippertoggle() {
        if (hw_.claw.is_extended() == true) {
            hw_.claw.retract();
        } else {
            hw_.claw.extend();
        }
    }
}



