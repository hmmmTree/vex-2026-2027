#include "robot/intake.hpp"

namespace robot {

Intake::Intake(Hardware& hw) : hw_(hw) {}
    void Intake::run(double speed) {
        hw_.top_roller.move_velocity(speed);
    }
    void Intake::run_top(double speed) {
        hw_.top_roller.move_velocity(speed);
    }
    void Intake::run_bottom(double speed) {
        hw_.bottom_roller.move_velocity(speed);
    }
    void Intake::stop() {
        hw_.top_roller.move_velocity(0);
        hw_.bottom_roller.move_velocity(0);
    }
    void Intake::lift() {
        hw_.intake_slider.extend();
    }
    void Intake::lower() {
        hw_.intake_slider.retract();
    }
    void Intake::toggle() {
        if (hw_.intake_slider.is_extended()) {
            hw_.intake_slider.retract();
        } else {
            hw_.intake_slider.extend();
        }
    }
}