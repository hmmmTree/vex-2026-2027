#pragma once

#include "robot/hardware.hpp"
#include "robot/interrupt.hpp"

namespace robot {

// Everything that writes to the brain screen in one place so the line numbers
// each subsystem uses stay visible next to each other
class Diagnostics {
public:
    explicit Diagnostics(Hardware& hardware);

    // Sticks per-motor velocity and the hottest motor on each side
    void show_drive(int forward, int rotate, double scale);

    // How a motion ended and what tripped if something did
    void show_result(const char* label, MotionResult result, const Interrupts& interrupts);

private:
    Hardware& hw_;
};

}
