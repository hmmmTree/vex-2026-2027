#pragma once

#include "robot/hardware.hpp"

namespace robot {

class Intake {
public:
    explicit Intake(Hardware& hw);

    void run(double speed);
    void run_top(double speed);
    void run_bottom(double speed);
    void stop();

    void lift();
    void lower();
    void toggle();

private:
    Hardware& hw_;
};

}