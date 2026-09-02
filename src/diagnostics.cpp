#include "robot/diagnostics.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace robot {

namespace {

double at(const std::vector<double>& values, std::size_t index) {
    return index < values.size() ? values[index] : 0.0;
}

double hottest(const std::vector<double>& temperatures) {
    double peak = 0.0;
    for (double temperature : temperatures) {
        if (std::isfinite(temperature) && temperature > peak) peak = temperature;
    }
    return peak;
}

}

Diagnostics::Diagnostics(Hardware& hardware) : hw_(hardware) {}

void Diagnostics::show_drive(int forward, int rotate, double scale) {
    const std::vector<double> left_velocity  = hw_.left.get_actual_velocity_all();
    const std::vector<double> right_velocity = hw_.right.get_actual_velocity_all();
    const std::vector<double> left_temp      = hw_.left.get_temperature_all();
    const std::vector<double> right_temp     = hw_.right.get_temperature_all();

    pros::lcd::print(3, "stk %4d/%4d spd %.2f", forward, rotate, scale);
    pros::lcd::print(4, "L %5.0f %5.0f %5.0f",
                     at(left_velocity, 0), at(left_velocity, 1), at(left_velocity, 2));
    pros::lcd::print(5, "R %5.0f %5.0f %5.0f",
                     at(right_velocity, 0), at(right_velocity, 1), at(right_velocity, 2));
    pros::lcd::print(6, "hot L%.0fC R%.0fC", hottest(left_temp), hottest(right_temp));
}

void Diagnostics::show_result(const char* label, MotionResult result,
                              const Interrupts& interrupts) {
    if (result == MotionResult::Interrupted) {
        pros::lcd::print(7, "%s: cut by %s", label, interrupts.reason().c_str());
    } else {
        pros::lcd::print(7, "%s: %s", label, to_string(result));
    }
}

}
