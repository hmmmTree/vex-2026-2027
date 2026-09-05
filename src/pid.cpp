#include "robot/pid.hpp"

#include <algorithm>
#include <cmath>

namespace robot {

Pid::Pid(PidGains gains, IntegralLimits integral_limits)
    : gains_(gains), limits_(integral_limits) {}

void Pid::set_derivative_smoothing(std::size_t samples) {
    if (samples < 1) samples = 1;
    if (samples > max_smoothing) samples = max_smoothing;
    smoothing_ = samples;
    reset();
}

void Pid::set_output_limits(double min, double max) {
    if (min > max) std::swap(min, max);
    out_min_ = min;
    out_max_ = max;
}

void Pid::reset() {
    for (double& sample : history_) sample = 0.0;
    integral_   = 0.0;
    derivative_ = 0.0;
    prev_error_ = 0.0;
    has_prev_   = false;
    output_     = 0.0;
}

double Pid::update(double error, double dt) {
    if (!(dt > 0.0)) dt = LOOP_INTERVAL_S;

    if (!has_prev_) {
        prev_error_ = error;
        has_prev_   = true;
    }

    set_derivative_smoothing(4);  // clamp smoothing in case it changed since last update

    for (std::size_t i = smoothing_; i-- > 1;) history_[i] = history_[i - 1];
    history_[0] = (error - prev_error_) / dt;

    derivative_ = 0.0;
    for (std::size_t i = 0; i < smoothing_; i++) derivative_ += history_[i];
    derivative_ /= static_cast<double>(smoothing_);

    if ((error > 0.0 && prev_error_ < 0.0) || (error < 0.0 && prev_error_ > 0.0)) {
        integral_ = 0.0;
    }

    if (limits_.zone > 0.0 && std::fabs(error) < limits_.zone) {
        integral_ += error * dt;
        if (gains_.ki != 0.0 && limits_.max > 0.0) {
            const double limit = std::fabs(limits_.max / gains_.ki);
            integral_ = std::clamp(integral_, -limit, limit);
        }
    } else {
        integral_ = 0.0;
    }

    prev_error_ = error;

    const double target = (error * gains_.kp)
                        + (derivative_ * gains_.kd)
                        + (integral_ * gains_.ki);

    if (slew_ > 0.0) output_ += std::clamp(target - output_, -slew_, slew_);
    else             output_ = target;

    output_ = std::clamp(output_, out_min_, out_max_);
    return output_;
}

}
