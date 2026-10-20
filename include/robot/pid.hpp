#pragma once

#include "robot/config.hpp"

#include <cstddef>
#include <limits>

namespace robot {

class Pid {
public:
    static constexpr std::size_t max_smoothing = 8;

    explicit Pid(PidGains gains, IntegralLimits integral_limits = {});

    void set_gains(PidGains gains) { gains_ = gains; }
    PidGains gains() const { return gains_; }

    void set_integral_limits(IntegralLimits limits) { limits_ = limits; }


    void set_derivative_smoothing(std::size_t samples);

    void set_slew(double max_step_per_update) { slew_ = max_step_per_update; }

    void set_output_limits(double min, double max);

    void reset();

    // `dt` is the time since the previous update, in seconds. Gains are in
    // per-tick units (one tick = LOOP_INTERVAL_S), so kd multiplies the error
    // change per loop and ki multiplies the running sum of error per loop.
    double update(double error, double dt = LOOP_INTERVAL_S);

    // Same, but the derivative comes from `error_change` (how much the error
    // moved since the last update) instead of differencing `error`. Pass the
    // change in the measurement, negated, and a jump in the target no longer
    // kicks the D term: it only reacts to the robot actually moving.
    double update_with_change(double error, double error_change, double dt = LOOP_INTERVAL_S);

    // Starts the slew from `output` instead of 0, for a motion that picks up
    // where a moving one left off. Call after set_derivative_smoothing(),
    // which resets the output.
    void prime(double output) { output_ = output; }

    double output() const { return output_; }
    double integral() const { return integral_; }
    double derivative() const { return derivative_; }

private:
    double step(double error, double error_change, double dt);

    PidGains       gains_;
    IntegralLimits limits_;

    double      history_[max_smoothing] = {};
    std::size_t smoothing_ = 1;

    double integral_   = 0.0;
    double derivative_ = 0.0;
    double prev_error_ = 0.0;
    bool   has_prev_   = false;

    double output_  = 0.0;
    double slew_    = 0.0;
    double out_min_ = -std::numeric_limits<double>::infinity();
    double out_max_ =  std::numeric_limits<double>::infinity();
};

}
