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

    // `dt` is the time since the preious update, in seconds.
    double update(double error, double dt = LOOP_INTERVAL_S);

    double output() const { return output_; }
    double integral() const { return integral_; }
    double derivative() const { return derivative_; }

private:
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
