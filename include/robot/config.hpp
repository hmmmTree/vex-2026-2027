#pragma once

#include <cmath>

namespace robot {

inline constexpr double conversion_jerry2in = 36.0 / 35.0;

inline constexpr double rad2deg = 180.0 / M_PI;
inline constexpr double deg2rad = M_PI / 180.0;

inline constexpr double ticks_per_rev   = 36000.0;
inline constexpr double wheel_diameter  = 2.0;
inline constexpr double inches_per_tick = (wheel_diameter * M_PI) / ticks_per_rev;

inline constexpr double DRIVE_MAX_RPM = 200.0;

// Every control loop in this project ticks at 20 ms.
inline constexpr double LOOP_INTERVAL_S  = 0.02;
inline constexpr int    LOOP_INTERVAL_MS = 20;

// One PID tuning passed as a single value instead of three loose doubles
struct PidGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
};

// How the integral term is allowed to behave: it only winds up once the error
// is small and its contribution to the output is bounded
struct IntegralLimits {
    double zone = 0.0;  // accumulate only while |error| < zone (0 disables the term)
    double max  = 0.0;  // cap on |ki * integral|
};

// Tracking-wheel geometry how far each tracker sits from the tracking centre,
// used to subtract the arc a tracker sweeps when the robot rotates in place
struct OdometryConfig {
    double x_tracker_offset = 0.0;
    double y_tracker_offset = 0.0;
};

}
