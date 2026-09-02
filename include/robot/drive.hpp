#pragma once

#include "api.h"
#include "robot/config.hpp"
#include "robot/hardware.hpp"
#include "robot/interrupt.hpp"
#include "robot/odometry.hpp"

namespace robot {

// A "drive to this point" request. Grouping the twelve loose parameters the old
// cordon() took into one value keeps call sites readable and lets a caller name
// only the fields it actually cares about.
struct CordonRequest {
    double x = 0.0;
    double y = 0.0;

    PidGains drive_gains{};
    PidGains turn_gains{};

    // Gains for the turn_at_end sweep. Deliberately separate from turn_gains:
    // steering while rolling and pivoting on the spot are different problems,
    // and the old code shared one kd between them even though the two loops
    // differentiated on different time bases.
    PidGains final_turn_gains{};

    double drive_timeout = 3.0;  // seconds
    double turn_timeout  = 2.0;

    // How close counts as arrived. Keep it generous for a waypoint the robot
    // should drive through, tight for the last point of a path.
    double exit_radius = 2.0;

    // Leave false for a pass-through waypoint so the robot keeps its momentum.
    bool stop_at_end = true;

    // Face this heading once the point is reached. The old code read
    // "theta != 0" as "turn", which made 0 degrees impossible to request.
    //
    // With lead > 0 this is also the heading the robot arrives *at*, so a
    // trailing turn is usually no longer needed.
    bool   turn_at_end   = false;
    double final_heading = 0.0;

    // --- boomerang --------------------------------------------------------

    // How far the carrot point leads the robot as a fraction of the distance
    // still to go 0 disables boomerang and the motion is the plain
    // drive-to-a-point it has always been anything above 0 turns it into a
    // drive-to-a-pose that arrives along final_heading 0.3 to 0.6 is the
    // usual working range and values at or above 1.0 put the carrot behind
    // the robot so it is clamped
    double lead = 0.0;

    // Arrive at final_heading backwards. The robot still finishes facing
    // final_heading it just gets there with its back to the approach.
    bool reverse = false;
};

// The drive base driver control and the autonomous motions built on top of it
// Every autonomous motion polls the shared Interrupts once per control tick and
// returns MotionResult::Interrupted motors stopped, if anything fired. None of
// them can loop forever: each has a timeout and a bound on consecutive sensor
// failures
class Drivetrain {
public:
    Drivetrain(Hardware& hardware, Odometry& odometry, Interrupts& interrupts);

    Drivetrain(const Drivetrain&)            = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;

    // --- driver control ---------------------------------------------------

    // `forward` and `rotate` are raw joystick counts (-127..127)
    void arcade(int forward, int rotate, double scale = 1.0);
    void tank(int left, int right, double scale = 1.0);

    void stop();
    void set_brake_mode(pros::motor_brake_mode_e mode);

    // --- autonomous motions -----------------------------------------------

    [[nodiscard]] MotionResult turn_to(double heading_deg, PidGains gains, double timeout_s);

    // `heading_gains` holds the robot's starting heading while it drives pass
    // an all-zero value to drive without heading correction
    [[nodiscard]] MotionResult drive_distance(double inches, PidGains gains,
                                              PidGains heading_gains, double timeout_s);

    [[nodiscard]] MotionResult cordon(const CordonRequest& request);

    // Percent of DRIVE_MAX_RPM to motor RPM This was vperc()
    static double velocity_percent_to_rpm(double percent);

    // Most recent turn PID sample, for on-screen tuning.
    double last_turn_error() const { return last_turn_error_; }
    double last_turn_power() const { return last_turn_power_; }

private:
    // Commands each side in percent of maximum velocity
    void set_wheel_percent(double left_percent, double right_percent);

    Hardware&   hw_;
    Odometry&   odometry_;
    Interrupts& interrupts_;

    double last_turn_error_ = 0.0;
    double last_turn_power_ = 0.0;
};

}
