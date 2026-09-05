#include "robot/drive.hpp"
#include "robot/pid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robot {

namespace {

constexpr IntegralLimits TURN_INTEGRAL         {10.0, 30.0};
constexpr IntegralLimits DRIVE_INTEGRAL        { 5.0, 25.0};
constexpr IntegralLimits CORDON_DRIVE_INTEGRAL { 8.0, 20.0};
constexpr IntegralLimits CORDON_TURN_INTEGRAL  {15.0, 20.0};

constexpr double TURN_SETTLE_DEG = 0.5;
constexpr int    TURN_SETTLE_MS  = 100;

constexpr double DRIVE_SETTLE_IN = 1.0;
constexpr int    DRIVE_SETTLE_MS = 100;

constexpr double MIN_HEADING_SCALE = 0.15;

constexpr double MAX_LEAD = 0.9;

constexpr double CARROT_EPSILON_IN = 0.1;

constexpr int MAX_SENSOR_FAULTS     = 50;
constexpr int SENSOR_RETRY_DELAY_MS = 10;

bool elapsed(std::uint32_t start_ms, double timeout_s) {
    return (pros::millis() - start_ms) > static_cast<std::uint32_t>(timeout_s * 1000.0);
}

}

Drivetrain::Drivetrain(Hardware& hardware, Odometry& odometry, Interrupts& interrupts)
    : hw_(hardware), odometry_(odometry), interrupts_(interrupts) {}

double Drivetrain::velocity_percent_to_rpm(double percent) {
    return (percent / 100.0) * DRIVE_MAX_RPM;
}

void Drivetrain::set_wheel_percent(double left_percent, double right_percent) {
    hw_.left.move_velocity(velocity_percent_to_rpm(left_percent));
    hw_.right.move_velocity(velocity_percent_to_rpm(right_percent));
}

void Drivetrain::stop() {
    hw_.left.move_velocity(0);
    hw_.right.move_velocity(0);
}

void Drivetrain::set_brake_mode(pros::motor_brake_mode_e mode) {
    hw_.left.set_brake_mode(mode);
    hw_.right.set_brake_mode(mode);
}

void Drivetrain::arcade(int forward, int rotate, double scale) {
    // The original opcontrol negated the turn axis, so keep steering the way
    // the driver has trained on.
    hw_.left.move(static_cast<int>((forward + rotate) * scale));
    hw_.right.move(static_cast<int>((forward - rotate) * scale));
}

void Drivetrain::tank(int left, int right, double scale) {
    hw_.left.move(static_cast<int>(left * scale));
    hw_.right.move(static_cast<int>(right * scale));
}

MotionResult Drivetrain::turn_to(double heading_deg, PidGains gains, double timeout_s) {
    set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    Pid controller(gains, TURN_INTEGRAL);
    controller.set_output_limits(-100.0, 100.0);

    const double        target     = normalise_angle(heading_deg);
    const std::uint32_t start_time = pros::millis();

    int          settled_ms    = 0;
    int          sensor_faults = 0;
    MotionResult result        = MotionResult::Timeout;

    while (true) {
        if (interrupts_.poll())             { result = MotionResult::Interrupted; break; }
        if (elapsed(start_time, timeout_s)) { result = MotionResult::Timeout;     break; }

        const double heading = odometry_.imu_heading();
        if (!std::isfinite(heading)) {
            if (++sensor_faults > MAX_SENSOR_FAULTS) { result = MotionResult::SensorFault; break; }
            pros::delay(SENSOR_RETRY_DELAY_MS);
            continue;
        }
        sensor_faults = 0;

        const double error = normalise_angle(target - heading);
        const double power = controller.update(error);

        last_turn_error_ = error;
        last_turn_power_ = power;

        set_wheel_percent(power, -power);

        settled_ms = (std::fabs(error) < TURN_SETTLE_DEG) ? settled_ms + LOOP_INTERVAL_MS : 0;
        if (settled_ms > TURN_SETTLE_MS) { result = MotionResult::Settled; break; }

        pros::delay(LOOP_INTERVAL_MS);
    }

    stop();
    return result;
}

MotionResult Drivetrain::drive_distance(double inches, PidGains gains,
                                        PidGains heading_gains, double timeout_s) {
    set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    Pid distance(gains, DRIVE_INTEGRAL);
    distance.set_derivative_smoothing(drive_tuning_.derivative_samples);
    distance.set_slew(drive_tuning_.slew);

    // Hold the heading the robot started at, if the caller asked for it and the
    // pose is actually readable right now
    bool   hold_heading   = (heading_gains.kp != 0.0 || heading_gains.kd != 0.0);
    double target_heading = 0.0;
    if (hold_heading) {
        Pose start;
        if (odometry_.read(start)) target_heading = start.heading;
        else                       hold_heading   = false;
    }
    Pid heading(heading_gains);

    const double        start_position = hw_.yrot.get_position() * inches_per_tick;
    const std::uint32_t start_time     = pros::millis();

    int          settled_ms = 0;
    MotionResult result     = MotionResult::Timeout;

    while (true) {
        if (interrupts_.poll())             { result = MotionResult::Interrupted; break; }
        if (elapsed(start_time, timeout_s)) { result = MotionResult::Timeout;     break; }

        const double travelled = (hw_.yrot.get_position() * inches_per_tick) - start_position;
        const double error     = inches - travelled;
        const double power     = distance.update(error);

        double correction = 0.0;
        if (hold_heading) {
            // A failed read means "no new information" not "off course" so
            // fall back to the target and let the correction settle to zero.
            Pose         now;
            const double current_heading = odometry_.read(now, 10) ? now.heading : target_heading;
            correction = heading.update(normalise_angle(target_heading - current_heading));
        }

        set_wheel_percent(power + correction, power - correction);

        settled_ms = (std::fabs(error) < DRIVE_SETTLE_IN) ? settled_ms + LOOP_INTERVAL_MS : 0;
        if (settled_ms > DRIVE_SETTLE_MS) { result = MotionResult::Settled; break; }

        pros::delay(LOOP_INTERVAL_MS);
    }

    stop();
    return result;
}

MotionResult Drivetrain::cordon(const CordonRequest& request) {
    set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    Pid distance(request.drive_gains, CORDON_DRIVE_INTEGRAL);
    distance.set_derivative_smoothing(cordon_tuning_.derivative_samples);
    distance.set_slew(cordon_tuning_.slew);
    distance.set_output_limits(-100.0, 100.0);
    Pid heading(request.turn_gains, CORDON_TURN_INTEGRAL);

    const std::uint32_t start_time = pros::millis();

    const double lead = std::clamp(request.lead, 0.0, MAX_LEAD);
    const double travel_heading =
        normalise_angle(request.final_heading + (request.reverse ? 180.0 : 0.0));
    const double travel_x = std::sin(travel_heading * deg2rad);
    const double travel_y = std::cos(travel_heading * deg2rad);

    int          sensor_faults = 0;
    MotionResult result        = MotionResult::Timeout;

    while (true) {
        if (interrupts_.poll())                         { result = MotionResult::Interrupted; break; }
        if (elapsed(start_time, request.drive_timeout)) { result = MotionResult::Timeout;     break; }

        Pose pose;
        if (!odometry_.read(pose)) {
            if (++sensor_faults > MAX_SENSOR_FAULTS) { result = MotionResult::SensorFault; break; }
            pros::delay(SENSOR_RETRY_DELAY_MS);
            continue;
        }
        sensor_faults = 0;

        const double dx            = request.x - pose.x;
        const double dy            = request.y - pose.y;
        const double distance_left = std::hypot(dx, dy);

        if (distance_left < request.exit_radius) { result = MotionResult::Settled; break; }

        const double trail    = distance_left * lead;
        const double carrot_x = request.x - trail * travel_x;
        const double carrot_y = request.y - trail * travel_y;

        const double to_carrot_x = carrot_x - pose.x;
        const double to_carrot_y = carrot_y - pose.y;

        double desired_heading = travel_heading;
        if (std::hypot(to_carrot_x, to_carrot_y) > CARROT_EPSILON_IN) {
            desired_heading =
                normalise_angle(90.0 - (std::atan2(to_carrot_y, to_carrot_x) * rad2deg));
        }
        // Reversing points the body the other way; the wheels do the rest
        if (request.reverse) desired_heading = normalise_angle(desired_heading + 180.0);

        const double turn_error = normalise_angle(desired_heading - pose.heading);
        const double turn_power = heading.update(turn_error);

        double drive_power = distance.update(distance_left);

        drive_power *= std::max(MIN_HEADING_SCALE, std::cos(turn_error * deg2rad));
        if (request.reverse) drive_power = -drive_power;

        double left  = drive_power + turn_power;
        double right = drive_power - turn_power;

        const double peak = std::max(std::fabs(left), std::fabs(right));
        if (peak > 100.0) {
            left  = left  / peak * 100.0;
            right = right / peak * 100.0;
        }

        set_wheel_percent(left, right);
        pros::delay(LOOP_INTERVAL_MS);
    }

    const bool aborted = (result == MotionResult::Interrupted)
                      || (result == MotionResult::SensorFault);
    if (request.stop_at_end || aborted) stop();

    if (aborted) return result;

    if (request.turn_at_end) {
        return turn_to(request.final_heading, request.final_turn_gains, request.turn_timeout);
    }
    return result;
}

}
