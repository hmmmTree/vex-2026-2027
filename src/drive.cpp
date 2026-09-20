#include "robot/drive.hpp"
#include "robot/pid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

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

constexpr int DRIVE_DISPLAY_EVERY_TICKS = 5;

constexpr double MIN_HEADING_SCALE = 0.0;

// The odometry task publishes on its own cadence, which is its 20 ms delay plus
// three smart-port device reads. The control loop samples at a flat 20 ms, so
// it sees the same heading twice and then a double-sized step. Differentiating
// that raw gives a spike on every step; averaging a few samples gives the slope.
constexpr std::size_t CORDON_HEADING_SMOOTHING = 3;

// Faster than the drivetrain can pivot at DRIVE_MAX_RPM, so any sample above
// this did not come from the robot moving.
constexpr double IMPOSSIBLE_DEG_PER_S = 400.0;

constexpr double MAX_LEAD = 0.15;

constexpr double CARROT_EPSILON_IN = 1.5;

constexpr int MAX_SENSOR_FAULTS     = 50;
constexpr int SENSOR_RETRY_DELAY_MS = 10;

// TEMPORARY trace storage. Samples are stashed here during the motion and
// printed once it ends: a blocking printf inside a 20 ms control loop stretches
// the tick and changes the behaviour being measured.
struct TraceSample {
    std::uint16_t ms;
    float dist, aim, head, terr, drive, turn, left, right;
};
constexpr int TRACE_MAX = 220;
TraceSample trace_buf[TRACE_MAX];

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

void Drivetrain::reverse() {
    hw_.left.set_reversed_all(!hw_.left.is_reversed());
    hw_.right.set_reversed_all(!hw_.right.is_reversed());
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

    int          settled_ms      = 0;
    int          display_counter = 0;
    MotionResult result          = MotionResult::Timeout;

    while (true) {
        if (interrupts_.poll())             { result = MotionResult::Interrupted; break; }
        if (elapsed(start_time, timeout_s)) { result = MotionResult::Timeout;     break; }

        const double travelled = (hw_.yrot.get_position() * inches_per_tick) - start_position;
        const double error     = inches - travelled;
        const double power     = distance.update(error);

        if (++display_counter >= DRIVE_DISPLAY_EVERY_TICKS) {
            display_counter = 0;
            pros::lcd::print(5, "DRV trv %.1f err %.1f pwr %.0f", travelled, error, power);
        }

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
    heading.set_derivative_smoothing(CORDON_HEADING_SMOOTHING);

    const std::uint32_t start_time = pros::millis();

    const double lead = std::clamp(request.lead, 0.0, MAX_LEAD);
    const double travel_heading =
        normalise_angle(request.final_heading + (request.reverse ? 180.0 : 0.0));
    const double travel_x = std::sin(travel_heading * deg2rad);
    const double travel_y = std::cos(travel_heading * deg2rad);

    int          sensor_faults = 0;
    MotionResult result        = MotionResult::Timeout;

    // TEMPORARY oscillation telemetry.
    int    osc_ticks    = 0;
    int    osc_flips    = 0;
    double osc_maxerr   = 0.0;
    double osc_mindist  = 1e9;
    double osc_preverr  = 0.0;
    bool   osc_haveprev = false;
    double osc_dhmin    =  1e9;
    double osc_dhmax    = -1e9;
    double osc_hmin     =  1e9;
    double osc_hmax     = -1e9;
    double        osc_maxrate = 0.0;
    double        osc_prevh   = 0.0;
    std::uint32_t osc_prevms  = 0;
    bool          osc_haveh   = false;
    int           osc_readfail = 0;
    int           osc_held     = 0;
    int           osc_jumps    = 0;
    double        osc_sumdelta = 0.0;
    double        osc_sumdt    = 0.0;

    int trace_n = 0;

    while (true) {
        if (interrupts_.poll())                         { result = MotionResult::Interrupted; break; }
        if (elapsed(start_time, request.drive_timeout)) { result = MotionResult::Timeout;     break; }

        Pose pose;
        if (!odometry_.read(pose)) {
            osc_readfail++;
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

        // TEMPORARY oscillation telemetry.
        osc_ticks++;
        if (std::fabs(turn_error) > osc_maxerr) osc_maxerr  = std::fabs(turn_error);
        if (distance_left < osc_mindist)        osc_mindist = distance_left;
        if (osc_haveprev && ((turn_error > 0.0) != (osc_preverr > 0.0))) osc_flips++;
        osc_preverr  = turn_error;
        osc_haveprev = true;
        if (desired_heading < osc_dhmin) osc_dhmin = desired_heading;
        if (desired_heading > osc_dhmax) osc_dhmax = desired_heading;
        if (pose.heading < osc_hmin) osc_hmin = pose.heading;
        if (pose.heading > osc_hmax) osc_hmax = pose.heading;
        // Real elapsed time, not the nominal tick: a retried odometry read can
        // put 40 or 60 ms between two samples, and dividing those by 20 ms
        // reports a turn rate two or three times the one the robot managed.
        const std::uint32_t now_ms = pros::millis();
        if (osc_haveh && now_ms > osc_prevms) {
            const double dt_s  = (now_ms - osc_prevms) / 1000.0;
            const double delta = std::fabs(normalise_angle(pose.heading - osc_prevh));
            const double rate  = delta / dt_s;
            if (rate > osc_maxrate) osc_maxrate = rate;

            // Average turn rate, and a count of samples the drivetrain could
            // not physically have produced. A high max with a low average and a
            // handful of jumps is a sensor spiking; a high average is the robot
            // genuinely spinning that fast.
            osc_sumdelta += delta;
            osc_sumdt    += dt_s;
            if (rate > IMPOSSIBLE_DEG_PER_S) osc_jumps++;
        }
        // A tick whose heading is bit-identical to the last one means the
        // odometry task has not published since. The proportion of these is the
        // staircase, measured directly instead of inferred.
        if (osc_haveh && pose.heading == osc_prevh) osc_held++;

        osc_prevh  = pose.heading;
        osc_prevms = now_ms;
        osc_haveh  = true;

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

        // TEMPORARY per-tick trace. Stash only, no I/O in the loop.
        if (trace_n < TRACE_MAX && (osc_ticks % 5) == 0) {
            trace_buf[trace_n++] = TraceSample{
                static_cast<std::uint16_t>(pros::millis() - start_time),
                static_cast<float>(distance_left),   static_cast<float>(desired_heading),
                static_cast<float>(pose.heading),    static_cast<float>(turn_error),
                static_cast<float>(drive_power),     static_cast<float>(turn_power),
                static_cast<float>(left),            static_cast<float>(right)};
        }

        set_wheel_percent(left, right);
        pros::delay(LOOP_INTERVAL_MS);
    }

    // TEMPORARY oscillation telemetry.
    // TEMPORARY trace dump. Written to the terminal and, if a microSD card is
    // fitted, appended to /usd/cordon.csv so the data survives without one.
    const std::uint32_t trace_ms = pros::millis() - start_time;
    std::FILE*          sd       = std::fopen("/usd/cordon.csv", "a");
    std::FILE*          sinks[2] = {stdout, sd};

    for (std::FILE* out : sinks) {
        if (out == nullptr) continue;
        std::fprintf(out, "\n# cordon -> %.1f,%.1f head %.1f lead %.2f exit %.1f : %s\n",
                     request.x, request.y, request.final_heading, lead,
                     request.exit_radius, to_string(result));
        std::fprintf(out, "# ticks %d over %lu ms, %.1f ms/tick\n", osc_ticks,
                     static_cast<unsigned long>(trace_ms),
                     osc_ticks > 0 ? static_cast<double>(trace_ms) / osc_ticks : 0.0);
        std::fprintf(out, "# t_ms,dist,aim,head,turn_err,drive,turn,left,right\n");
        for (int i = 0; i < trace_n; i++) {
            const TraceSample& s = trace_buf[i];
            std::fprintf(out, "%u,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.0f,%.0f\n",
                         static_cast<unsigned>(s.ms), static_cast<double>(s.dist),
                         static_cast<double>(s.aim), static_cast<double>(s.head),
                         static_cast<double>(s.terr), static_cast<double>(s.drive),
                         static_cast<double>(s.turn), static_cast<double>(s.left),
                         static_cast<double>(s.right));
        }
        std::fflush(out);
    }

    if (sd != nullptr) std::fclose(sd);
    pros::lcd::print(1, "held %d/%d rf %d %.1fms sd:%s", osc_held, osc_ticks, osc_readfail,
                     osc_ticks > 0 ? static_cast<double>(trace_ms) / osc_ticks : 0.0,
                     sd != nullptr ? "ok" : "none");

    pros::lcd::print(0, "hd swing %.0f rate %.0f/s",
                     osc_hmax - osc_hmin, osc_maxrate);
    pros::lcd::print(3, "ticks %d flips %d", osc_ticks, osc_flips);
    pros::lcd::print(4, "maxerr %.0f mindst %.1f", osc_maxerr, osc_mindist);
    pros::lcd::print(6, "aim %.0f jmp %d avg %.0f", osc_dhmax - osc_dhmin, osc_jumps,
                     osc_sumdt > 0.0 ? osc_sumdelta / osc_sumdt : 0.0);
    pros::lcd::print(5, "period %.0f ms",
                     osc_flips > 1 ? (2.0 * osc_ticks * LOOP_INTERVAL_MS) / osc_flips : 0.0);

    const bool aborted = (result == MotionResult::Interrupted)
                      || (result == MotionResult::SensorFault);
    if (request.stop_at_end || aborted) stop();

    if (aborted) return result;

    if (request.turn_at_end) {
        // Let the drive come to rest first. turn_to builds its controller fresh,
        // so on the first tick it has no derivative history and cannot see the
        // rotation the robot is already carrying out of the drive phase. Starting
        // the pivot mid-roll hands it momentum it does not know about, and it
        // sails past the target.
        stop();
        pros::delay(200);
        return turn_to(request.final_heading, request.final_turn_gains, request.turn_timeout);
    }
    return result;
}

}
