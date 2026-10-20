#include "robot/autons.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace robot {

namespace {

// set_pose()'s heading_side; every start pose so far has used -1.
constexpr double HEADING_SIDE = -1.0;

// Set by an auton that leaves results on lines 3-6; cleared when the next one starts.
std::atomic<bool> hold_screen{false};


// ============================================================================
//  PATHS
//
//  One function per path. Write it for RED with the yellow pins on the LEFT,
//  exactly as drawn in path.jerryio. Yellow right and blue are converted
//  automatically (see DRAWN_ALLIANCE in autons.hpp). Paste path.jerryio output
//  straight into a `const CordonRequest path[] = { ... };` and hand it to
//  ctx.run(path).
//
//  If one yellow side needs something the other doesn't (an extra pin, a
//  different route), branch on `ctx.yellow == YellowPos::Right`. Points inside
//  the branch are still written in the red/yellow-left frame and converted
//  like every other point.
//
//  Subsystems: give a point .on_start / .at_end / .pause_after, after
//  .reverse (see CordonRequest in drive.hpp), e.g.
//      .at_end      = [&] { ctx.robot.lift().grippertoggle(); },
//      .pause_after = 1.0,
//
//  To add a path: write the function, then add it to AUTONS at the bottom.
// ============================================================================

MotionResult do_nothing(const AutonContext&) {
    return MotionResult::Settled;
}

// The auton that used to live in autonomous().
MotionResult basic_close(const AutonContext& ctx) {
    // This path was run with kp 6, not the tuned 4 in autons.hpp. Kept as it
    // was; delete this line to use the shared gains.
    constexpr PidGains DRIVE_GAINS{6.00, 0.00, 4.00};

    ctx.start_at(0.0, -61.0, 0.0);

    const CordonRequest path[] = {
        {
            .x                = 0,
            .y                = -50.2,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = 1,
            .stop_at_end      = true,   // next point reverses; don't carry forward momentum into it
            .turn_at_end      = false,
            .final_heading    = 35,
            .lead             = BOOMERANG_LEAD,
        },
        {
            .x                = -10.5,
            .y                = -61.2,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = 2,
            .stop_at_end      = true,
            .turn_at_end      = false,
            .final_heading    = 45.0,
            .lead             = BOOMERANG_LEAD,
            .reverse          = true,
        },
    };
    MotionResult result = ctx.run(path);
    if (result != MotionResult::Settled) return result;

    pros::delay(500);

    const CordonRequest path2[] = {
        {
            .x                = -17.0,
            .y                = -54.6,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = 1.00,
            .stop_at_end      = true,
            .turn_at_end      = true,
            .final_heading    = -65.0,
            .lead             = BOOMERANG_LEAD,
        },
    };
    return ctx.run(path2);
}

// Template path. Copy it, rename it,
// and replace the numbers with real ones.
MotionResult basic_far(const AutonContext& ctx) {
    ctx.start_at(0.0, -61.0, 0.0);
// ===== src/main.cpp, in autonomous() ================================
    const CordonRequest path[] = {
        {
            .x                = 14,
            .y                = -44.8,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = 1.25,
            .stop_at_end      = true,
            
            .turn_at_end      = false,
            .final_heading    = 100,
            .lead             = 0.33,
            .pause_after      = 1,
        },
    

        {
            .x                = -18.6,
            .y                = -29.3,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = FINAL_RADIUS,  // curve needs the last inches to finish turning
            .stop_at_end      = false,
            .turn_at_end      = false,
            .final_heading    = 135.0,
            .lead             = 0.45,
            .reverse          = true,
        },
        /*

        {
            .x                = -23.2,
            .y                = -36.8,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = FINAL_RADIUS,
            .stop_at_end      = false,
            .turn_at_end      = false,
            .final_heading    = 180.0,
            .lead             = BOOMERANG_LEAD,
        },
        */
    };

    return ctx.run(path);
}

MotionResult awp(const AutonContext& ctx) {
    ctx.start_at(0.0, -61.0, 0.0);

    const CordonRequest path[] = {
        {
            .x                = -24.0,
            .y                = -36.0,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = FINAL_RADIUS,
            .stop_at_end      = true,
            .turn_at_end      = false,
            .final_heading    = -45.0,
            .lead             = BOOMERANG_LEAD,
        },
    };
    return ctx.run(path);
}

MotionResult skills(const AutonContext& ctx) {
    ctx.start_at(0.0, -61.0, 0.0);

    const CordonRequest path[] = {
        {
            .x                = -24.0,
            .y                = -36.0,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = FINAL_RADIUS,
            .stop_at_end      = true,
            .turn_at_end      = false,
            .final_heading    = -45.0,
            .lead             = BOOMERANG_LEAD,
        },
    };
    return ctx.run(path);
}

MotionResult both_yellow(const AutonContext& ctx) {
    ctx.start_at(0.0, -61.0, 0.0);

    const CordonRequest path[] = {
        {
            .x                = -24.0,
            .y                = -36.0,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = FINAL_RADIUS,
            .stop_at_end      = true,
            .turn_at_end      = false,
            .final_heading    = -45.0,
            .lead             = BOOMERANG_LEAD,
        },
    };
    return ctx.run(path);
}

MotionResult no_yellow(const AutonContext& ctx) {
    ctx.start_at(0.0, -61.0, 0.0);

    const CordonRequest path[] = {
        {
            .x                = -24.0,
            .y                = -36.0,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = FINAL_RADIUS,
            .stop_at_end      = true,
            .turn_at_end      = false,
            .final_heading    = -45.0,
            .lead             = BOOMERANG_LEAD,
        },
    };
    return ctx.run(path);
}

// ---- tracker offset tuning -------------------------------------------------
//
// Spins on the spot, both ways, and works out where each tracking wheel sits
// relative to the tracking centre. In a pure spin the robot does not move, so
// whatever a tracker reads is the arc it swept, and odometry.cpp cancels it as
//     delta_x_in - dh * x_tracker_offset = 0  ->  x_off =  sum(dx) / sum(dh)
//     delta_y_in + dh * y_tracker_offset = 0  ->  y_off = -sum(dy) / sum(dh)
// Both directions are averaged so drift and scrub one way cancel the other.
// Prints the result; it does not change OdometryConfig, copy it in by hand.

constexpr int    TUNE_SPIN_POWER = 35;     // out of 127; slow keeps the wheels from scrubbing
constexpr double TUNE_SPIN_DEG   = 1800.0;  // 5 turns each way

struct SpinTotals {
    double dx_in  = 0.0;
    double dy_in  = 0.0;
    double dh_rad = 0.0;
};

// Gives up on the spin with a reason, so the screen says why instead of a
// bare "cut by". trip() keeps the first reason if an interrupt already fired.
bool give_up(Robot& robot, const char* reason) {
    robot.drivetrain().stop();
    robot.interrupts().trip(reason);
    return false;
}

// Spins until the IMU has turned TUNE_SPIN_DEG, summing tracker travel and heading.
bool spin_and_measure(Robot& robot, int rotate, SpinTotals& out) {
    Hardware& hw = robot.hardware();

    const double       first_h = robot.odometry().fused_imu_raw();
    const std::int32_t first_x = hw.xrot.get_position();
    const std::int32_t first_y = hw.yrot.get_position();
    if (!std::isfinite(first_h)) return give_up(robot, "no IMU heading");
    if (first_x == PROS_ERR)     return give_up(robot, "no X tracker");
    if (first_y == PROS_ERR)     return give_up(robot, "no Y tracker");

    double prev_h = first_h;
    double prev_x = first_x;
    double prev_y = first_y;

    double turned_deg = 0.0;
    const std::uint32_t start = pros::millis();

    while (std::abs(turned_deg) < TUNE_SPIN_DEG) {
        if (robot.interrupts().poll())      return give_up(robot, "");
        if (pros::millis() - start > 30000) return give_up(robot, "spin took >30s");

        robot.drivetrain().arcade(0, rotate);
        pros::delay(LOOP_INTERVAL_MS);

        const double h = robot.odometry().fused_imu_raw();
        const std::int32_t x = hw.xrot.get_position();
        const std::int32_t y = hw.yrot.get_position();
        if (!std::isfinite(h) || x == PROS_ERR || y == PROS_ERR) continue;

        const double dh = normalise_angle(h - prev_h);
        turned_deg  += dh;
        out.dh_rad  += dh * deg2rad;
        out.dx_in   += (x - prev_x) * inches_per_tick;
        out.dy_in   += (y - prev_y) * inches_per_tick;
        prev_h = h;
        prev_x = x;
        prev_y = y;
    }

    robot.drivetrain().stop();
    pros::delay(1000);  // let it stop and settle before the next spin
    return true;
}

MotionResult tune_odom_offsets(const AutonContext& ctx) {
    Robot& robot = ctx.robot;
    robot.drivetrain().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(500);

    SpinTotals cw, ccw;
    if (!spin_and_measure(robot,  TUNE_SPIN_POWER, cw) ||
        !spin_and_measure(robot, -TUNE_SPIN_POWER, ccw)) {
        return MotionResult::Interrupted;  // main.cpp prints the reason
    }

    const double x_cw  =  cw.dx_in  / cw.dh_rad;
    const double x_ccw =  ccw.dx_in / ccw.dh_rad;
    const double y_cw  = -cw.dy_in  / cw.dh_rad;
    const double y_ccw = -ccw.dy_in / ccw.dh_rad;
    const double x_off = (x_cw + x_ccw) / 2.0;
    const double y_off = (y_cw + y_ccw) / 2.0;

    const OdometryConfig now = robot.odometry().config();
    pros::lcd::print(3, "x_off %.2f (cw %.2f ccw %.2f)", x_off, x_cw, x_ccw);
    pros::lcd::print(4, "y_off %.2f (cw %.2f ccw %.2f)", y_off, y_cw, y_ccw);
    pros::lcd::print(5, "current x %.2f y %.2f", now.x_tracker_offset, now.y_tracker_offset);
    pros::lcd::print(6, "turned cw %.0f ccw %.0f deg", cw.dh_rad * rad2deg, ccw.dh_rad * rad2deg);
    std::printf("odom offsets: x %.3f (cw %.3f ccw %.3f)  y %.3f (cw %.3f ccw %.3f)\n",
                x_off, x_cw, x_ccw, y_off, y_cw, y_ccw);

    // Keep opcontrol's drive telemetry off lines 3-6 so this stays readable.
    hold_screen.store(true);
    return MotionResult::Settled;
}

// ---- turn test ----------------------------------------------------------------
//
// Turns on the spot through a 90 and a 180 and back, with TURN_GAINS, and
// shows where each one ended. R1 on the controller runs it.

MotionResult turn_test(const AutonContext& ctx) {
    Robot& robot = ctx.robot;
    ctx.start_at(0.0, -61.0, 0.0);

    const double targets[] = {90.0, 0.0, 180.0, 0.0};
    const PidGains gains = TURN_GAINS;
    MotionResult result = MotionResult::Settled;

    for (int i = 0; i < 4; i++) {
        const std::uint32_t start = pros::millis();
        result = robot.drivetrain().turn_to(targets[i], gains, TURN_TIMEOUT);
        const std::uint32_t took = pros::millis() - start;

        pros::delay(300);  // let it coast, so overshoot after "settled" shows up too
        const double err = normalise_angle(targets[i] - robot.odometry().imu_heading());

        pros::lcd::print(3 + i, "to %4.0f: err %5.1f  %4lums %s", targets[i], err,
                         static_cast<unsigned long>(took), to_string(result));
        std::printf("turn test: to %.0f err %.2f in %lu ms (%s) kp %.2f kd %.2f\n",
                    targets[i], err, static_cast<unsigned long>(took), to_string(result),
                    gains.kp, gains.kd);
        if (result == MotionResult::Interrupted || result == MotionResult::SensorFault) break;
    }

    robot.drivetrain().stop();
    hold_screen.store(true);
    return result;
}

// Everything the selector can choose from, in the order the button cycles.
// The first entry is what runs if nobody touches the selector.
const Auton AUTONS[] = {
    {"basic far",      basic_far},
    {"basic close",    basic_close},
    {"awp",            awp},
    {"both yellow (other teams poop)", both_yellow},
    {"no yellow cuz where poop", no_yellow},
    {"skills",         skills},
    {"tune offsets",   tune_odom_offsets},
    {"turn test",      turn_test},
    {"do nothing",     do_nothing},
    

};
constexpr int AUTON_COUNT = sizeof(AUTONS) / sizeof(AUTONS[0]);

// ============================================================================
//  SELECTOR
// ============================================================================

std::atomic<int> sel_alliance{static_cast<int>(DRAWN_ALLIANCE)};
std::atomic<int> sel_yellow  {static_cast<int>(YellowPos::Left)};
std::atomic<int> sel_path    {0};

// All on line 7: opcontrol's drive telemetry rewrites lines 3-6 every few
// loops, and only show_result() (after auton) touches line 7.
void draw_selection() {
    pros::lcd::print(7, "%s | %s | %s",
                     alliance_name(static_cast<Alliance>(sel_alliance.load())),
                     yellow_name(static_cast<YellowPos>(sel_yellow.load())),
                     AUTONS[sel_path.load()].name);
}

void next_alliance() { sel_alliance.store((sel_alliance.load() + 1) % 2); draw_selection(); }
void next_yellow()   { sel_yellow.store((sel_yellow.load() + 1) % 2);     draw_selection(); }
void next_path()     { sel_path.store((sel_path.load() + 1) % AUTON_COUNT); draw_selection(); }

// Reflect across x = y: yellow left -> yellow right, same alliance.
// A heading's direction (sin h, cos h) becomes (cos h, sin h), i.e. 90 - h.
void swap_yellow(double& x, double& y, double& heading) {
    const double old_x = x;
    x       = y;
    y       = old_x;
    heading = normalise_angle(90.0 - heading);
}

// Turn half way round: one alliance -> the other, same yellow side.
void swap_alliance(double& x, double& y, double& heading) {
    x       = -x;
    y       = -y;
    heading = normalise_angle(heading + 180.0);
}

// Applies whichever of the two the selection needs. Order does not matter:
// both together are a reflection across x = -y either way round.
void convert(const AutonContext& ctx, double& x, double& y, double& heading) {
    if (ctx.yellow == YellowPos::Right)  swap_yellow(x, y, heading);
    if (ctx.alliance != DRAWN_ALLIANCE)  swap_alliance(x, y, heading);
}

}

const char* alliance_name(Alliance alliance) {
    return alliance == Alliance::Red ? "RED" : "BLUE";
}

const char* yellow_name(YellowPos yellow) {
    return yellow == YellowPos::Left ? "LEFT" : "RIGHT";
}

CordonRequest AutonContext::as_driven(CordonRequest point) const {
    convert(*this, point.x, point.y, point.final_heading);
    return point;
}

MotionResult AutonContext::wait(double seconds) const {
    const std::uint32_t end = pros::millis() + static_cast<std::uint32_t>(seconds * 1000.0);
    while (pros::millis() < end) {
        if (robot.interrupts().poll()) return MotionResult::Interrupted;
        pros::delay(LOOP_INTERVAL_MS);
    }
    return MotionResult::Settled;
}

void AutonContext::start_at(double x, double y, double heading) const {
    convert(*this, x, y, heading);
    robot.odometry().set_pose(x, y, heading, HEADING_SIDE);
}

void auton_selector_start() {
    pros::lcd::register_btn0_cb(next_alliance);
    pros::lcd::register_btn1_cb(next_yellow);
    pros::lcd::register_btn2_cb(next_path);
    draw_selection();
}

bool auton_screen_held() {
    return hold_screen.load();
}

MotionResult run_selected_auton(Robot& robot) {
    hold_screen.store(false);
    const AutonContext ctx{
        robot,
        static_cast<Alliance>(sel_alliance.load()),
        static_cast<YellowPos>(sel_yellow.load()),
    };
    return AUTONS[sel_path.load()].run(ctx);
}

const char* selected_auton_name() {
    return AUTONS[sel_path.load()].name;
}

MotionResult run_turn_test(Robot& robot) {
    hold_screen.store(false);
    const AutonContext ctx{robot, DRAWN_ALLIANCE, YellowPos::Left};
    return turn_test(ctx);
}

}
