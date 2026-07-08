#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include "pros/ai_vision.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({11, -16, 20});
pros::MotorGroup right_mg({-19, 13, -17});

// TODO: TEMPORARY TEST PORTS — 19 and 20 are already taken by drive motors,
// and a smart port only supports one device, so the IMU and X tracker do NOT
// work as wired here (get_heading() returns inf, get_position() returns
// PROS_ERR, and odometry is garbage). Move both to free ports before running
// odometry or auton.
pros::Imu inertial(19);      // WARNING: port 19 is also a right-drive motor!
pros::Rotation xrot(20);     // WARNING: port 20 is also a left-drive motor!
pros::Rotation yrot(1);
pros::AIVision aivision(5);

const double conversion_jerry2in = 36.0/35.0;

static double botx = 0;
static double boty = 0;
static double both = 0;
static double starting_heading = 0.0;

static double debug_turn_error = 0.0;
static double debug_turn_power = 0.0;
// middle goal is (0,0)
// heading 0 is facing perpendicular to field wall

//.05 tkd
//3.5 tkp

const double ekp = 0;// need to tune dis
const double ekd = 0;

double saved_drive_error = 0.0;

double inith = 0.0;
double initv = 0.0;

double rad2deg = 180.0 / acos(-1.0);
double deg2rad = M_PI / 180.0;

double adjustment = .01; // need to tune dis

double ticks_per_rev = 36000.0;
double wheel_diameter = 2.0;
double inches_per_tick = (wheel_diameter * M_PI) / ticks_per_rev;

// ── Tracking wheel offsets from the robot's CENTER OF ROTATION (inches) ──
// Even though heading comes from the IMU, an off-center tracking wheel rolls
// along an arc whenever the robot spins, recording distance that is NOT real
// translation. Each odometry loop subtracts that arc (offset × Δheading).
//   X_TRACKER_OFFSET: how far the sideways (x) tracker sits IN FRONT of the
//                     center of rotation (negative if behind it)
//   Y_TRACKER_OFFSET: how far the forward (y) tracker sits to the RIGHT of
//                     the center of rotation (negative if left of it)
// TODO: MEASURE these on the robot and fill them in — 0 disables the
// correction (current behavior). Quick check after measuring: spin the bot
// in place; X/Y on the LCD should barely move.
double X_TRACKER_OFFSET = 0.0;  // inches — MEASURE ME
double Y_TRACKER_OFFSET = 0.0;  // inches — MEASURE ME

double headingside = 1;


struct FieldTag { int id; double x; double y; };
static const FieldTag TAGS[] = {
    // { id, x_inches,  y_inches }
    {  1,   0.0,   72.0 },   //replace with your measured positions
    {  2,  72.0,    0.0 },
    {  3,   0.0,  -72.0 },
    {  4, -72.0,    0.0 },
};
static const int N_TAGS = sizeof(TAGS) / sizeof(TAGS[0]);

static constexpr double HFOV_DEG = 73.0;
static constexpr double IMAGE_W  = 320.0;
static const double FOCAL_PX = (IMAGE_W / 2.0) / std::tan(HFOV_DEG / 2.0 * M_PI / 180.0);
static constexpr double TAG_SIZE = 5.0;  // inches — MEASURE YOUR TAG AND CHANGE THIS

// NOTE: AprilTag detections carry no confidence score in the AI Vision API
// (only AI-model detections do), so tag quality is filtered by width instead.
static constexpr double MIN_WIDTH = 15;    // skip tags smaller than 15px (too far/blurry)
static constexpr double MAX_JUMP  = 12.0;  // skip fix if > 12" from the default start pose

static constexpr double BLEND = 0.3;

static double   av_last_dist = 0;
static double   av_est_x     = 0;
static double   av_est_y     = 0;
static uint32_t av_updates   = 0;

pros::Mutex odom_lock;

double normalise_angle(double angle) {
    angle = fmod(angle, 360.0);

    if (angle > 180.0)  angle -= 360.0;
    if (angle < -180.0) angle += 360.0;

    return angle;
}

void set_pose(double x, double y, double heading, double headingside_param) {
    // Offset the IMU so (imu + starting_heading) equals `heading` right now.
    // This works mid-run too — not just right after calibration, when the
    // IMU happens to read 0.
    double imu_now = inertial.get_heading();
    if (std::isinf(imu_now) || std::isnan(imu_now)) imu_now = 0.0;

    while(true)   {
        if (odom_lock.take(50)) {  // 50ms timeout instead of TIMEOUT_MAX
            botx = x;
            boty = y;
            headingside = headingside_param;
            starting_heading = normalise_angle(heading - imu_now);
            both = normalise_angle(heading);
            odom_lock.give();
            break;
        } else {
            pros::lcd::print(0, "Set Pose: Mutex timeout!");
        }
    }

    // Print to confirm
    pros::lcd::print(0, "POSE SET: %.2f %.2f", x, y);
}



// ── One-shot AprilTag starting-pose fix ────────────────────────────────
// Tries for (max_attempts × 50ms) to see a mapped tag. On success it SETS
// botx/boty from the largest (closest → most accurate) visible tag and
// returns true. On failure it leaves the current default pose untouched
// and returns false. Uses the KNOWN start heading, so no IMU required.
bool apriltag_initial_fix(double start_heading, int max_attempts) {
    // Default/start pose (set_pose was called just before this) — used by
    // the MAX_JUMP sanity check below
    double def_x = 0, def_y = 0;
    if (odom_lock.take(50)) { def_x = botx; def_y = boty; odom_lock.give(); }

    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        int count = aivision.get_object_count();
        pros::lcd::print(1, "FIX try %d/%d objs:%d", attempt, max_attempts, count);

        double best_width = 0, best_x = 0, best_y = 0;
        int    best_id = -1;

        for (int i = 0; i < count; i++) {
            pros::AIVision::Object obj = aivision.get_object(i);
            if (!pros::AIVision::is_type(obj, pros::AivisionDetectType::tag)) continue;

            const auto& t = obj.object.tag;
            double min_x = std::min(std::min((double)t.x0, (double)t.x1),
                                    std::min((double)t.x2, (double)t.x3));
            double max_x = std::max(std::max((double)t.x0, (double)t.x1),
                                    std::max((double)t.x2, (double)t.x3));
            double tag_width = max_x - min_x;

            pros::lcd::print(3, "saw T%d w:%.0f", obj.id, tag_width);
            if (tag_width < MIN_WIDTH) continue;

            const FieldTag* tag = nullptr;
            for (int k = 0; k < N_TAGS; k++)
                if (TAGS[k].id == obj.id) { tag = &TAGS[k]; break; }
            if (!tag) { pros::lcd::print(4, "T%d not in map", obj.id); continue; }

            double distance     = (TAG_SIZE * FOCAL_PX) / tag_width;
            double pixel_offset = ((min_x + max_x) * 0.5) - (IMAGE_W / 2.0);
            double angle_offset = std::atan2(pixel_offset, FOCAL_PX) * rad2deg;
            double bearing_rad  = normalise_angle(start_heading + angle_offset) * deg2rad;

            double est_x = tag->x - distance * std::sin(bearing_rad);
            double est_y = tag->y - distance * std::cos(bearing_rad);

            // Sanity check: the fix can't be far from where the robot was
            // physically placed on the field
            if (std::hypot(est_x - def_x, est_y - def_y) > MAX_JUMP) {
                pros::lcd::print(4, "T%d fix too far off", obj.id);
                continue;
            }

            if (tag_width > best_width) {          // keep the closest tag this frame
                best_width = tag_width;
                best_x = est_x;  best_y = est_y;  best_id = obj.id;
            }
        }

        if (best_id >= 0) {
            if (odom_lock.take(50)) {
                botx = best_x;
                boty = best_y;
                odom_lock.give();
            }
            pros::lcd::print(1, "FIX OK T%d (%.1f,%.1f)", best_id, best_x, best_y);
            return true;
        }
        pros::delay(50);
    }
    pros::lcd::print(1, "FIX: none - default pose");
    return false;
}


void updatepose(void* ignore){
    double prev_x = 0;
    double prev_y = 0;
    double prev_raw_h = 0;      // raw IMU reading (no starting_heading offset)
    bool have_prev_h = false;
    int loop_count = 0;
    int display_counter = 0;

    xrot.reset_position();
    yrot.reset_position();

    while(true) {
        loop_count++;
        
        // 1. Get Sensor Readings
        double curr_x = xrot.get_position();
        double curr_y = yrot.get_position();
        double raw_h  = inertial.get_heading();
        double curr_h = raw_h + starting_heading;

        // 2. Check for IMU errors — one inf/NaN reading would poison
        // botx/boty permanently (NaN never leaves the accumulator)
        if (std::isinf(curr_h) || std::isnan(curr_h)) {
            pros::delay(10);
            continue;
        }
        // Δheading uses the RAW reading so a set_pose() mid-run (which shifts
        // starting_heading) can't show up as a fake rotation
        if (!have_prev_h) { prev_raw_h = raw_h; have_prev_h = true; }

        // 3. Calculate deltas
        double delta_x = (curr_x - prev_x) * inches_per_tick;
        double delta_y = (curr_y - prev_y) * inches_per_tick;
        double delta_h_rad = normalise_angle(raw_h - prev_raw_h) * deg2rad;
        prev_raw_h = raw_h;

        // 3b. Tracking wheel offset compensation: remove the arc the wheels
        // roll when the robot rotates (clockwise spin → x tracker mounted in
        // front rolls right, y tracker mounted right of center rolls back).
        // Without this, every turn leaks a bit of fake x/y movement.
        delta_x -= delta_h_rad * X_TRACKER_OFFSET;
        delta_y += delta_h_rad * Y_TRACKER_OFFSET;

        double theta = -normalise_angle(curr_h) * deg2rad;

        // 4. Update odometry with mutex protection - USE TIMEOUT
        if (!odom_lock.take(50)) {
            // If we can't get the lock in 50ms, skip this update
            pros::delay(10);
            continue;
        }
        
        botx += delta_x * cos(theta) - delta_y * sin(theta);
        boty += delta_x * sin(theta) + delta_y * cos(theta);
        both = normalise_angle(curr_h);
        
        odom_lock.give();

        prev_x = curr_x;
        prev_y = curr_y;

        // 5. CRITICAL: Only update LCD every 10 loops (reduces crash risk)
        display_counter++;
        if (display_counter >= 10) {
            display_counter = 0;
            pros::lcd::print(2, "X:%.1f Y:%.1f H:%.0f", botx, boty, both);
        }

        pros::delay(20); 
    }
}


// REMOVED: the global `pros::Task odometry_task_pose(updatepose, nullptr);`
// that used to be here. It launched a SECOND copy of updatepose alongside the
// one created in initialize(), so every wheel movement was added to botx/boty
// TWICE (odometry read ~2x the real distance). The task is now started once,
// inside initialize().


void initialize() {
    pros::lcd::initialize();

    left_mg.move_velocity(0);
    right_mg.move_velocity(0);
    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    xrot.reset();
    yrot.reset();
    yrot.set_reversed(true);

    // NEW: calibrate the IMU before odometry starts trusting its heading.
    // Blocks for ~2 seconds — the robot must be COMPLETELY STILL during this.
    inertial.reset(true);

    // Odometry runs continuously in the background (started ONCE, here)
    pros::Task odometry_task_pose(updatepose, nullptr);

    // Configure vision, then let it settle before the one-shot fix
    aivision.enable_detection_types(pros::AivisionModeType::tags);
    aivision.set_tag_family(pros::AivisionTagFamily::tag_25H9);
    aivision.start_awb();
    pros::delay(300);

    // Fallback/default pose first (this also sets starting_heading)
    double start_heading = 0.0;
    set_pose(0, 0, start_heading, -1);

    // Try once (~0.75s) to override with a tag-derived pose.
    // If no mapped tag is seen, the default above is kept.
    apriltag_initial_fix(start_heading, 15);
}


void disabled() {
}


void competition_initialize() {

}


// move_velocity() scale: must match the CONFIGURED gearset, not the physical
// cartridge. The drive cartridges are blue (600 RPM) but the motors are set
// to green, so ±200 here still spans the full speed range. If the configured
// gearset ever changes, change this too (red = 100, green = 200, blue = 600).
static constexpr double DRIVE_MAX_RPM = 200.0;

static double vperc(double perc) {
	return (perc / 100.0) * DRIVE_MAX_RPM;
}

void opcontrol() {
	// The autonomous PIDs leave the drive in HOLD — release it for the driver
	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	double isdone = 0;
	double speed = 1;
	while (true){
		int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        int left_power = (forward - turn*-1) * speed;
        int right_power = (forward + turn*-1) * speed;

        left_mg.move(left_power);
        right_mg.move(right_power);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) speed = 1.0;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) speed = 0.4;
        
        pros::delay(20);	}
}


//odometry + PID (now with I!)
//
// NEW SIGNATURE: turn(angle, kd, kp, ki, timeout)
//
// Integral notes:
//  - Only accumulates inside I_ZONE (close to target). Far away it stays 0,
//    so it can't wind up during the long approach.
//  - Resets the moment error changes sign (we crossed the target), which
//    kills the classic integral-overshoot oscillation.
//  - Its total contribution is capped at I_MAX percent.
// Derivative is deliberately kept as (error - prev_error), NOT divided by dt,
// so your existing kd tuning (~0.05) still means the same thing.
void turn(double ang, double kd, double kp, double ki, double timeout) {
    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
    double target_heading = normalise_angle(ang);
    double error = 0;
    double prev_error = 0;
    double derivative = 0;
    double integral = 0;
    double tvel = 0;

    const double I_ZONE = 10.0;   // deg — only integrate when within this of target
    const double I_MAX  = 30.0;   // max ± percent the I term may contribute
    
    double starting_time = pros::millis();
    int time_settled = 0;
    bool first_loop = true;

    while (true) {
        // Read the IMU directly — `both` is only written every 20 ms by the
        // odometry task, so it can be one cycle stale and jitter the D term
        double current_heading = inertial.get_heading() + starting_heading;
        if (std::isinf(current_heading) || std::isnan(current_heading)) {
            pros::delay(10);
            continue;
        }
        current_heading = normalise_angle(current_heading);

        // Calculate Error
        error = normalise_angle(target_heading - current_heading);

        // Prevent derivative kick on first loop
        if (first_loop) {
            prev_error = error;
            first_loop = false;
        }

        derivative = error - prev_error;

        // ---- INTEGRAL (with anti-windup) ----
        // 1) crossed the target? dump the integral so it can't drag us past
        if ((error > 0 && prev_error < 0) || (error < 0 && prev_error > 0)) {
            integral = 0;
        }
        // 2) only accumulate near the target, 3) cap the contribution
        if (fabs(error) < I_ZONE) {
            integral += error * 0.02;                 // dt = 20 ms
            if (ki != 0) {
                double lim = fabs(I_MAX / ki);
                if (integral >  lim) integral =  lim;
                if (integral < -lim) integral = -lim;
            }
        } else {
            integral = 0;
        }

        prev_error = error;

        tvel = (error * kp) + (derivative * kd) + (integral * ki);
        if (tvel >  100.0) tvel =  100.0;
        if (tvel < -100.0) tvel = -100.0;

        debug_turn_error = error;
        debug_turn_power = tvel;

        left_mg.move_velocity(vperc(tvel));
        right_mg.move_velocity(vperc(-tvel));

        // Exit Logic
        if (pros::millis() - starting_time > timeout * 1000) break;
        if (fabs(error) < 0.5) time_settled += 20;
        else time_settled = 0;
        if (time_settled > 100) break;

        pros::delay(20);
    }
    left_mg.move_velocity(0);
    right_mg.move_velocity(0);
}


// NEW SIGNATURE: drive(distance, kd, kp, ki, ekd, ekp, timeout)
//
// - ki: integral on distance, same anti-windup scheme as turn().
// - ekd/ekp: these were accepted but IGNORED before. They now do what they
//   were clearly meant for — a heading-hold P(D) that locks the heading
//   measured at the start of the drive and steers back if the bot wanders.
//   Pass 0 for both (your current globals) and behavior is identical to before.
void drive(double dis, double kd, double kp, double ki, double ekd, double ekp, double timeout) {
    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Record starting position, then drive relative to that
    // (inches_per_tick == PI/18000 per centidegree — same math as before)
    double start_pos = yrot.get_position() * inches_per_tick;
    double target_distance = dis;  // Distance to travel from current position
    
    double error = target_distance;
    double prev_error = error;

    double integral = 0;
    const double I_ZONE = 5.0;    // inches — only integrate when this close
    const double I_MAX  = 25.0;   // max ± percent from the I term
    
    double dist_derivative_1 = 0;
    double dist_derivative_2 = 0;
    double dist_derivative_3 = 0;
    double dist_derivative_4 = 0;
    double dist_derivative_5 = 0;
    double dist_derivative_6 = 0;
    double smoothed_derivative = 0;

    double current_tvel = 0;
    double slew_step = 3;

    // Heading hold: capture the heading we start at
    bool hold_heading = (ekp != 0.0 || ekd != 0.0);
    double target_heading = 0.0;
    double prev_h_error = 0.0;
    if (hold_heading) {
        if (odom_lock.take(50)) {
            target_heading = both;
            odom_lock.give();
        } else {
            hold_heading = false;   // couldn't read heading — drive without correction
        }
    }

    double starting_time = pros::millis();
    
    int time_settled = 0; 
    double settle_threshold = 1; 

    while (true) {
        // How far we've traveled since start
        double current_pos = yrot.get_position() * inches_per_tick;
        double distance_traveled = current_pos - start_pos;
        
        // Error is how much further we need to go
        error = target_distance - distance_traveled;

        dist_derivative_6 = dist_derivative_5;
        dist_derivative_5 = dist_derivative_4;
        dist_derivative_4 = dist_derivative_3;
        dist_derivative_3 = dist_derivative_2;
        dist_derivative_2 = dist_derivative_1;
        dist_derivative_1 = (error - prev_error) / 0.02; 
        smoothed_derivative = (dist_derivative_1 + dist_derivative_2 + dist_derivative_3 + 
                              dist_derivative_4 + dist_derivative_5 + dist_derivative_6) / 6;

        // ---- INTEGRAL (with anti-windup) ----
        if ((error > 0 && prev_error < 0) || (error < 0 && prev_error > 0)) {
            integral = 0;   // overshot the target — dump it
        }
        if (fabs(error) < I_ZONE) {
            integral += error * 0.02;                 // dt = 20 ms
            if (ki != 0) {
                double lim = fabs(I_MAX / ki);
                if (integral >  lim) integral =  lim;
                if (integral < -lim) integral = -lim;
            }
        } else {
            integral = 0;   // far from target — pure PD, no windup
        }

        prev_error = error;

        double target_vel = (error * kp) + (smoothed_derivative * kd) + (integral * ki);

        double step = target_vel - current_tvel;
        if (step > slew_step) {
            step = slew_step;
        } else if (step < -slew_step) {
            step = -slew_step;
        }
        current_tvel += step;

        // ---- HEADING HOLD (ekp/ekd) ----
        // Positive h_error means target heading is clockwise of current
        // → left faster, right slower (same convention as turn()).
        double correction = 0.0;
        if (hold_heading) {
            double curr_h = target_heading;           // fallback: no correction
            if (odom_lock.take(10)) {
                curr_h = both;
                odom_lock.give();
            }
            double h_error      = normalise_angle(target_heading - curr_h);
            double h_derivative = (h_error - prev_h_error) / 0.02;
            prev_h_error        = h_error;
            correction = (h_error * ekp) + (h_derivative * ekd);
        }

        left_mg.move_velocity(vperc(current_tvel + correction));
        right_mg.move_velocity(vperc(current_tvel - correction));

        // Exit conditions
        if ((pros::millis() - starting_time) > timeout * 1000) break;

        if (fabs(error) < settle_threshold) {
            time_settled += 1; 
        } else {
            time_settled = 0;   
        }

        if (time_settled > 5) break;

        pros::delay(20);
    }

    left_mg.move_velocity(0);
    right_mg.move_velocity(0);
}


// NEW SIGNATURE:
// cordon(x, y, dkd, dkp, dki, tkd, tkp, tki, theta, dtimeout, ttimeout,
//        exit_radius = 2.0, stop_at_end = true)
//
// Both internal loops are now full PIDs:
//  - drive integral only arms inside D_I_ZONE inches of the target
//    (distance is always positive here, so no sign-cross reset needed —
//    leaving the zone clears it instead)
//  - turn integral only arms inside T_I_ZONE degrees and resets when the
//    heading error crosses zero
// Also fixed: first-loop derivative kick (prev errors used to start at 0,
// so the very first D term was a huge spike).
void cordon(double target_x,  double target_y,
            double dkd,       double dkp,      double dki,
            double tkd,       double tkp,      double tki,
            double theta,
            double dtimeout,  double ttimeout,
            double exit_radius = 2.0,
            bool   stop_at_end = true)
{
    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 
    double prev_drive_error = 0.0;
    double prev_turn_error  = 0.0;
    double drive_integral   = 0.0;
    double turn_integral    = 0.0;
    bool   first_loop       = true;
    double start_time       = pros::millis();

    const double D_I_ZONE = 8.0;    // inches
    const double D_I_MAX  = 20.0;   // percent cap on drive I term
    const double T_I_ZONE = 15.0;   // degrees
    const double T_I_MAX  = 20.0;   // percent cap on turn I term
 
    while (true) {
 
        // ── 1. Read odometry ─────────────────────────────────────────────
        double curr_x, curr_y, curr_h;
        if (!odom_lock.take(50)) { pros::delay(10); continue; }
        curr_x = botx;
        curr_y = boty;
        curr_h = both;
        odom_lock.give();
 
        // ── 2. Vector to target ──────────────────────────────────────────
        double dx       = target_x - curr_x;
        double dy       = target_y - curr_y;
        double distance = std::hypot(dx, dy);
 
        // ── 3. Exit conditions ───────────────────────────────────────────
        if (distance < exit_radius)                          break;
        if ((pros::millis() - start_time) > dtimeout*1000)  break;
 
        // ── 4. DRIVE PID (how far left to go) ───────────────────────────
        double drive_error = distance;
        if (first_loop) prev_drive_error = drive_error;   // no first-loop D kick

        double drive_deriv = (drive_error - prev_drive_error) / 0.02;
        prev_drive_error   = drive_error;

        if (drive_error < D_I_ZONE) {
            drive_integral += drive_error * 0.02;
            if (dki != 0) {
                double lim = fabs(D_I_MAX / dki);
                if (drive_integral >  lim) drive_integral =  lim;
                if (drive_integral < -lim) drive_integral = -lim;
            }
        } else {
            drive_integral = 0;
        }

        double drive_power = (drive_error * dkp) + (drive_deriv * dkd)
                           + (drive_integral * dki);
        drive_power = std::max(-100.0, std::min(drive_power, 100.0));
 
        // ── 5. TURN PID (which way to face) ─────────────────────────────
        double math_angle      = std::atan2(dy, dx) * rad2deg;
        double desired_heading = normalise_angle(90.0 - math_angle);
        double turn_error      = normalise_angle(desired_heading - curr_h);

        if (first_loop) {                                  // no first-loop D kick
            prev_turn_error = turn_error;
            first_loop = false;
        }

        double turn_deriv = (turn_error - prev_turn_error) / 0.02;

        // crossed the desired heading? dump the integral
        if ((turn_error > 0 && prev_turn_error < 0) ||
            (turn_error < 0 && prev_turn_error > 0)) {
            turn_integral = 0;
        }
        prev_turn_error = turn_error;

        if (fabs(turn_error) < T_I_ZONE) {
            turn_integral += turn_error * 0.02;
            if (tki != 0) {
                double lim = fabs(T_I_MAX / tki);
                if (turn_integral >  lim) turn_integral =  lim;
                if (turn_integral < -lim) turn_integral = -lim;
            }
        } else {
            turn_integral = 0;
        }

        double turn_power = (turn_error * tkp) + (turn_deriv * tkd)
                          + (turn_integral * tki);
 
        // ── 6. Scale drive by heading alignment ──────────────────────────
        //   cos(0°)  = 1.0  → full speed when pointing at target
        //   cos(90°) = 0.0  → nearly stopped when sideways (let turn catch up)
        //   Clamped to 0.15 so the bot never fully stops spinning in place
        double heading_scale = std::cos(turn_error * deg2rad);
        heading_scale = std::max(0.15, heading_scale);
        drive_power  *= heading_scale;
 
        // ── 7. Mix into left/right and normalise ─────────────────────────
        //   Normalisation keeps the left/right ratio correct even when their
        //   sum would exceed 100% — turn is never clipped out accidentally
        double left_vel  = drive_power + turn_power;
        double right_vel = drive_power - turn_power;
        double max_val   = std::max(std::fabs(left_vel), std::fabs(right_vel));
        if (max_val > 100.0) {
            left_vel  = left_vel  / max_val * 100.0;
            right_vel = right_vel / max_val * 100.0;
        }
 
        left_mg.move_velocity(vperc(left_vel));
        right_mg.move_velocity(vperc(right_vel));
 
        pros::delay(20);
    }
 
    // Stop only when this is a deliberate stopping point
    if (stop_at_end) {
        left_mg.move_velocity(0);
        right_mg.move_velocity(0);
    }
 
    // Optional final heading turn (same as before, now passes tki through)
    if (theta != 0.0) {
        turn(theta, tkd, tkp, tki, ttimeout);
    }
}
 
//auton

void autonomous() {
    // Example calls with the new ki arguments:
    //   turn(90, 0.05, 3.5, /*tki*/ 5.0, 2.0);
    //   drive(24, kd, kp, /*ki*/ 3.0, ekd, ekp, 3.0);
    //   cordon(24, 24, dkd, dkp, /*dki*/ 3.0, tkd, tkp, /*tki*/ 5.0, 0, 3.0, 1.5);
}