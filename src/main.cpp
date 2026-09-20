#include "main.h"

#include "pros/rtos.hpp"
#include "robot/robot.hpp"

#include <cstddef>

using namespace robot;

namespace {

Robot& bot() { return Robot::instance(); }

constexpr double START_HEADING = 0.0;

constexpr double CM_TO_IN = 1.0 / 2.54;
constexpr double LEG_CM   = 10.0;
//constexpr double LEG_IN   = LEG_CM * CM_TO_IN;

// {kp, ki, kd}
constexpr PidGains DRIVE_GAINS{4.0, 0.0, 4.0}; //tuned im gonna kill u if u touch this

// as it did before.
constexpr PidGains CORDON_TURN_GAINS{0.6, 0.0, 5}; //tuned im gonna kill u if u touch this
constexpr PidGains TURN_GAINS       {0.6, 0.0, 1.25}; //tuned im gonna kill u if u touch this

constexpr PidGains HEADING_HOLD_GAINS{0.0, 0.0, 0.0};

// A pass-through point hands off to the next leg as soon as the robot is this
// close, so the distance PID never reaches the bottom of its ramp: speed at the
// handoff is roughly drive kp x this radius. It is also how far the robot is
// allowed to miss the point by, so bigger is not free.
constexpr double WAYPOINT_RADIUS = 6.0;
constexpr double FINAL_RADIUS    = 2.0;

constexpr double BOOMERANG_LEAD = 0.5; //tuned im gonna kill u if u touch this

constexpr double DRIVE_TIMEOUT = 8.0;
constexpr double TURN_TIMEOUT  = 1.0;

constexpr double MACRO_DISTANCE_IN = 12.0;

constexpr double MOTOR_TEMP_LIMIT_C = 55.0;

template <std::size_t N>
MotionResult run_path(Drivetrain& drivetrain, const CordonRequest (&points)[N]) {
    MotionResult result = MotionResult::Settled;
    for (std::size_t i = 0; i < N; i++) {
        result = drivetrain.cordon(points[i]);
        if (result != MotionResult::Settled) break;
    }
    return result;
}

void run_interruptible_macro() {
    Robot&      robot      = bot();
    Interrupts& interrupts = robot.interrupts();

    interrupts.clear();
    interrupts.arm();

    ScopedInterrupt takeover(interrupts, "driver takeover",
                             triggers::driver_takeover(robot.hardware().master));

    const MotionResult result = robot.drivetrain().drive_distance(
        MACRO_DISTANCE_IN, DRIVE_GAINS, HEADING_HOLD_GAINS, DRIVE_TIMEOUT);

    robot.diagnostics().show_result("macro", result, interrupts);
}

}

void initialize() {
    pros::lcd::initialize();

    Robot& robot = bot();

    robot.hardware().configure();
    robot.hardware().inertial.reset(true);
    pros::delay(250);

    

    robot.odometry().start();
    pros::delay(750);

    robot.odometry().set_pose(x_int, y_int, START_HEADING, -1.0);
    robot.vision().initial_fix(START_HEADING, 15);


    
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
    Robot&      robot      = bot();
    Interrupts& interrupts = robot.interrupts();

    interrupts.clear();
    interrupts.arm();

    ScopedInterrupt overheating(interrupts, "drive too hot",
                                triggers::motors_over_temp(robot.hardware().left,
                                                           MOTOR_TEMP_LIMIT_C));
    ScopedInterrupt overheating_right(interrupts, "drive too hot",
                                       triggers::motors_over_temp(robot.hardware().right,
                                                                  MOTOR_TEMP_LIMIT_C));
    ScopedInterrupt takeover(interrupts, "driver takeover",
                             triggers::driver_takeover(robot.hardware().master));

/*
const MotionResult turn_result =
        robot.drivetrain().turn_to(20, TURN_GAINS, TURN_TIMEOUT);
        pros::delay(500);
    robot.diagnostics().show_result("auton", turn_result, interrupts);
//*/



// ===== src/main.cpp, in the anonymous namespace =====================
constexpr PidGains DRIVE_GAINS      {6.00, 0.00, 4.00};
constexpr PidGains CORDON_TURN_GAINS{0.60, 0.00, 5.00};
constexpr PidGains TURN_GAINS       {0.60, 0.00, 1.25};


// ===== src/main.cpp, in initialize() ================================
    robot.odometry().set_pose(0.0, -61.0, 0.0, -1.0);

// ===== src/main.cpp, in autonomous() ================================
    const CordonRequest path[] = {
        {
            .x                = 15.5,
            .y                = -54.2,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = 1.75,
            .stop_at_end      = false,
            .turn_at_end      = false,
            .final_heading    = 0.0,
            .lead             = BOOMERANG_LEAD,
        },
        /*
        {
            .x                = -9.5,
            .y                = -54.2,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = 3.50,
            .stop_at_end      = false,
            .turn_at_end      = false,
            .final_heading    = 100.0,
            .lead             = 0.000,
        },
        {
            .x                = 16.9,
            .y                = -56.0,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = 1.00,
            .stop_at_end      = true,
            .turn_at_end      = false,
            .final_heading    = 45.0,
            .lead             = BOOMERANG_LEAD,
        },
        {
            .x                = 16.3,
            .y                = -30.9,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = FINAL_RADIUS,
            .stop_at_end      = false,
            .turn_at_end      = false,
            .final_heading    = 200.0,
            .lead             = BOOMERANG_LEAD,
            .reverse          = true,
        },
        {
            .x                = -12.1,
            .y                = -47.0,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = 1.00,
            .stop_at_end      = true,
            .turn_at_end      = false,
            .final_heading    = 270.0,
            .lead             = BOOMERANG_LEAD,
        },
        {
            .x                = -19.1,
            .y                = -27.5,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = FINAL_RADIUS,
            .stop_at_end      = true,
            .turn_at_end      = false,
            .final_heading    = 135.0,
            .lead             = BOOMERANG_LEAD,
            .reverse          = true,
        },
        {
            .x                = -16.4,
            .y                = -40.1,
            .drive_gains      = DRIVE_GAINS,
            .turn_gains       = CORDON_TURN_GAINS,
            .final_turn_gains = TURN_GAINS,
            .drive_timeout    = DRIVE_TIMEOUT,
            .exit_radius      = 1.00,
            .stop_at_end      = true,
            .turn_at_end      = false,
            .final_heading    = 225.0,
            .lead             = BOOMERANG_LEAD,
        },
        */
    };


    const MotionResult result = run_path(robot.drivetrain(), path);

    robot.drivetrain().stop();
    robot.diagnostics().show_result("auton", result, interrupts);
  
/*   
    const MotionResult result =
        robot.drivetrain().drive_distance(24, DRIVE_GAINS, HEADING_HOLD_GAINS, DRIVE_TIMEOUT);
        pros::delay(500);
        robot.drivetrain().turn_to(90, TURN_GAINS, TURN_TIMEOUT);
    robot.diagnostics().show_result("auton", result, interrupts);
*/    
}

void opcontrol() {
    Robot&            robot      = bot();
    Drivetrain&       drivetrain = robot.drivetrain();
    pros::Controller& master     = robot.hardware().master;

    drivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    double speed        = 1.0;
    int    diag_counter = 0;

    while (true) {
        const int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int rotate  = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        drivetrain.arcade(forward, rotate, speed);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP))    speed = 1.0;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) speed = 0.4;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) speed = speed*-1.0;

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            run_interruptible_macro();
            drivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }

        

        if (++diag_counter >= 12) {
            diag_counter = 0;
            robot.diagnostics().show_drive(forward, rotate, speed);
        }
        

        pros::delay(LOOP_INTERVAL_MS);
    }
}
