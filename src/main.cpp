#include "main.h"

#include "pros/rtos.hpp"
#include "robot/robot.hpp"

using namespace robot;

namespace {

Robot& bot() { return Robot::instance(); }

constexpr double START_HEADING = 0.0;

constexpr double CM_TO_IN = 1.0 / 2.54;
constexpr double LEG_CM   = 10.0;
//constexpr double LEG_IN   = LEG_CM * CM_TO_IN;

// {kp, ki, kd}
constexpr PidGains DRIVE_GAINS{5.0, 0.0, 0.5};

// as it did before.
constexpr PidGains CORDON_TURN_GAINS{3.5, 0.0, 0.05};
constexpr PidGains TURN_GAINS       {3.5, 0.0, 0.001};

constexpr PidGains HEADING_HOLD_GAINS{0.0, 0.0, 0.0};

constexpr double WAYPOINT_RADIUS = 1.0;
constexpr double FINAL_RADIUS    = 0.5;

constexpr double BOOMERANG_LEAD = 0.5;

constexpr double DRIVE_TIMEOUT = 3.0;
constexpr double TURN_TIMEOUT  = 2.0;

constexpr double MACRO_DISTANCE_IN = 12.0;

constexpr double MOTOR_TEMP_LIMIT_C = 55.0;

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

    robot.odometry().start();

    robot.vision().begin();
    pros::delay(300);

    robot.odometry().set_pose(x_int, y_int, START_HEADING, -1.0);
    robot.vision().initial_fix(START_HEADING, 15);

    pros::lcd::print(1, "ROBOT READY");


    
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
    const CordonRequest waypoint{
        .x                = 0.0,
        .y                = 10,
        .drive_gains      = DRIVE_GAINS,
        .turn_gains       = CORDON_TURN_GAINS,
        .drive_timeout    = DRIVE_TIMEOUT,
        .exit_radius      = WAYPOINT_RADIUS,
        .stop_at_end      = false, 
    };

    const CordonRequest finish{
        .x             = 10,
        .y             = 10,
        .drive_gains   = DRIVE_GAINS,
        .turn_gains    = CORDON_TURN_GAINS,
        .drive_timeout = DRIVE_TIMEOUT,
        .exit_radius   = FINAL_RADIUS,
        .stop_at_end   = true,
        .final_heading = 90.0,
        .lead          = BOOMERANG_LEAD,
    };



    MotionResult result = robot.drivetrain().cordon(waypoint);
    if (!interrupts.tripped()) {
        result = robot.drivetrain().cordon(finish);
    }

    robot.drivetrain().stop();
    robot.diagnostics().show_result("auton", result, interrupts);
*/  
    
    robot.drivetrain().drive_distance(24, DRIVE_GAINS, HEADING_HOLD_GAINS, DRIVE_TIMEOUT);
    //pros::delay(500);
    //robot.drivetrain().turn_to(90, TURN_GAINS, TURN_TIMEOUT);
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
