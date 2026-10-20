#include "main.h"

#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robot/autons.hpp"
#include "robot/lift.hpp"
#include "robot/robot.hpp"

#include <cstddef>

using namespace robot;

namespace {

Robot& bot() { return Robot::instance(); }

constexpr double START_HEADING = 0.0;

constexpr double CM_TO_IN = 1.0 / 2.54;
constexpr double LEG_CM   = 10.0;
//constexpr double LEG_IN   = LEG_CM * CM_TO_IN;

// Gains, radii and timeouts live in robot/autons.hpp so the paths can use them too.

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

// Runs whatever the brain-screen selector shows, with the same safety cut-offs
// as a match. Shared by autonomous() and the controller's LEFT arrow; R1 passes
// the turn test instead.
void run_auton(MotionResult (*routine)(Robot&) = run_selected_auton,
               const char* name = nullptr) {
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

    // Pick alliance / yellow position / path on the brain screen before the
    // match. The paths themselves are in src/autons.cpp.
    const MotionResult result = routine(robot);
    robot.drivetrain().stop();
    robot.diagnostics().show_result(name ? name : selected_auton_name(), result, interrupts);
}

}

void initialize() {
    pros::lcd::initialize();

    Robot& robot = bot();

    robot.hardware().configure();
    // Start both calibrating, then wait for both, so the second does not add
    // another ~2 s on top of the first.
    robot.hardware().inertial1.reset(false);
    robot.hardware().inertial2.reset(false);
    double timeout = 0;
    while (robot.hardware().inertial1.is_calibrating() == 1 ||
           robot.hardware().inertial2.is_calibrating() == 1 || timeout < 3000) {
        pros::delay(10);
        timeout += 10;
    }
    pros::delay(250);

    

    robot.odometry().start();
    pros::delay(750);

    robot.odometry().set_pose(x_int, y_int, START_HEADING, -1.0);
    robot.vision().initial_fix(START_HEADING, 15);

    auton_selector_start();


    
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
    run_auton();
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

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))   robot.lift().up(1000);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) robot.lift().down(1000);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) if (speed == 1) speed = 0.5; else speed = 1.0;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) robot.lift().clawtoggle();

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) drivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) robot.lift().drop(LIFT_GAINS, 2);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) robot.lift().toggle(LIFT_GAINS, 2);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) robot.intake().toggle();

        
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) robot.intake().run(1000);
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) robot.intake().run(-1000);
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) robot.lift().grippertoggle();
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) robot.lift().armtoggle();

        /*
        // Practice without a competition switch: runs the selected auton.
        // Touching a stick stops it and hands control back.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            run_auton();
            drivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            run_auton(run_turn_test, "turn test");
            drivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }
*/

        

        if (++diag_counter >= 12) {
            diag_counter = 0;
            if (!auton_screen_held()) robot.diagnostics().show_drive(forward, rotate, speed);
        }


        pros::delay(LOOP_INTERVAL_MS);
    }
}
