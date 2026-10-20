#include "robot/lift.hpp"
#include "robot/config.hpp"
#include "robot/hardware.hpp"
#include "robot/odometry.hpp"
#include "robot/pid.hpp"
#include "robot/interrupt.hpp"
#include <cmath>
#include "robot/diagnostics.hpp"
#include "robot/config.hpp"


namespace robot {


    namespace {
    constexpr IntegralLimits LIFT_INTEGRAL{1.0, 2000.0};
    constexpr double LIFT_SETTLE_IN = 0.5;
    }  
    
    Lift::Lift(Hardware& hw, Odometry& odometry) : hw_(hw), odometry_(odometry) {}
    
    void Lift::up(double speed) {
        hw_.lift.move_velocity(speed);
    }
    void Lift::down(double speed) {
        hw_.lift.move_velocity(-speed);
    }
    
    void Lift::toggle(LiftGains gains, double timeout) {
    int current_stage = odometry_.lift_stage();
    int stage = current_stage + 1;
    if (stage > 8) stage = 0; // wrap around to stage need to change tho
    Pid controller({gains.kp, gains.ki, gains.kd}, LIFT_INTEGRAL);   // create the PID
    controller.set_output_limits(-12000, 12000);

    const double  target_height = stage * LIFT_IN_PER_STAGE;
    const std::uint32_t start_time = pros::millis();

    while (true) {
        const double error = target_height - odometry_.lift_height();
        if (std::fabs(error) < LIFT_SETTLE_IN) break;
        if (pros::millis() - start_time >= timeout * 1000) break;

        const double power = controller.update(error) + gains.kG;
        hw_.lift.move_voltage(static_cast<int>(power));

        pros::delay(LOOP_INTERVAL_MS);
    }
    hw_.lift.move_voltage(static_cast<int>(gains.kG));   // hold position
    odometry_.set_lift_stage(stage);
}

    void Lift::drop(LiftGains gains, double timeout) {
        Pid controller({gains.kp, gains.ki, gains.kd}, LIFT_INTEGRAL);   // create the PID
        controller.set_output_limits(-12000, 12000);

        const double  target_height = 0;
        const std::uint32_t start_time = pros::millis();

        while (true) {
            const double error = target_height - odometry_.lift_height();
            if (std::fabs(error) < LIFT_SETTLE_IN) break;
            if (pros::millis() - start_time >= timeout * 1000) break;

            const double power = controller.update(error) + gains.kG;
            hw_.lift.move_voltage(static_cast<int>(power));

            pros::delay(LOOP_INTERVAL_MS);
        }
        hw_.lift.move_voltage(static_cast<int>(gains.kG));   // hold position
    }

    void Lift::armtoggle() {
        if (hw_.claw_flip.is_extended() == true) {
            hw_.claw_flip.retract();
        } else {
            hw_.claw_flip.extend();
        }
    }
    void Lift::clawtoggle() {
        if (hw_.flipper_claw.is_extended() == true) {
            hw_.flipper_claw.retract();
        } else {
            hw_.flipper_claw.extend();
        }
    } 
    void Lift::grippertoggle() {
        if (hw_.claw.is_extended() == true) {
            hw_.claw.retract();
        } else {
            hw_.claw.extend();
        }
    }
}



