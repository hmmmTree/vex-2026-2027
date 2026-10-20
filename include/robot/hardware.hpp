#pragma once

#include "api.h"
#include "pros/adi.hpp"

namespace robot {

constexpr int LIFT_ROT_PORT = 3; //placeholder for now, will be changed when we know what port the lift is on
constexpr int IMU_PORT1      = 17;
constexpr int IMU_PORT2      = 18; //placeholder for now, will be changed when we know what port the second IMU is on
constexpr int XROT_PORT     = 6; //placeholder: was 18, which the left drive uses
constexpr int YROT_PORT     = 1; //placeholder: was 19, which the left drive uses
constexpr int AIVISION_PORT = 13; //placeholder: was 20, whichthe left drive uses
inline constexpr std::array<std::int8_t, 3> LEFT_PORTS  = { -11,   -12,   -13};
inline constexpr std::array<std::int8_t, 3> RIGHT_PORTS = {18, 19, 20};
constexpr int TOP_ROLLER     = -1; //placeholder for now, will be changed when we know what port the top roller is on
constexpr int BOTTOM_ROLLER  = -6; //placeholder for now, will be changed when we know what port the bottom roller is on
// {full motor, half (5.5W) motor}; the half motor is reversed
inline constexpr std::array<std::int8_t, 2> LIFT_PORTS = {-5, -4}; //placeholder for now, will be changed when we know what ports the lift is on


constexpr char CLAW_FLIP     = 'D'; //placeholder for now, will be changed when we know what port the claw is on
constexpr char FLIPPER_CLAW  = 'B'; //placeholder for now, will be changed when we know what port the flipper claw is on
constexpr char CLAW          = 'A'; //placeholder for now, will be changed when we know what port the claw is on
constexpr char INTAKE_SLIDER = 'C'; //placeholder for now, will be changed when we know what port the intake is on


class Hardware {
public:
    Hardware();

    Hardware(const Hardware&)            = delete;
    Hardware& operator=(const Hardware&) = delete;


    void configure();

    pros::Controller master;

    pros::MotorGroup left;
    pros::MotorGroup right;

    pros::Imu inertial1;
    pros::Imu inertial2;

    pros::Rotation xrot;
    pros::Rotation yrot;
    pros::Rotation lift_r;

    pros::AIVision aivision;

    
    pros::Motor top_roller;
    pros::Motor bottom_roller;
    pros::MotorGroup lift;

    pros::adi::Pneumatics claw_flip;
    pros::adi::Pneumatics flipper_claw;
    pros::adi::Pneumatics intake_slider;
    pros::adi::Pneumatics claw;
};

}
