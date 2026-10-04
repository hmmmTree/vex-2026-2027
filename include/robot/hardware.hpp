#pragma once

#include "api.h"
#include "pros/adi.hpp"

namespace robot {

constexpr int LIFT_ROT_PORT = 3; //placeholder for now, will be changed when we know what port the lift is on
constexpr int IMU_PORT      = 17;
constexpr int XROT_PORT     = 11; //placeholder: was 18, which the left drive uses
constexpr int YROT_PORT     = 12; //placeholder: was 19, which the left drive uses
constexpr int AIVISION_PORT = 13; //placeholder: was 20, which the left drive uses
inline constexpr std::array<std::int8_t, 3> LEFT_PORTS  = { -20,   -19,   -18};
inline constexpr std::array<std::int8_t, 3> RIGHT_PORTS = {10, 9, 8};
constexpr int TOP_ROLLER     = 1; //placeholder for now, will be changed when we know what port the top roller is on
constexpr int BOTTOM_ROLLER  = 2; //placeholder for now, will be changed when we know what port the bottom roller is on
constexpr int LIFT_PORT      = 5; //placeholder for now, will be changed when we know what port the lift is on


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

    pros::Imu inertial;

    pros::Rotation xrot;
    pros::Rotation yrot;
    pros::Rotation lift_r;

    pros::AIVision aivision;

    
    pros::Motor top_roller;
    pros::Motor bottom_roller;
    pros::Motor lift;

    pros::adi::Pneumatics claw_flip;
    pros::adi::Pneumatics flipper_claw;
    pros::adi::Pneumatics intake_slider;
    pros::adi::Pneumatics claw;
};

}
