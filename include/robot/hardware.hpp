#pragma once

#include "api.h"

namespace robot {

constexpr int IMU_PORT      = 17;
constexpr int XROT_PORT     = 2;
constexpr int YROT_PORT     = 3;
constexpr int AIVISION_PORT = 5;


class Hardware {
public:
    Hardware();

    Hardware(const Hardware&)            = delete;
    Hardware& operator=(const Hardware&) = delete;


    void configure();

    pros::Controller master;
    pros::MotorGroup left;
    pros::MotorGroup right;
    pros::Imu        inertial;
    pros::Rotation   xrot;
    pros::Rotation   yrot;
    pros::AIVision   aivision;
};

}
