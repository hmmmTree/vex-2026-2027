#pragma once

#include "api.h"

namespace robot {

constexpr int IMU_PORT      = 17;
constexpr int XROT_PORT     = 20;
constexpr int YROT_PORT     = 19;
constexpr int AIVISION_PORT = 5;
inline constexpr std::array<std::int8_t, 3> LEFT_PORTS  = { 2,   -3,   -4};
inline constexpr std::array<std::int8_t, 3> RIGHT_PORTS = {11, -12, 13};


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
