#pragma once

#include "api.h"

namespace robot {

// ---------------------------------------------------------------------------
// V5 brain port map (rewired for the 2026-27 robot)
//
// The previous build had the IMU on port 19 and the X tracker on port 20,
// both of which were also drive motors, so the sensors returned PROS_ERR and
// odometry was garbage. Every device was moved to a free port at the same
// time; this table is the single source of truth for where things plug in.
//
//   Port  Device                    Notes
//   ----  ------------------------  -------------------------------------
//    1    Left drive motor  (front) forward
//    6    Left drive motor  (mid)   forward
//    9    Left drive motor  (back)  forward
//   18    Right drive motor (front) reversed
//   19    Right drive motor (mid)   reversed
//   20    Right drive motor (back)  reversed
//   17    Inertial sensor (IMU)
//    2    X tracking wheel (Rotation, sideways)
//    3    Y tracking wheel (Rotation, forward, reversed in configure())
//    5    AI Vision sensor
//
// Motor ports are given as signed values to pros::MotorGroup: a negative
// port means the motor is reversed. The whole right side is mirrored, so all
// three right motors are reversed and all three left motors are forward.
// If a motor is ever remounted the other way round, flip its sign here.
// ---------------------------------------------------------------------------

constexpr int LEFT_FRONT_PORT  = 1;
constexpr int LEFT_MID_PORT    = 6;
constexpr int LEFT_BACK_PORT   = 9;
constexpr int RIGHT_FRONT_PORT = -18;
constexpr int RIGHT_MID_PORT   = -19;
constexpr int RIGHT_BACK_PORT  = -20;

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
