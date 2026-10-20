#pragma once

#include "robot/robot.hpp"

#include <cstddef>

namespace robot {

// ---- tuning shared by autons and driver macros -----------------------------

// {kp, ki, kd}
inline constexpr PidGains DRIVE_GAINS       {4.0, 0.0, 4.0};   //tuned im gonna kill u if u touch this
inline constexpr PidGains CORDON_TURN_GAINS {0.6, 0.0, 5.0};   //tuned im gonna kill u if u touch this
inline constexpr PidGains TURN_GAINS        {0.6, 0.0, 1.25};  //tuned im gonna kill u if u touch this
inline constexpr PidGains HEADING_HOLD_GAINS{0.0, 0.0, 0.0};

// {kp, ki, kd, kG}
inline constexpr LiftGains LIFT_GAINS{0.0, 0.0, 0.0, 600};

// A pass-through point hands off to the next leg as soon as the robot is this
// close, so the distance PID never reaches the bottom of its ramp: speed at the
// handoff is roughly drive kp x this radius. It is also how far the robot is
// allowed to miss the point by, so bigger is not free.
inline constexpr double WAYPOINT_RADIUS = 6.0;
inline constexpr double FINAL_RADIUS    = 2.0;

inline constexpr double BOOMERANG_LEAD = 0.5;  //tuned im gonna kill u if u touch this

inline constexpr double DRIVE_TIMEOUT = 8.0;
inline constexpr double TURN_TIMEOUT  = 1.0;
inline constexpr double LIFT_TIMEOUT  = 1.5;

// ---- selection --------------------------------------------------------------

enum class Alliance { Red, Blue };

// Which way the yellow pins are set up at the start of the match, as seen by
// the alliance's own drivers.
enum class YellowPos { Left, Right };

// Every path is drawn in path.jerryio for this alliance with the yellow pins on
// the LEFT. The three other cases are made from it automatically.
inline constexpr Alliance DRAWN_ALLIANCE = Alliance::Red;

// The field (field.png) is cut diagonally: red owns the bottom-left half, blue
// the top-right, split by the double line from top-left to bottom-right.
//
// Yellow RIGHT is yellow LEFT reflected across the other diagonal, the line
// x = y from bottom-left to top-right. That keeps the robot in its own half
// and swaps x and y:
//     (x, y) -> (y, x)      heading h -> 90 - h
// Every game element on red's side lands on a matching one (red 2 <-> red 3,
// yellow 1 <-> yellow 4, crosses and wall goals onto each other). The corner
// zones do NOT: they run along the left wall, and the swap lays them along the
// bottom wall instead. A point inside a corner zone needs its own branch.
//
// The other alliance is the same field turned half way round (red 2/3 bottom
// left land on blue 2/3 top right, red 1/4 yellows on blue 1/4):
//     (x, y) -> (-x, -y)    heading h -> h + 180
// Turning the field round does not change which way the pins look from that
// alliance's drivers, so blue LEFT is red LEFT turned round, blue RIGHT is red
// RIGHT turned round.
//
// Headings are compass style throughout: 0 = +y (up the screen in
// path.jerryio), 90 = +x, clockwise positive.

const char* alliance_name(Alliance alliance);
const char* yellow_name(YellowPos yellow);

// What a path function gets handed. Write every path for DRAWN_ALLIANCE with
// yellow on the left; start_at() and run() convert it to whatever is selected.
struct AutonContext {
    Robot&    robot;
    Alliance  alliance;
    YellowPos yellow;   // as picked on the brain screen

    // Seeds odometry with the starting pose as drawn, converted.
    void start_at(double x, double y, double heading) const;

    // A point as drawn, converted to the selected alliance and yellow side.
    CordonRequest as_driven(CordonRequest point) const;

    // Sits still for `seconds`, but returns Interrupted straight away if an
    // interrupt (stick moved, motors hot) fires. Use it instead of pros::delay.
    MotionResult wait(double seconds) const;

    // Drives a path array straight out of path.jerryio, converted, running
    // each point's on_start / at_end / pause_after (see CordonRequest).
    // Stops at the first point that does not settle and returns why; at_end
    // and the pause are skipped for that point.
    template <std::size_t N>
    MotionResult run(const CordonRequest (&points)[N]) const {
        MotionResult result = MotionResult::Settled;
        for (std::size_t i = 0; i < N; i++) {
            const CordonRequest& point = points[i];
            if (point.on_start) point.on_start();

            result = robot.drivetrain().cordon(as_driven(point));
            if (result != MotionResult::Settled) break;

            if (point.at_end) point.at_end();
            if (point.pause_after > 0.0) {
                robot.drivetrain().stop();
                result = wait(point.pause_after);
                if (result != MotionResult::Settled) break;
            }
        }
        robot.drivetrain().stop();
        dump_cordon_trace();
        return result;
    }
};

using AutonFn = MotionResult (*)(const AutonContext&);

struct Auton {
    const char* name;  // keep it short, it goes on the brain screen
    AutonFn     run;
};

// Brain screen buttons: left = alliance, centre = yellow side, right = path.
// Call once from initialize(); the selection stays put until changed.
void auton_selector_start();

// Runs whatever the selector currently shows.
MotionResult run_selected_auton(Robot& robot);

// Name of the selected path, for diagnostics.
const char* selected_auton_name();

// Turns 90 / 0 / 180 / 0 with TURN_GAINS
// and leaves each turn's error on brain lines 3-6.
MotionResult run_turn_test(Robot& robot);

// True while the last auton's results (e.g. tune offsets) are on brain lines
// 3-6; opcontrol leaves those lines alone until the next auton starts.
bool auton_screen_held();

}
