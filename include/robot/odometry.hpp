#pragma once

#include "api.h"
#include "robot/config.hpp"
#include "robot/hardware.hpp"

#include <atomic>
#include <cstdint>
#include <memory>

namespace robot {

struct Pose {
    double x       = 0.0;
    double y       = 0.0;
    double heading = 0.0;
};

// Wraps an angle in degrees into (-180, 180].
double normalise_angle(double angle);

class Odometry {
public:
    explicit Odometry(Hardware& hardware, OdometryConfig config = {});
    ~Odometry();

    Odometry(const Odometry&)            = delete;
    Odometry& operator=(const Odometry&) = delete;

    // Resets the trackers and spawns the update task. Idempotent.
    void start();

    // Asks the task to finish its current tick, then joins it
    void stop();

    bool running() const { return task_ != nullptr; }

    // False if the pose mutex could not be taken in time; `out` is untouched
    [[nodiscard]] bool read(Pose& out, std::uint32_t timeout_ms = 50) const;

    // Moves the estimate without disturbing the heading, for a vision fix
    bool set_translation(double x, double y, std::uint32_t timeout_ms = 50);

    // Seeds the pose and the IMU-to-field heading offset.
    void set_pose(double x, double y, double heading, double heading_side);

    // Field heading read straight off the IMU, bypassing the pose mutex. NaN if
    // the IMU is not answering Use it in lops that only need a heading and
    // should not contend with the update task
    double imu_heading() const;

    double heading_offset() const;
    double heading_side() const;

    OdometryConfig config() const;
    void           set_config(OdometryConfig config);

private:
    static void task_entry(void* self);
    void        run();

    Hardware&           hw_;
    mutable pros::Mutex lock_;

    OdometryConfig config_;
    double         x_              = 0.0;
    double         y_              = 0.0;
    double         heading_        = 0.0;
    double         heading_offset_ = 0.0;
    double         heading_side_   = 1.0;

    std::atomic<bool>           stop_requested_{false};
    std::unique_ptr<pros::Task> task_;
};

}
