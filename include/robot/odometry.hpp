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

    // Raw heading from both IMUs, in degrees [0, 360). Averages them while they
    // agree. Ignores one that is unplugged, calibrating, jumped faster than the
    // robot can turn, or has drifted away from the other, and keeps ignoring it
    // until it agrees again. NaN only when neither can be trusted.
    double fused_imu_raw() const;

    // Bit 0 = inertial1 used last read, bit 1 = inertial2
    std::uint8_t imus_in_use() const { return imu_used_.load(); }

    double heading_offset() const;
    double heading_side() const;

    OdometryConfig config() const;
    void           set_config(OdometryConfig config);

    double lift_height() const;          // inches, takes the lock
    double lift_stage() const;
    void   reset_lift_height(double h = 0.0);
    void set_lift_stage(double stage);

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

    // Per-IMU history for fused_imu_raw(), guarded by imu_lock_ (separate from
    // lock_ so the turn loop never waits on the odometry task for a heading)
    struct ImuTrack {
        double        last    = 0.0;
        std::uint32_t last_ms = 0;
        bool          seen    = false;  // has given a good reading since it last dropped out
        bool          faulted = false;  // ignored until it agrees with the other IMU again
    };
    mutable pros::Mutex          imu_lock_;
    mutable ImuTrack             imu_track_[2];
    mutable double               last_fused_      = 0.0;
    mutable bool                 have_last_fused_ = false;
    mutable std::atomic<std::uint8_t> imu_used_{0};

    std::atomic<bool>           stop_requested_{false};
    std::unique_ptr<pros::Task> task_;

    double lift_height_ = 0.0;
    double lift_zero_deg_ = 0.0;
    double lift_stage_ = 0.0;
};

}
