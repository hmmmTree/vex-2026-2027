#include "robot/odometry.hpp"

#include <cmath>
#include <limits>

namespace robot {

namespace {

// Releases an already-taken mutex however the scope is left, so an early return
// can never strand the update task. take() stays explicit because every caller
// here wants to choose its own timeout and handle failure.
class ReleaseOnExit {
public:
    explicit ReleaseOnExit(pros::Mutex& mutex) : mutex_(mutex) {}
    ~ReleaseOnExit() { mutex_.give(); }

    ReleaseOnExit(const ReleaseOnExit&)            = delete;
    ReleaseOnExit& operator=(const ReleaseOnExit&) = delete;

private:
    pros::Mutex& mutex_;
};

constexpr int DISPLAY_EVERY_TICKS = 10;

// A rotation sensor reports failure as PROS_ERR, which is INT32_MAX and so
// stays perfectly finite once it becomes a double. It has to be checked by
// value, or a dead port reads as a few hundred thousand inches of travel.
bool read_ticks(const pros::Rotation& sensor, double& out) {
    const std::int32_t raw = sensor.get_position();
    if (raw == PROS_ERR) return false;
    out = static_cast<double>(raw);
    return true;
}

}

double normalise_angle(double angle) {
    angle = std::fmod(angle, 360.0);
    if (angle > 180.0)  angle -= 360.0;
    if (angle < -180.0) angle += 360.0;
    return angle;
}

Odometry::Odometry(Hardware& hardware, OdometryConfig config)
    : hw_(hardware), config_(config) {}

Odometry::~Odometry() {
    stop();
}

void Odometry::start() {
    if (task_) return;

    hw_.xrot.reset_position();
    hw_.yrot.reset_position();

    stop_requested_.store(false);
    task_ = std::make_unique<pros::Task>(&Odometry::task_entry, this, "odometry");
}

void Odometry::stop() {
    if (!task_) return;

    stop_requested_.store(true);
    task_->join();
    task_.reset();
}

void Odometry::task_entry(void* self) {
    static_cast<Odometry*>(self)->run();
}

bool Odometry::read(Pose& out, std::uint32_t timeout_ms) const {
    if (!lock_.take(timeout_ms)) return false;
    ReleaseOnExit guard(lock_);

    out.x       = x_;
    out.y       = y_;
    out.heading = heading_;
    return true;
}

bool Odometry::set_translation(double x, double y, std::uint32_t timeout_ms) {
    if (!lock_.take(timeout_ms)) return false;
    ReleaseOnExit guard(lock_);

    x_ = x;
    y_ = y;
    return true;
}

void Odometry::set_pose(double x, double y, double heading, double heading_side) {
    double imu_now = hw_.inertial.get_heading();
    if (!std::isfinite(imu_now)) imu_now = 0.0;

    while (!lock_.take(50)) {
        pros::lcd::print(0, "Set Pose: Mutex timeout!");
    }
    {
        ReleaseOnExit guard(lock_);
        x_              = x;
        y_              = y;
        heading_side_   = heading_side;
        heading_offset_ = normalise_angle(heading - imu_now);
        heading_        = normalise_angle(heading);
    }

    pros::lcd::print(0, "POSE SET: %.2f %.2f", x, y);
}

double Odometry::imu_heading() const {
    const double raw = hw_.inertial.get_heading();
    if (!std::isfinite(raw)) return std::numeric_limits<double>::quiet_NaN();
    return normalise_angle(raw + heading_offset());
}

double Odometry::heading_offset() const {
    if (!lock_.take(TIMEOUT_MAX)) return 0.0;
    ReleaseOnExit guard(lock_);
    return heading_offset_;
}

double Odometry::heading_side() const {
    if (!lock_.take(TIMEOUT_MAX)) return 1.0;
    ReleaseOnExit guard(lock_);
    return heading_side_;
}

OdometryConfig Odometry::config() const {
    if (!lock_.take(TIMEOUT_MAX)) return OdometryConfig{};
    ReleaseOnExit guard(lock_);
    return config_;
}

void Odometry::set_config(OdometryConfig config) {
    if (!lock_.take(TIMEOUT_MAX)) return;
    ReleaseOnExit guard(lock_);
    config_ = config;
}

void Odometry::run() {
    double prev_x      = 0.0;
    double prev_y      = 0.0;
    double prev_raw_h  = 0.0;
    bool   have_prev_h = false;
    int    display_counter = 0;

    while (!stop_requested_.load()) {
        // Device reads happen outside the lock they talk to the V5 brain and
        // must not hold up a control loop that only wants the last pose.
        // Every path out of this iteration must delay, or the task spins and
        // starves the rest of the system including the brain screen.
        pros::delay(LOOP_INTERVAL_MS);

        // Decided before any early exit, so a tick that cannot produce a pose
        // still gets to say so instead of leaving the line frozen.
        const bool display_due = ++display_counter >= DISPLAY_EVERY_TICKS;
        if (display_due) display_counter = 0;

        double curr_x = 0.0;
        double curr_y = 0.0;
        const bool x_ok = read_ticks(hw_.xrot, curr_x);
        const bool y_ok = read_ticks(hw_.yrot, curr_y);

        // The trackers are essential; without them there is nothing to
        // integrate. The IMU is not: with it absent the heading simply holds
        // whatever set_pose() seeded, which is all straight-line work needs.
        if (!x_ok || !y_ok) {
            if (display_due) {
                pros::lcd::print(2, "ODOM no data: %s%s", x_ok ? "" : "X ", y_ok ? "" : "Y");
            }
            continue;
        }

        double     raw_h = hw_.inertial.get_heading();
        const bool h_ok  = std::isfinite(raw_h);
        if (!h_ok) raw_h = have_prev_h ? prev_raw_h : 0.0;

        // The first good sample only seeds the reference; there is no interval
        // to integrate yet, and comparing against an unset reference would
        // reject every reading that follows.
        if (!have_prev_h) {
            prev_x      = curr_x;
            prev_y      = curr_y;
            prev_raw_h  = raw_h;
            have_prev_h = true;
            continue;
        }

        const double delta_x_in = (curr_x - prev_x) * inches_per_tick;
        const double delta_y_in = (curr_y - prev_y) * inches_per_tick;
        const double delta_h    = normalise_angle(raw_h - prev_raw_h);

        // Ignore movement larger than the configured tolerance, which is
        // probably a bad read. Drop the interval but still advance the
        // reference, otherwise one bad sample makes every later one look like
        // an equally large jump and the estimate never recovers.
        const bool implausible = std::abs(delta_x_in) > config_.tolerance ||
                                 std::abs(delta_y_in) > config_.tolerance ||
                                 std::abs(delta_h)    > config_.tolerance_heading;

        if (!implausible && lock_.take(50)) {
            ReleaseOnExit guard(lock_);

            const double curr_h      = normalise_angle(raw_h + heading_offset_);
            const double delta_h_rad = delta_h * deg2rad;

            // Each tracker sits off the tracking centre, so a pure rotation
            // sweeps it through an arc that is not real translation.
            const double delta_x = delta_x_in - delta_h_rad * config_.x_tracker_offset;
            const double delta_y = delta_y_in + delta_h_rad * config_.y_tracker_offset;

            const double theta = -curr_h * deg2rad;

            x_ += delta_x * std::cos(theta) - delta_y * std::sin(theta);
            y_ += delta_x * std::sin(theta) + delta_y * std::cos(theta);
            heading_ = curr_h;
        }

        prev_x     = curr_x;
        prev_y     = curr_y;
        prev_raw_h = raw_h;

        if (display_due) {
            Pose shown;
            if (read(shown, 5)) {
                pros::lcd::print(2, "X:%.1f Y:%.1f H:%.0f%s%s",
                                 shown.x, shown.y, shown.heading,
                                 implausible ? " !" : "", h_ok ? "" : " noIMU");
            } else {
                pros::lcd::print(2, "ODOM pose locked");
            }
        }
    }
}

}
