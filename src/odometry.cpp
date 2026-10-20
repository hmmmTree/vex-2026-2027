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

// Odometry tick. Half the control loop, so every 20 ms control tick sees a
// fresh pose. The rotation sensors and IMU both report every 10 ms.
constexpr std::uint32_t ODOM_INTERVAL_MS = 10;

constexpr int DISPLAY_EVERY_TICKS = 20;  // ~200 ms, as before

// A rotation sensor reports failure as PROS_ERR, which is INT32_MAX and so
// stays perfectly finite once it becomes a double. It has to be checked by
// value, or a dead port reads as a few hundred thousand inches of travel.
bool read_ticks(const pros::Rotation& sensor, double& out) {
    const std::int32_t raw = sensor.get_position();
    if (raw == PROS_ERR) return false;
    out = static_cast<double>(raw);
    return true;
}

// Two IMUs closer than this are averaged; further apart, one of them is wrong.
constexpr double IMU_AGREE_DEG      = 5.0;
// The V5 IMU cannot report a faster turn than this, so a bigger step between
// reads is a reset or a glitch, not the robot moving.
constexpr double IMU_MAX_RATE_DPS   = 1000.0;
constexpr double IMU_JUMP_SLACK_DEG = 5.0;

// An IMU reports failure as PROS_ERR_F (infinity), and reads 0 while
// calibrating, so both have to be filtered before the value means anything.
bool read_imu(pros::Imu& imu, double& out) {
    if (imu.is_calibrating()) return false;  // also true on PROS_ERR
    const double h = imu.get_heading();
    if (!std::isfinite(h) || h < 0.0 || h > 360.0) return false;
    out = h;
    return true;
}

double wrap_360(double angle) {
    angle = std::fmod(angle, 360.0);
    return angle < 0.0 ? angle + 360.0 : angle;
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
    double imu_now = fused_imu_raw();
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
    const double raw = fused_imu_raw();
    if (!std::isfinite(raw)) return std::numeric_limits<double>::quiet_NaN();
    return normalise_angle(raw + heading_offset());
}

double Odometry::fused_imu_raw() const {
    pros::Imu* const imus[2] = {&hw_.inertial1, &hw_.inertial2};

    // Device reads outside the lock, same as the odometry task does.
    double reading[2] = {0.0, 0.0};
    bool   ok[2];
    for (int i = 0; i < 2; ++i) ok[i] = read_imu(*imus[i], reading[i]);
    const std::uint32_t now = pros::millis();

    if (!imu_lock_.take(TIMEOUT_MAX)) return std::numeric_limits<double>::quiet_NaN();
    ReleaseOnExit guard(imu_lock_);

    for (int i = 0; i < 2; ++i) {
        ImuTrack& t = imu_track_[i];
        if (!ok[i]) {
            // One that was working and then dropped out (unplugged, brownout)
            // recalibrates to 0 when it comes back, so it has to re-earn trust.
            // Calibrating at boot is not a fault: it has never been seen yet.
            if (t.seen) t.faulted = true;
            t.seen = false;
            continue;
        }
        if (t.seen) {
            const double allowed = IMU_MAX_RATE_DPS * (now - t.last_ms) / 1000.0 + IMU_JUMP_SLACK_DEG;
            if (std::abs(normalise_angle(reading[i] - t.last)) > allowed) t.faulted = true;
        }
        t.last    = reading[i];
        t.last_ms = now;
        t.seen    = true;
    }

    std::uint8_t used = 0;
    double       result = 0.0;

    if (ok[0] && ok[1]) {
        const double diff = normalise_angle(reading[1] - reading[0]);
        if (std::abs(diff) <= IMU_AGREE_DEG) {
            // Agreeing is how a faulted IMU earns its way back in.
            imu_track_[0].faulted = imu_track_[1].faulted = false;
            used   = 0b11;
            result = wrap_360(reading[0] + diff / 2.0);  // circular mean: 359 and 1 give 0, not 180
        } else if (imu_track_[0].faulted != imu_track_[1].faulted) {
            used = imu_track_[0].faulted ? 0b10 : 0b01;
        } else {
            // They disagree and neither has done anything obviously wrong
            // (slow drift). Trust whichever stayed with the last good heading
            // and drop the other, so the choice does not flip every tick.
            int keep = 0;
            if (have_last_fused_ &&
                std::abs(normalise_angle(reading[1] - last_fused_)) <
                std::abs(normalise_angle(reading[0] - last_fused_))) {
                keep = 1;
            }
            imu_track_[1 - keep].faulted = true;
            used = keep == 0 ? 0b01 : 0b10;
        }
        if (used != 0b11) result = reading[used == 0b01 ? 0 : 1];
    } else if (ok[0] && !imu_track_[0].faulted) {
        used = 0b01; result = reading[0];
    } else if (ok[1] && !imu_track_[1].faulted) {
        used = 0b10; result = reading[1];
    } else {
        imu_used_.store(0);
        return std::numeric_limits<double>::quiet_NaN();
    }

    last_fused_      = result;
    have_last_fused_ = true;
    imu_used_.store(used);
    return result;
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

double Odometry::lift_height() const {
    if (!lock_.take(TIMEOUT_MAX)) return 0.0;
    ReleaseOnExit guard(lock_);
    return lift_height_;
}

void Odometry::reset_lift_height(double h) {
    const std::int32_t raw = hw_.lift_r.get_position();
    if (raw == PROS_ERR) return;
    if (!lock_.take(TIMEOUT_MAX)) return;
    ReleaseOnExit guard(lock_);
    // pick the zero so that the current sensor angle reads as height h
    lift_zero_deg_ = raw / 100.0 - h / LIFT_IN_PER_DEG;
    lift_height_   = h;
}

double Odometry::lift_stage() const {
    if (!lock_.take(TIMEOUT_MAX)) return 0.0;
    ReleaseOnExit guard(lock_);
    return lift_stage_;
}

void Odometry::set_lift_stage(double stage) {
    if (!lock_.take(TIMEOUT_MAX)) return;
    ReleaseOnExit guard(lock_);
    lift_stage_ = stage;
}

void Odometry::run() {
    double prev_x      = 0.0;
    double prev_y      = 0.0;
    double prev_raw_h  = 0.0;
    bool   have_prev_h = false;
    int    display_counter = 0;

    std::uint32_t last_tick = pros::millis();

    while (!stop_requested_.load()) {
        // Device reads happen outside the lock they talk to the V5 brain and
        // must not hold up a control loop that only wants the last pose.
        // Every path out of this iteration must delay, or the task spins and
        // starves the rest of the system including the brain screen.
        //
        // Fixed cadence, faster than the 20 ms control loops: a flat delay
        // plus the device reads made this tick ~25 ms, so cordon saw the same
        // pose twice and then a double step, and the carrot moved in jerks.
        // Always sleeps at least 1 ms even when a tick overruns.
        const std::uint32_t since = pros::millis() - last_tick;
        pros::delay(since >= ODOM_INTERVAL_MS ? 1 : ODOM_INTERVAL_MS - since);
        last_tick = pros::millis();

        // Decided before any early exit, so a tick that cannot produce a pose
        // still gets to say so instead of leaving the line frozen.
        const bool display_due = ++display_counter >= DISPLAY_EVERY_TICKS;
        if (display_due) display_counter = 0;

        // The heading is an absolute reading, not something integrated, so it
        // is published as soon as it is read. It used to be assigned only
        // inside the guarded block below, which meant any skipped iteration -
        // a failed tracker read, a rejected interval, a missed lock - left it
        // frozen while the robot kept turning, and the next good tick snapped
        // it forward by everything that had accumulated. A control loop
        // differentiating that sees a step, not a turn rate.
        double     raw_h = fused_imu_raw();
        double lift_raw = 0.0;
        const bool lift_ok = read_ticks(hw_.lift_r, lift_raw);  // same PROS_ERR check, centidegree
        const bool h_ok  = std::isfinite(raw_h);

        double curr_x = 0.0;
        double curr_y = 0.0;
        const bool x_ok = read_ticks(hw_.xrot, curr_x);
        const bool y_ok = read_ticks(hw_.yrot, curr_y);

        // The trackers are essential; without them there is nothing to
        // integrate. The IMU is not: with it absent the heading simply holds
        // whatever set_pose() seeded, which is all straight-line work needs.
        if (!x_ok || !y_ok) {
            // The heading is still good. Leaving it frozen here is what made it
            // snap forward by the whole accumulated turn on the next good tick.
            if ((h_ok || lift_ok) && lock_.take(50)) {
                ReleaseOnExit guard(lock_);
                if (h_ok)    heading_     = normalise_angle(raw_h + heading_offset_);
                if (lift_ok) lift_height_ = (lift_raw / 100.0 - lift_zero_deg_) * LIFT_IN_PER_DEG;
            }
            if (display_due) {
                pros::lcd::print(2, "ODOM no data: %s%s", x_ok ? "" : "X ", y_ok ? "" : "Y");
            }
            continue;
        }

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

        // One lock for both updates, not two: the control loop asks for the
        // pose on the same 20 ms cadence this task runs on, and every extra
        // take is another chance for one of them to wait on the other.
        // The heading is an absolute reading, so it is published whatever the
        // interval did; only the integrated position is gated on plausibility.
        if (lock_.take(50)) {
            ReleaseOnExit guard(lock_);

            const double curr_h = normalise_angle(raw_h + heading_offset_);
            if (h_ok) heading_ = curr_h;
            if (lift_ok) lift_height_ = (lift_raw / 100.0 - lift_zero_deg_) * LIFT_IN_PER_DEG;

            if (!implausible) {
                const double delta_h_rad = delta_h * deg2rad;

                // Each tracker sits off the tracking centre, so a pure rotation
                // sweeps it through an arc that is not real translation.
                const double delta_x = delta_x_in - delta_h_rad * config_.x_tracker_offset;
                const double delta_y = delta_y_in + delta_h_rad * config_.y_tracker_offset;

                const double theta = -curr_h * deg2rad;

                x_ += delta_x * std::cos(theta) - delta_y * std::sin(theta);
                y_ += delta_x * std::sin(theta) + delta_y * std::cos(theta);
            }
        }

        prev_x     = curr_x;
        prev_y     = curr_y;
        prev_raw_h = raw_h;

        if (display_due) {
            Pose shown;
            if (read(shown, 5)) {
                const std::uint8_t used = imu_used_.load();
                pros::lcd::print(2, "X:%.1f Y:%.1f H:%.0f%s%s",
                                 shown.x, shown.y, shown.heading,
                                 implausible ? " !" : "",
                                 used == 0b11 ? "" : used == 0b01 ? " IMU2x"
                                                   : used == 0b10 ? " IMU1x" : " noIMU");
            } else {
                pros::lcd::print(2, "ODOM pose locked");
            }
        }
    }
}

}
