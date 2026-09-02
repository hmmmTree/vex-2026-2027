#include "robot/interrupt.hpp"

#include <cmath>
#include <utility>

namespace robot {

namespace {

class Lock {
public:
    explicit Lock(pros::Mutex& mutex) : mutex_(mutex) { mutex_.take(TIMEOUT_MAX); }
    ~Lock() { mutex_.give(); }

    Lock(const Lock&)            = delete;
    Lock& operator=(const Lock&) = delete;

private:
    pros::Mutex& mutex_;
};

int safe_analog(pros::Controller& controller, pros::controller_analog_e_t channel) {
    const std::int32_t value = controller.get_analog(channel);
    return value == PROS_ERR ? 0 : static_cast<int>(value);
}

}

const char* to_string(MotionResult result) {
    switch (result) {
        case MotionResult::Settled:     return "settled";
        case MotionResult::Timeout:     return "timeout";
        case MotionResult::Interrupted: return "interrupted";
        case MotionResult::SensorFault: return "sensor fault";
    }
    return "unknown";
}

bool reached_target(MotionResult result) {
    return result == MotionResult::Settled;
}

Interrupts::Interrupts() : entries_(std::make_shared<const Table>()) {}

Interrupts::Handle Interrupts::add(std::string name, Predicate predicate) {
    if (!predicate) return invalid_handle;

    Lock guard(lock_);
    auto next = std::make_shared<Table>(*entries_);
    const Handle handle = next_handle_++;
    next->push_back(Entry{handle, std::move(name), std::move(predicate)});
    entries_ = std::move(next);
    return handle;
}

void Interrupts::remove(Handle handle) {
    if (handle == invalid_handle) return;

    Lock guard(lock_);
    auto next = std::make_shared<Table>();
    next->reserve(entries_->size());
    for (const Entry& entry : *entries_) {
        if (entry.handle != handle) next->push_back(entry);
    }
    entries_ = std::move(next);
}

void Interrupts::clear() {
    Lock guard(lock_);
    entries_ = std::make_shared<const Table>();
}

void Interrupts::trip(std::string reason) {
    Lock guard(lock_);
    if (tripped_) return; 
    tripped_ = true;
    reason_  = std::move(reason);
}

void Interrupts::arm() {
    Lock guard(lock_);
    tripped_ = false;
    reason_.clear();
}

bool Interrupts::tripped() const {
    Lock guard(lock_);
    return tripped_;
}

std::string Interrupts::reason() const {
    Lock guard(lock_);
    return reason_;
}

std::size_t Interrupts::count() const {
    Lock guard(lock_);
    return entries_->size();
}

bool Interrupts::poll() {
    std::shared_ptr<const Table> snapshot;
    {
        Lock guard(lock_);
        if (tripped_) return true;
        snapshot = entries_;
    }

    // must not deadlock.
    for (const Entry& entry : *snapshot) {
        if (entry.predicate && entry.predicate()) {
            trip(entry.name);
            return true;
        }
    }
    return false;
}

ScopedInterrupt::ScopedInterrupt(Interrupts& interrupts, std::string name,
                                 Interrupts::Predicate predicate)
    : interrupts_(&interrupts),
      handle_(interrupts.add(std::move(name), std::move(predicate))) {}

ScopedInterrupt::~ScopedInterrupt() {
    release();
}

ScopedInterrupt::ScopedInterrupt(ScopedInterrupt&& other) noexcept
    : interrupts_(other.interrupts_), handle_(other.handle_) {
    other.interrupts_ = nullptr;
    other.handle_     = Interrupts::invalid_handle;
}

ScopedInterrupt& ScopedInterrupt::operator=(ScopedInterrupt&& other) noexcept {
    if (this != &other) {
        release();
        interrupts_       = other.interrupts_;
        handle_           = other.handle_;
        other.interrupts_ = nullptr;
        other.handle_     = Interrupts::invalid_handle;
    }
    return *this;
}

void ScopedInterrupt::release() {
    if (interrupts_ != nullptr) interrupts_->remove(handle_);
    interrupts_ = nullptr;
    handle_     = Interrupts::invalid_handle;
}

namespace triggers {

Interrupts::Predicate driver_takeover(pros::Controller& controller, int deadband) {
    return [&controller, deadband] {
        return std::abs(safe_analog(controller, pros::E_CONTROLLER_ANALOG_LEFT_Y))  > deadband
            || std::abs(safe_analog(controller, pros::E_CONTROLLER_ANALOG_LEFT_X))  > deadband
            || std::abs(safe_analog(controller, pros::E_CONTROLLER_ANALOG_RIGHT_Y)) > deadband
            || std::abs(safe_analog(controller, pros::E_CONTROLLER_ANALOG_RIGHT_X)) > deadband;
    };
}

Interrupts::Predicate button_pressed(pros::Controller& controller,
                                     pros::controller_digital_e_t button) {
    return [&controller, button] {
        const std::int32_t pressed = controller.get_digital(button);
        return pressed != PROS_ERR && pressed != 0;
    };
}

Interrupts::Predicate motors_over_temp(pros::MotorGroup& motors, double celsius) {
    return [&motors, celsius] {
        for (double temperature : motors.get_temperature_all()) {
            if (std::isfinite(temperature) && temperature > celsius) return true;
        }
        return false;
    };
}

Interrupts::Predicate after(double seconds) {
    const std::uint32_t deadline = pros::millis() + static_cast<std::uint32_t>(seconds * 1000.0);
    return [deadline] { return pros::millis() >= deadline; };
}

Interrupts::Predicate any_of(std::vector<Interrupts::Predicate> predicates) {
    return [predicates = std::move(predicates)] {
        for (const auto& predicate : predicates) {
            if (predicate && predicate()) return true;
        }
        return false;
    };
}

Interrupts::Predicate all_of(std::vector<Interrupts::Predicate> predicates) {
    return [predicates = std::move(predicates)] {
        if (predicates.empty()) return false;
        for (const auto& predicate : predicates) {
            if (!predicate || !predicate()) return false;
        }
        return true;
    };
}

}

}
