#pragma once

#include "api.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace robot {

enum class MotionResult {
    Settled,      // reached the target and held it
    Timeout,      // ran out of time still trying
    Interrupted,  // an interrupt condition fired
    SensorFault,  // the sensors it needs stopped answering
};

const char* to_string(MotionResult result);

// True when the motion ended on its own terms rather than being cut short.
bool reached_target(MotionResult result);


class Interrupts {
public:
    using Predicate = std::function<bool()>;
    using Handle    = int;

    static constexpr Handle invalid_handle = -1;

    Interrupts();

    Interrupts(const Interrupts&)            = delete;
    Interrupts& operator=(const Interrupts&) = delete;

    Handle add(std::string name, Predicate predicate);

    void remove(Handle handle);
    void clear();

    void trip(std::string reason);

    void arm();

    bool        tripped() const;
    std::string reason() const;
    std::size_t count() const;

    bool poll();

private:
    struct Entry {
        Handle      handle;
        std::string name;
        Predicate   predicate;
    };
    using Table = std::vector<Entry>;

    mutable pros::Mutex          lock_;
    std::shared_ptr<const Table> entries_;
    Handle                       next_handle_ = 0;
    bool                         tripped_     = false;
    std::string                  reason_;
};

class ScopedInterrupt {
public:
    ScopedInterrupt(Interrupts& interrupts, std::string name, Interrupts::Predicate predicate);
    ~ScopedInterrupt();

    ScopedInterrupt(const ScopedInterrupt&)            = delete;
    ScopedInterrupt& operator=(const ScopedInterrupt&) = delete;

    ScopedInterrupt(ScopedInterrupt&& other) noexcept;
    ScopedInterrupt& operator=(ScopedInterrupt&& other) noexcept;

    void release();

    Interrupts::Handle handle() const { return handle_; }

private:
    Interrupts*        interrupts_;
    Interrupts::Handle handle_;
};

namespace triggers {

Interrupts::Predicate driver_takeover(pros::Controller& controller, int deadband = 20);

Interrupts::Predicate button_pressed(pros::Controller& controller,
                                     pros::controller_digital_e_t button);

Interrupts::Predicate motors_over_temp(pros::MotorGroup& motors, double celsius);

Interrupts::Predicate after(double seconds);

Interrupts::Predicate any_of(std::vector<Interrupts::Predicate> predicates);
Interrupts::Predicate all_of(std::vector<Interrupts::Predicate> predicates);

}

}
