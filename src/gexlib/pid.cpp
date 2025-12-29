#include "gexlib/pid.hpp"
#include "pros/rtos.hpp"
#include <optional>
#include <stdexcept>
#include <type_traits>

using namespace gexlib;

int8_t sign(auto val) {
    return (0 < val) - (val < 0);
}

void PID::reset() {
    last_update_time = -1;
    integral = 0;
    target = std::nullopt;
    last_error = std::nullopt;
}

void PID::set_target(float target, bool reset) {
    this->target = target;
    if (reset) {
        last_update_time = -1;
        integral = 0;
        last_error = std::nullopt;
    }
}

template<typename T>
    requires std::integral<T> || std::floating_point<T>
float PID::update(float state,
    std::optional<float> target,
    std::optional<T> now)
{
    if (target.has_value()) this->target = target;
    
    double delta;
    long long time;
    if (now.has_value()) {
        if constexpr (std::is_floating_point_v<T>) {
            time = (long long) (now.value() * 1e6);
            delta = last_update_time.has_value() ? 
                now.value() - (last_update_time.value() / 1e6) : 0;
        } else {
            time = now.value();
            delta = last_update_time.has_value() ? 
                (now.value() - last_update_time.value()) / 1e6 : 0;
        }
    } else {
        time = pros::micros();
        delta = last_update_time.has_value() ? 
            (time - last_update_time.value()) / 1e6 : 0;
    }

    float targ;
    if (target.has_value()) {
        targ = target.value();
    } else if (this->target.has_value()) {
        targ = this->target.value();
    } else {
        throw std::invalid_argument("You did not supply a target to PID! \
            Please pass target to PID.update or use PID.set_target!");
    }

    float error = targ - state;
    float derivative = 0;
    if (last_update_time.has_value() && last_error.has_value()) {
        integral += error * delta;
        derivative += (error - last_error.value()) / delta;

        if (integral_reset && sign(error) != sign(last_error.value())) {
            integral = 0;
        }
    }

    last_update_time = time;
    last_error = error;

    return kp*error + ki*integral + kd*derivative;
}

template float PID::update<float>(float, std::optional<float>, std::optional<float>);
template float PID::update<int>(float, std::optional<float>, std::optional<int>);
template float PID::update<uint>(float, std::optional<float>, std::optional<uint>);
template float PID::update<long long>(float, std::optional<float>, std::optional<long long>);
template float PID::update<unsigned long long>(float, std::optional<float>, std::optional<unsigned long long>);
template float PID::update<double>(float, std::optional<float>, std::optional<double>);
template float PID::update<long double>(float, std::optional<float>, std::optional<long double>);