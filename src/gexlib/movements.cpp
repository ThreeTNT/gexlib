#include "Eigen/Core"
#include "gexlib/chassis.hpp"
#include "gexlib/future.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include <limits>

using namespace gexlib;

static MovementResult execute_movement_internal(
    HolonomicChassis* chassis,
    MovementConfig const& config,
    std::function<bool (void)> cancelled)
{
    long long start_time = pros::micros();
    double timeout = config.timeout.value_or(std::numeric_limits<double>::infinity());

    chassis->position_pid.reset();
    chassis->angular_pid.reset();

    Eigen::Vector2f pos_error {0, 0};
    float theta_error = 0;

    long long ctime = pros::micros();
    while ((ctime - start_time) / 1e6 <= timeout) {
        ctime = pros::micros();

        // Copy over pose data to avoid locking over and over
        Eigen::Vector2f robot_pos = chassis->pos();
        float robot_theta = chassis->angle();

        // Compute position error and control if its desired in the config
        pos_error = {0, 0};
        float pos_control = 0;
        if (config.pos.has_value()) {
            pos_error = config.pos.value() - robot_pos;
            pos_error /= std::max(1e-5f, pos_error.norm());
            pos_control = chassis->position_pid.update<long long>(pos_error.norm(), 0, ctime);
        }

        // Compute theta error and control if its desired in the config
        theta_error = 0;
        float theta_control = 0;
        if (config.angle.has_value()) {
            theta_error = std::remainder(config.angle.value() - robot_theta, 360);
            theta_control = chassis->angular_pid.update<long long>(pos_error.norm(), std::nullopt, ctime);
        }

        // Compute exit conditions
        bool sat = true;
        if (config.angle_threshold.has_value())
            sat &= std::abs(theta_error) <= config.angle_threshold.value();
        if (config.angular_speed_threshold.has_value())
            sat &= chassis->angular_speed() <= config.angular_speed_threshold.value();
        if (config.distance_threshold.has_value())
            sat &= pos_error.norm() <= config.distance_threshold.value();
        if (config.speed_threshold.has_value())
            sat &= chassis->speed() <= config.speed_threshold.value();

        if (sat) {
            return MovementResult {
                .resultcode = ResultCode::SUCCESS,
                .time_taken = (ctime - start_time) / 1e6,
                .position_error = config.pos.has_value() ? std::optional(pos_error) : std::nullopt,
                .angle_error = config.angle.has_value() ? std::optional(theta_error) : std::nullopt
            };
        } else if (cancelled()) {
            return MovementResult {
                .resultcode = ResultCode::CANCELLED,
                .time_taken = (ctime - start_time) / 1e6,
                .position_error = config.pos.has_value() ? std::optional(pos_error) : std::nullopt,
                .angle_error = config.angle.has_value() ? std::optional(theta_error) : std::nullopt
            };
        }

        std::vector<std::pair<uint8_t, float>> controls;
        for (auto const& motor : chassis->motors) {
            float dot = theta_control;
            if (config.pos.has_value()) {
                dot += motor.direction.dot(pos_error) * pos_control;
            }
            controls.emplace_back(motor.port, dot);
        }

        // Find the maximum control output
        auto max_it = std::max_element(
            controls.begin(), 
            controls.end(),
            [](const auto& a, const auto& b) { return std::abs(a.second) < std::abs(b.second); });
        float maxdot = std::max(1.0f, std::abs(max_it->second));

        // Output to pros
        for (const auto& [port, ctrl] : controls) {
            int millivolts = std::round(ctrl / maxdot * 12000);
            pros::c::motor_move_voltage(port, millivolts);
        }

        pros::delay(10);
    }

    return MovementResult {
        .resultcode = ResultCode::TIMEOUT,
        .time_taken = (ctime - start_time) / 1e6,
        .position_error = config.pos.has_value() ? std::optional(pos_error) : std::nullopt,
        .angle_error = config.angle.has_value() ? std::optional(theta_error) : std::nullopt
    };
}

Future<MovementResult> HolonomicChassis::execute_movement_async(
    MovementConfig const& config)
{
    typedef struct {
        Promise<MovementResult> promise;
        HolonomicChassis* chassis;
        MovementConfig config;
    } params_t;

    auto lambda = [](void* p) {
        std::unique_ptr<params_t> params(static_cast<params_t*>(p));
        auto future = params->promise.get_future();

        auto res = execute_movement_internal(
            params->chassis, 
            params->config, 
            [&future]() { return future.cancelled(); });

        params->promise.set_value(res);
    };

    auto params = new params_t {
        .promise = Promise<MovementResult>(),
        .chassis = this,
        .config = config
    };

    auto fut = params->promise.get_future();

    pros::c::task_create(
        lambda,
        static_cast<void*>(params),
        TASK_PRIORITY_DEFAULT,
        TASK_STACK_DEPTH_DEFAULT,
        "Movement task");

    return fut;
}

MovementResult HolonomicChassis::execute_movement(
    MovementConfig const& config)
{
    return execute_movement_internal(
        this, 
        config, 
        []() -> bool { return false; });
}
