#include "Eigen/Core"
#include "gexlib/chassis.hpp"
#include "gexlib/future.hpp"
#include "pros/motors.h"
#include <limits>

using namespace gexlib;

static inline void zero_velocity(HolonomicChassis* chassis) {
    for (auto& motor : chassis->motors) {
        pros::c::motor_move_velocity(motor.port, 0);
    }
}

static MovementResult execute_movement_internal(
    HolonomicChassis* chassis,
    MovementConfig const& config,
    std::function<bool (void)> cancelled)
{
    long long start_time = pros::micros();
    long long last_time = start_time;
    double timeout = config.timeout.value_or(std::numeric_limits<double>::infinity());

    chassis->position_pid.reset();
    chassis->angular_pid.reset();

    Eigen::Vector2f pos_error {0, 0};
    float theta_error = 0;

    float controls[21] = {0};
    for (auto& motor : chassis->motors) {
        controls[motor.port] = pros::c::motor_get_actual_velocity(motor.port);
    }

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
            pos_error = chassis->to_local_frame(config.pos.value());
            pos_control = chassis->position_pid.update<long long>(0, pos_error.norm(), ctime);
        }

        // Compute theta error and control if its desired in the config
        theta_error = 0;
        float theta_control = 0;
        if (config.angle.has_value()) {
            theta_error = std::remainder(config.angle.value() - robot_theta, 360);
            theta_control = chassis->angular_pid.update<long long>(0, theta_error, ctime);
        }

        // Compute exit conditions
        bool sat = true;
        if (config.angle.has_value()) {
            sat &= std::abs(theta_error) <= config.angle_threshold.value()
                && config.angle_threshold.has_value();

            if (config.angular_speed_threshold.has_value())
                sat &= chassis->angular_speed() <= config.angular_speed_threshold.value();
        }
        if (config.pos.has_value()) {
            sat &= pos_error.norm() <= config.distance_threshold.value() 
                && config.distance_threshold.has_value();

            if (config.speed_threshold.has_value())
                sat &= chassis->speed() <= config.speed_threshold.value();
        }

        // printf("Robot pos: (%.4f, %.4f), Robot theta: %.4f, Pos error: (%.4f, %.4f), Theta error: %.4f, Sat: %d\n", 
        //     robot_pos.x(), robot_pos.y(), robot_theta, pos_error.x(), pos_error.y(), theta_error, sat);

        if (sat) {
            zero_velocity(chassis);
            return MovementResult {
                .resultcode = ResultCode::SUCCESS,
                .time_taken = (ctime - start_time) / 1e6,
                .position_error = config.pos.has_value() ? std::optional(pos_error) : std::nullopt,
                .angle_error = config.angle.has_value() ? std::optional(theta_error) : std::nullopt
            };
        } else if (cancelled()) {
            zero_velocity(chassis);
            return MovementResult {
                .resultcode = ResultCode::CANCELLED,
                .time_taken = (ctime - start_time) / 1e6,
                .position_error = config.pos.has_value() ? std::optional(pos_error) : std::nullopt,
                .angle_error = config.angle.has_value() ? std::optional(theta_error) : std::nullopt
            };
        }

        std::vector<std::pair<int8_t, float>> dots;
        float pos_error_norm = std::max(1e-5f, pos_error.norm());
        float max_dot = 1.0f;
        for (auto const& motor : chassis->motors) {
            float dot = theta_control;

            if (config.pos.has_value()) {
                dot += motor.direction.dot(pos_error / pos_error_norm)
                    * pos_control;
                    // * std::cos(theta_error * DEGREE_TO_RAD);
            }

            max_dot = std::max(max_dot, std::abs(dot));
            dots.emplace_back(motor.port, dot);
        }

        // Calculate slew in RPM
        double dt = (ctime - last_time) / 1e6;
        float slew = config.slew_rate * dt;

        // Output to pros
        for (auto& [port, ctrl] : dots) {
            float desired_velo = std::round(ctrl * config.max_velocity / max_dot);
            float actual = std::clamp(
                desired_velo,
                std::max(-config.max_velocity, controls[port] - slew),
                std::min(config.max_velocity, controls[port] + slew));
            printf("port %d: control %.4f, output %.2f\n", port, ctrl, actual);

            controls[port] = actual;
            pros::c::motor_move_velocity(port, actual);
        }

        last_time = ctime;
        pros::delay(10);
    }

    zero_velocity(chassis);
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

    // pros::c::task_create(
    //     lambda,
    //     static_cast<void*>(params),
    //     TASK_PRIORITY_DEFAULT,
    //     TASK_STACK_DEPTH_DEFAULT,
    //     "Movement task");

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
