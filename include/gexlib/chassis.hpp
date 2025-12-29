#pragma once

#include "Eigen/Core"
#include "future.hpp"
#include "pid.hpp"
#include "pros/rtos.hpp"
#include <optional>
#include <vector>

namespace gexlib {
struct Motor {
    Eigen::Vector2f direction;
    int8_t port;

    static Motor from_degrees(int8_t port, float angle);
    static Motor from_radians(int8_t port, float angle);
};

enum ResultCode {
    SUCCESS = 0,
    CANCELLED = 1,
    TIMEOUT = 2,
    ERROR = -1
};

struct MovementResult {
    ResultCode resultcode;
    double time_taken;

    std::optional<Eigen::Vector2f> position_error;
    std::optional<float> angle_error;
};

struct MovementConfig {
    /*
     Position to go to

     Leaving as `std::nullopt` will make the robot not move across the field,
     only turning in place (if `angle` is not `std::nullopt`)

     This is by default in the global reference frame (or the reference frame of
     the last reset)
    */
    std::optional<Eigen::Vector2f> pos = std::nullopt;
    /*
     Desired angle of the robot

     Leaving as `std::nullopt` will make the robot not turn when moving,
     only moving across the field (if `pos` is not `std::nullopt`)
    */
    std::optional<float> angle = std::nullopt;

    /* 
     Maximum time to allocate to the movement before giving up and exiting
     as a `ResultCode::TIMEOUT`.

     Leaving this as `std::nullopt` will make the movement never exit until
     the robot satisfies all four thresholds for exit (angle, angular velocity,
     distance, speed). As such, it is recommended to leave at least a high value
     for the timeout.
     */
    std::optional<double> timeout = std::nullopt;

    /* 
     Acceptable angle error (difference between target and robot angle) 
     tolerance for determining wether to exit as a successful movement

     Leaving this as `std::nullopt` will make the movement never exit,
     which is not recommended - The robot will keep on trying to reach the
     desired angle until timeout.
     */
    std::optional<float> angle_threshold = std::nullopt;

    /*
     Acceptable angular velocity tolerance for determining wether to exit
     as a successful movement

     This is used as an ensurance to the angle error, making sure that
     if the movement exits as a success, in the immediate time after the
     movement the angle would not change relatively much. This can be used to
     ensure precise movements, though it requires more time for the movement
     to settle down.

     Leaving this as `std::nullopt` turns off this option, allowing
     exit at any angular velocity.
     */
    std::optional<float> angular_speed_threshold = std::nullopt;

    /*
     Acceptable distance error (distance between robot pos and target pos) 
     tolerance for determining wether to exit as a successful movement

     Leaving this as `std::nullopt` will make the movement never exit,
     which is not recommended - The robot will keep on trying to reach the
     desired position until timeout.
    */
    std::optional<float> distance_threshold = std::nullopt;

    /*
     Acceptable speed tolerance for determining wether to exit as a successful
     movement.

     This is used as an ensurance to the distance error, making sure that
     if the movement exits as a success, in the immediate time after the
     movement the robot would not move very much around the target. This can 
     be used to ensure precise movements, though it requires more time for 
     the movement to settle down.

     Leaving this as `std::nullopt` turns off this option, allowing
     exit at any speed.
     */
    std::optional<float> speed_threshold = std::nullopt;
};

class HolonomicChassis {
public:
    // Wheel modules that define drive direction and port assignments.
    std::vector<Motor> motors;
    // Port for a PROS inertial sensor used for heading feedback.
    uint8_t inertial_port;

    // Tracking wheel encoder ports and their physical offsets (inches or meters) from robot center.
    int8_t horizontal_tracker_port;
    int8_t vertical_tracker_port;
    float horizontal_tracker_offset;
    float vertical_tracker_offset;

    // PID controllers used to hold/drive position and heading goals.
    PID position_pid;
    PID angular_pid;

    // Get the position of the robot
    Eigen::Vector2f pos() { return position.lock()->pos; }
    // Set the position to `x = 0` and `y = 0`
    void reset_pos() { set_pos({0, 0}); }
    // Set the position to some arbitrary point
    void set_pos(Eigen::Vector2f const& pos) {
        auto lock = position.lock();
        lock->pos = pos;
        lock->last_pos = pos;
        lock->last_time = -1;
    }

    // Get the robot angle
    float angle() { return theta.lock()->theta; }
    // Set the robot angle to 0
    void reset_angle() { set_angle(0); }
    // Set the robot angle to some arbitrary degree
    void set_angle(float angle) {
        auto lock = theta.lock();
        lock->theta = angle;
        lock->last_theta = angle;
        lock->last_time = -1;
    }

    // Convenient function to reset both position and angle at the same time
    void reset_pose() { set_pose({0, 0}, 0); }
    // Set the pose (pos, theta pair) to some arbitrary value
    void set_pose(Eigen::Vector2f const& pos, float angle) {
        set_pos(pos);
        set_angle(angle);
    }

    // Get the velocity vector, in the x and y directions separately.
    // Note this can have negative values.
    Eigen::Vector2f velocity() { return position.lock()->derivative(); }
    // Get the angular velocity (how fast the angle of the robot is changing over time)
    float angular_velocity() { return theta.lock()->derivative(); }
    // Get the speed of the robot.
    // Note this is always positive.
    float speed() { return velocity().norm(); }
    // Get the angular speed, or the absolute value of the angular velocity.
    float angular_speed() { return std::abs(angular_velocity()); }

    Eigen::Vector2f to_local_frame(Eigen::Vector2f const& gpos);
    Eigen::Vector2f to_global_frame(Eigen::Vector2f const& gpos);

    /*
     Executes a movement synchronously, meaning the function does not
     return until the movement is complete and a result is returned.

     This can be used for simple movements, where you don't need to execute
     intermediate logic (such as turning on intake midway through moving)
     and only desire the movement itself.
    */
    MovementResult execute_movement(MovementConfig const& config);

    /*
     Executes a movement asynchronously, meaning the function immediately
     returns a `Future<MovementResult>` representing the eventual completion
     of the command while a background task performs the motion.

     This is ideal when higher level logic needs to keep running in parallel
     (coordinating other subsystems, reacting to sensors, chaining commands,
     etc.). The returned future can be polled, awaited, or ignored based on
     how tightly the caller needs to synchronize with the chassis.
    */
    Future<MovementResult> execute_movement_async(MovementConfig const& config);

    /*
     Start the odometry task, tracking the robot's position and angle.
    */
    void start_odometry(void);

private:
    struct Position {
        Eigen::Vector2f pos = {0, 0};
        Eigen::Vector2f last_pos = {0, 0};
        long long last_time = -1;

        Eigen::Vector2f derivative() {
            if (last_time == -1) {
                return {0, 0};
            } else {
                double dt = (pros::micros() - last_time) / 1e6;
                return (pos - last_pos) / dt;
            }
        }
    };

    struct Angle {
        float theta = 0;
        float last_theta = 0;
        long long last_time = -1;

        float derivative() {
            if (last_time == -1) {
                return 0;
            } else {
                double dt = std::max(1e-6, (pros::micros() - last_time) / 1e6);
                return (theta - last_theta) / dt;
            }
        }
    };

    pros::MutexVar<Position> position;
    pros::MutexVar<Angle> theta;
    pros::task_t odom_task = nullptr;

    static void odom_task_func(void* p);
};
}