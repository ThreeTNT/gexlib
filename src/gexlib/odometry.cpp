#include "gexlib/chassis.hpp"
#include "pros/imu.h"
#include "pros/rotation.h"
#include "pros/rtos.h"
#include "utils.hpp"

using namespace gexlib;

constexpr float EPSILON32 = 1e-3;

void HolonomicChassis::odometry_task_func(void* p) {
    HolonomicChassis* obj = static_cast<HolonomicChassis*>(p);

    float last_ht_pos = 0;
    float last_vt_pos = 0;
    float last_theta = 0;

    long long ctime = pros::micros();
    obj->position.lock()->last_time = ctime;
    obj->theta.lock()->last_time = ctime;

    pros::delay(10);

    while (true) {
        ctime = pros::micros();

        float cur_ht_pos = pros::c::rotation_get_position(obj->horizontal_tracker_port) \
            * CENTIGREE_TO_RAD * obj->tracker_wheel_radius;
        float cur_vt_pos = pros::c::rotation_get_position(obj->vertical_tracker_port) \
            * CENTIGREE_TO_RAD * obj->tracker_wheel_radius;
        float cur_theta = pros::c::imu_get_rotation(obj->inertial_port) 
            * DEGREE_TO_RAD 
            * -1; // we multiply by -1 because clockwise is positive in VEX but negative in canonical trigonometry

        float dtheta = cur_theta - last_theta;

        Eigen::Vector2f deltapos;
        if (std::abs(dtheta) < EPSILON32) {
            Eigen::Vector2f deltas = {
                cur_ht_pos - last_ht_pos,
                cur_vt_pos - last_vt_pos
            };
            deltapos = rotation(last_theta) * deltas;
        } else {
            auto rh = (cur_ht_pos - last_ht_pos) / dtheta - obj->horizontal_tracker_offset;
            auto rv = (cur_vt_pos - last_vt_pos) / dtheta - obj->vertical_tracker_offset;

            Eigen::Vector2f unit_diff = 
                unit_vector(cur_theta) - unit_vector(last_theta);

            deltapos = {
                rh * unit_diff.y() + rv * unit_diff.x(),
                -rh * unit_diff.x() + rv * unit_diff.y(),
            };
        }

        { // Very important, or else locks would not unlock
            auto poslock = obj->position.lock();
            auto thetalock = obj->theta.lock();
            poslock->last_pos = poslock->pos;
            poslock->pos += deltapos;
            poslock->last_time = ctime;
            thetalock->last_theta = thetalock->theta;
            thetalock->theta += dtheta * RAD_TO_DEGREE; // She prefers degrees moment!
            thetalock->last_time = ctime;
        }

        last_ht_pos = cur_ht_pos;
        last_vt_pos = cur_vt_pos;
        last_theta = cur_theta;

        pros::delay(10);
    }
}

void HolonomicChassis::start_odometry(void) {
    if (odom_task != nullptr) {
        return;
    }

    pros::c::imu_reset(inertial_port);
    pros::delay(1000);
    while (pros::c::imu_get_status(inertial_port) == pros::E_IMU_STATUS_CALIBRATING) {
        pros::delay(100);
    }
    pros::c::rotation_reset_position(horizontal_tracker_port);
    pros::c::rotation_reset_position(vertical_tracker_port);

    odom_task = pros::c::task_create(
        HolonomicChassis::odometry_task_func, 
        this, 
        TASK_PRIORITY_DEFAULT,
        TASK_STACK_DEPTH_DEFAULT,
        "odom_task"
    );
}