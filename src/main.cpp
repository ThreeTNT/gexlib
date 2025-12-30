#include "main.h"
#include "gexlib/chassis.hpp"
#include "gexlib/utils.hpp"
#include "pros/motors.h"

static gexlib::HolonomicChassis* chassis;

void initialize() {
    printf("Initializing chassis...\n");

    chassis = new gexlib::HolonomicChassis {
        std::vector<gexlib::Motor> {
            gexlib::Motor::from_degrees(1, -45),
            gexlib::Motor::from_degrees(-2, 0),
            gexlib::Motor::from_degrees(3, 45),
            gexlib::Motor::from_degrees(8, 135),
            gexlib::Motor::from_degrees(-9, 180),
            gexlib::Motor::from_degrees(10, 225)
        },
        19, 
        5, 
        6,
        0.0363539, 
        0.0360267,
        1.0 * gexlib::INCH_TO_METER,
        gexlib::PID(1.75, 0.1, 0.4),   // position PID
        gexlib::PID(0.5 * gexlib::DEGREE_TO_RAD, 0.0, 0.05 * gexlib::DEGREE_TO_RAD)    // angular PID
    };

    for (auto& motor : chassis->motors) {
        pros::c::motor_set_gearing(motor.port, pros::motor_gearset_e_t::E_MOTOR_GEARSET_06);
        pros::c::motor_set_encoder_units(motor.port, pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
    }

    printf("Chassis initialized\n");
}

void disabled() {
    printf("Disabled...\n");
}
void competition_initialize() {
    printf("Competition Initialize...\n");
}
void autonomous() {
    printf("Autonomous...\n");
}

static void tune_odom(float velocity = 200, float duration = 3.0f) {
    std::vector<Eigen::Vector2f> points;
    
    auto start_time = pros::millis();

    for (auto& motor : chassis->motors) {
        pros::c::motor_move_velocity(motor.port, velocity);
    }

    while (true) {
        auto pos = chassis->pos();
        auto angle = chassis->angle();
        points.push_back(pos);
        pros::delay(20);
        if (pros::millis() - start_time > (int) (duration * 1000)) {
            break;
        }
    }

    for (auto& motor : chassis->motors) {
        pros::c::motor_move_velocity(motor.port, 0);
    }

    printf("P = [");
    for (size_t i = 0; i < points.size(); i++) {
        printf(
            (i != points.size() - 1) ? "(%.4f,%.4f)," : "(%.4f,%.4f)",
            points[i].x(), 
            points[i].y());
    }
    printf("]\n");
}

static lv_obj_t* label;

static void lvgl_task(void* p) {
    while (true) {
        auto pos = chassis->pos();
        auto angle = chassis->angle();
        char buf[64];
        snprintf(buf, sizeof(buf), "Position: (%.4f, %.4f)\nAngle: %.4f", 
            pos.x(), pos.y(), angle);
        lv_label_set_text(label, buf);

        pros::delay(100);
    }
}

void opcontrol() {
    chassis->start_odometry();

    label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Initializing...");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    pros::task_t lvgl_updater = pros::c::task_create(
        lvgl_task,
        nullptr,
        TASK_PRIORITY_DEFAULT,
        TASK_STACK_DEPTH_DEFAULT,
        "LVGL Updater");

    while (true) {
        auto pos = chassis->pos();
        auto angle = chassis->angle();
        printf("Position: (%.4f, %.4f), Angle: %.4f\n", pos.x(), pos.y(), angle);
        pros::delay(100);
    }

    // chassis->execute_movement(
    //     {
    //         .pos = Eigen::Vector2f {0.6, 0.0},
    //         .angle = 90.0f,
    //         .timeout = 5,
    //         .angle_threshold = 1.0f,
    //         .distance_threshold = 0.01,
    //     }
    // );

    // chassis->execute_movement(
    //     {
    //         // .pos = Eigen::Vector2f {0.6, 0.0},
    //         .angle = 0.0f,
    //         .timeout = 3.5,
    //         .angle_threshold = 1.0f,
    //         .angular_speed_threshold = 10.0f,
    //         .distance_threshold = 0.01,
    //     }
    // );
    // chassis->execute_movement(
    //     {
    //         .pos = Eigen::Vector2f {0.6, 0.6},
    //         .angle = 90.0f,
    //         .timeout = 2.5,
    //         .distance_threshold = 0.01,
    //     }
    // );

    // chassis->execute_movement(
    //     {
    //         .pos = Eigen::Vector2f {0.6, 0.0},
    //         .angle = 0.0f,
    //         .timeout = 2.5,
    //         .distance_threshold = 0.01,
    //     }
    // );

    // chassis->execute_movement(
    //     {
    //         .pos = Eigen::Vector2f {0.0, 0.0},
    //         .angle = 0.0f,
    //         .timeout = 2.5,
    //         .distance_threshold = 0.01,
    //     }
    // );
}