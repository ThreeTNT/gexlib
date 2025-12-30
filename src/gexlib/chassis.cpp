
#include "gexlib/chassis.hpp"
#include "utils.hpp"

using namespace gexlib;

Motor Motor::from_degrees(int8_t port, float angle) {
    return from_radians(port, angle * (M_PI / 180.0f));
}

Motor Motor::from_radians(int8_t port, float angle) {
    return {
        .direction = unit_vector(angle),
        .port = port
    };
}

Eigen::Vector2f HolonomicChassis::to_local_frame(const Eigen::Vector2f& gpos) {
    auto position = pos();
    float theta = angle();
    auto rot = rotation(-theta * DEGREE_TO_RAD);
    return rot * (gpos - position);
}

Eigen::Vector2f HolonomicChassis::to_global_frame(const Eigen::Vector2f& gpos) {
    auto position = pos();
    float theta = angle();
    auto rot = rotation(theta * DEGREE_TO_RAD);
    return rot * gpos + position;
}