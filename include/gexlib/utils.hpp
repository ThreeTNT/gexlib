#pragma once

#include "Eigen/Core"
#include <concepts>

namespace gexlib {

constexpr float DEGREE_TO_RAD = M_PI / 180;
constexpr float CENTIGREE_TO_RAD = 100 * DEGREE_TO_RAD;
constexpr float RAD_TO_DEGREE = 180 / M_PI;

template<std::floating_point T>
inline auto rotation(T theta) -> Eigen::Matrix<T, 2, 2> {
    T c = (T) cos(theta);
    T s = (T) sin(theta);
    return Eigen::Matrix<T, 2, 2> {
        {c, -s}, 
        {s, c}
    };
}

template<std::floating_point T>
inline auto unit_vector(T theta) -> Eigen::Vector2<T> {
    T c = (T) cos(theta);
    T s = (T) sin(theta);
    return Eigen::Vector2<T> { c, s };
}
}