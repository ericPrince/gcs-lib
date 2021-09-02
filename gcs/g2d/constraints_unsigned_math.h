#ifndef GCS_G2D_CONSTRAINTS_UNSIGNED_MATH
#define GCS_G2D_CONSTRAINTS_UNSIGNED_MATH

#include <ceres/ceres.h>

#include "gcs/core/constraints.h"

namespace gcs {

//! @brief Set the distance between 2 points
template <typename T>
T distance(const T& x1, const T& y1, const T& x2, const T& y2, const T& d) {
    return ceres::hypot(x2 - x1, y2 - y1) - ceres::abs(d);
}

CSTR_CREATE_FUNCTOR_(distance, 5);

//! @brief Constrain a point to lie on a line
//! @note (x3, y3) is the point
template <typename T>
T point_on_line(const T& x1,
                const T& y1,
                const T& x2,
                const T& y2,
                const T& x3,
                const T& y3) {
    return (y3 - y1) * (x2 - x1) - (x3 - x1) * (y2 - y1);
}

CSTR_CREATE_FUNCTOR_(point_on_line, 6);

//! @brief Set the offset between a line and a point
template <typename T>
T offset_line_point(const T& x1,
                    const T& y1,
                    const T& x2,
                    const T& y2,
                    const T& x3,
                    const T& y3,
                    const T& d) {
    const auto dL = ceres::hypot(x2 - x1, y2 - y1);

    return std::min((dL * (y3 - y1) + d * (x2 - x1)) * (x2 - x1) -
                        (dL * (x3 - x1) - d * (y2 - y1)) * (y2 - y1),

                    (dL * (y3 - y1) - d * (x2 - x1)) * (x2 - x1) -
                        (dL * (x3 - x1) + d * (y2 - y1)) * (y2 - y1));
}

CSTR_CREATE_FUNCTOR_(offset_line_point, 7);

//! @brief Set the angle between 2 lines
template <typename T>
T angle_point_4(const T& x1,
                const T& y1,
                const T& x2,
                const T& y2,
                const T& x3,
                const T& y3,
                const T& x4,
                const T& y4,
                const T& a) {
    // TODO: could treat as a re-parametrization??
    return ceres::abs(ceres::atan2(x4 - x3, y4 - y3) -
                      ceres::atan2(x2 - x1, y2 - y1)) -
           a;
}

CSTR_CREATE_FUNCTOR_(angle_point_4, 9);

//! @brief Set the angle between 2 lines
template <typename T>
T angle_point_3(const T& x1,
                const T& y1,
                const T& x2,
                const T& y2,
                const T& x3,
                const T& y3,
                const T& a) {
    return ceres::abs(ceres::atan2(x3 - x2, y3 - y2) -
                      ceres::atan2(x1 - x2, y1 - y2)) -
           a;

    // return angle_point_4(x1, y1,
    //                      x2, y2,
    //                      x2, y2,
    //                      x3, y3,
    //                      a);
}

CSTR_CREATE_FUNCTOR_(angle_point_3, 7);

//! @brief Set the angle of a line
template <typename T>
T angle_point_2(
    const T& x1, const T& y1, const T& x2, const T& y2, const T& a) {
    return ceres::abs(ceres::atan2(x1 - x2, y1 - y2)) - a;  // TODO: abs?
}

CSTR_CREATE_FUNCTOR_(angle_point_2, 5);

//---------------------------
// Derived constraints
//---------------------------

//! @brief Constrain a point to a circle
template <typename T>
T point_on_circle(
    const T& x1, const T& y1, const T& x2, const T& y2, const T& r) {
    return distance(x1, y1, x2, y2, r);
}

CSTR_CREATE_FUNCTOR_(point_on_circle, 5);

//! @brief Set tangency between a line and a circle
template <typename T>
T tangent_line_circle(const T& x1,
                      const T& y1,
                      const T& x2,
                      const T& y2,
                      const T& xc,
                      const T& yc,
                      const T& r) {
    return offset_line_point(x1, y1, x2, y2, xc, yc, r);
}

CSTR_CREATE_FUNCTOR_(tangent_line_circle, 7);

//! @brief Set the length of a line
template <typename T>
T line_length(const T& x1, const T& y1, const T& x2, const T& y2, const T& d) {
    return distance(x1, y1, x2, y2, d);
}

CSTR_CREATE_FUNCTOR_(line_length, 5);

//! @brief Set tangency between 2 circles
template <typename T>
T tangent_circles(const T& x1,
                  const T& y1,
                  const T& x2,
                  const T& r1,
                  const T& y2,
                  const T& r2) {
    // TODO: can control tangency type
    return ceres::fmin(distance(x1, y1, x2, y2, r1 + r2),
                       distance(x1, y1, x2, y2, r1 - r2));
}

CSTR_CREATE_FUNCTOR_(tangent_circles, 6);

}  // namespace gcs

#endif  // GCS_G2D_CONSTRAINTS_UNSIGNED_MATH
