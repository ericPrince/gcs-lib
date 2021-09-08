#ifndef GCS_G2D_GEOMETRY
#define GCS_G2D_GEOMETRY

#include <ceres/ceres.h>

#include <vector>

#include "gcs/core/core.h"

namespace gcs {

namespace g2d {

struct Point : gcs::Geometry {
    gcs::Variable x;
    gcs::Variable y;

    Point(gcs::Variable x, gcs::Variable y) : x{x}, y{y} {}

    std::vector<gcs::Variable*> get_variables() { return {&x, &y}; }
};

struct Line : gcs::Geometry {
    gcs::g2d::Point p1;
    gcs::g2d::Point p2;

    Line(gcs::g2d::Point p1, gcs::g2d::Point p2) : p1{p1}, p2{p2} {}

    std::vector<gcs::Variable*> get_variables() {
        return {&p1.x, &p1.y, &p2.x, &p2.y};
    }
};

struct Circle : gcs::Geometry {
    gcs::g2d::Point center;
    gcs::Variable radius;

    Circle(gcs::g2d::Point center, gcs::Variable radius)
        : center{center}, radius{radius} {}

    std::vector<gcs::Variable*> get_variables() {
        return {&center.x, &center.y, &radius};
    }
};

}  // namespace g2d

}  // namespace gcs

#endif  // GCS_G2D_GEOMETRY