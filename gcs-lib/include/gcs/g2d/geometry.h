#ifndef GCS_LIB_INCLUDE_GCS_G2D_GEOMETRY
#define GCS_LIB_INCLUDE_GCS_G2D_GEOMETRY

#include <gcs/core.h>

#include <vector>

namespace gcs {

namespace g2d {

struct Point : Geometry {
    Variable x;
    Variable y;

    Point(Variable x, Variable y) : x{x}, y{y} {}

    std::vector<Variable*> get_variables();
};

struct Line : Geometry {
    Point p1;
    Point p2;

    Line(Point p1, Point p2) : p1{p1}, p2{p2} {}

    std::vector<Variable*> get_variables();
};

struct Circle : Geometry {
    Point p;
    Variable r;

    Circle(Point p, Variable r) : p{p}, r{r} {}

    std::vector<Variable*> get_variables();
};

}  // namespace g2d

}  // namespace gcs

#endif  // GCS_LIB_INCLUDE_GCS_G2D_GEOMETRY
