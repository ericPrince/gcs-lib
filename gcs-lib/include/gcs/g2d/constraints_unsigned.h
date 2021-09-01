#ifndef GCS_LIB_INCLUDE_GCS_G2D_CONSTRAINTS_UNSIGNED
#define GCS_LIB_INCLUDE_GCS_G2D_CONSTRAINTS_UNSIGNED

#include <ceres/ceres.h>
#include <gcs/core.h>
#include <gcs/g2d/geometry.h>

namespace gcs {

namespace g2d {

struct Distance : Constraint {
    Point* p1;
    Point* p2;
    Variable* d;
    Equation eqn;

    Distance(Point& p1, Point& p2, Variable& d)
        : p1{&p1}, p2{&p2}, d{&d}, eqn{{&p1.x, &p1.y, &p2.x, &p2.y, &d}} {}

    void add_cost_function(ceres::Problem& problem);
};

struct PointOnLine : Constraint {
    Point* p;
    Line* L;
    Equation eqn;

    PointOnLine(Point& p, Line& L)
        : p{&p}, L{&L}, eqn{{&p.x, &p.y, &L.p1.x, &L.p1.y, &L.p2.x, &L.p2.y}} {}

    void add_cost_function(ceres::Problem& problem);
};

struct OffsetLinePoint : Constraint {
    Line* L;
    Point* p;
    Variable* d;
    Equation eqn;

    OffsetLinePoint(Point& p, Line& L, Variable& d)
        : p{&p},
          L{&L},
          d{&d},
          eqn{{&p.x, &p.y, &L.p1.x, &L.p1.y, &L.p2.x, &L.p2.y, &d}} {}

    void add_cost_function(ceres::Problem& problem);
};

struct AngleBetweenLines : Constraint {
    Line* L1;
    Line* L2;
    Variable* a;

    AngleBetweenLines(Line& L1, Line& L2, Variable& a)
        : L1{&L1}, L2{&L2}, a{&a} {}

    std::vector<Equation> get_equations();

    void add_cost_function(ceres::Problem& problem);
};

struct AngleThreePoints : Constraint {
    Point* p1;
    Point* p2;
    Point* p3;
    Variable* a;

    AngleThreePoints(Point& p1, Point& p2, Point& p3, Variable& a)
        : p1{&p1}, p2{&p2}, p3{&p3}, a{&a} {}

    std::vector<Equation> get_equations();

    void add_cost_function(ceres::Problem& problem);
};

struct AngleLine : Constraint {
    Line* L;
    Variable* a;

    AngleLine(Line& L, Variable& a) : L{&L}, a{&a} {}

    std::vector<Equation> get_equations();

    void add_cost_function(ceres::Problem& problem);
};

struct PointOnCircle : Constraint {
    Point* p;
    Circle* c;

    PointOnCircle(Point& p, Circle& c) : p{&p}, c{&c} {}

    std::vector<Equation> get_equations();

    void add_cost_function(ceres::Problem& problem);
};

struct TangentLineCircle : Constraint {
    Line* L;
    Circle* c;

    TangentLineCircle(Line& L, Circle& c) : L{&L}, c{&c} {}

    std::vector<Equation> get_equations();

    void add_cost_function(ceres::Problem& problem);
};

struct LineLength : Constraint {
    Line* L;
    Variable* d;

    LineLength(Line& L, Variable& d) : L{&L}, d{&d} {}

    std::vector<Equation> get_equations();

    void add_cost_function(ceres::Problem& problem);
};

struct TangentCircles : Constraint {
    Circle* c1;
    Circle* c2;

    TangentCircles(Circle& c1, Circle& c2) : c1{&c1}, c2{&c2} {}

    std::vector<Equation> get_equations();

    void add_cost_function(ceres::Problem& problem);
};

}  // namespace g2d

}  // namespace gcs

#endif  // GCS_LIB_INCLUDE_GCS_G2D_CONSTRAINTS_UNSIGNED
