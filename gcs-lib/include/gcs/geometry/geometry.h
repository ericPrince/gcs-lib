#ifndef GCS_GEOMETRY_H_
#define GCS_GEOMETRY_H_

#include <gcs/constraints/constraints2d_unsigned.h>
#include <gcs/solver/solve_elements.h>

#include <vector>

// geom 2D

namespace GCS {

struct Geometry {
    virtual std::vector<Variable*> get_variables() = 0;
};

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

//-----------------------------------------------------------------------------

struct Constraint {
    // virtual std::vector<Equation> get_equations() = 0;

    virtual void add_cost_function(ceres::Problem& problem) = 0;
};

// TODO: separation b/n solve nodes and data? or nah??

// TODO: auto-generate these
// - input
//   - ordered set of geometries/variables
//   - set of constraint functions/functors
// - output
//   - auto-gen structures
//   - ceres functors
//   - mapping of vars to functors
struct SetConstant : Constraint {
    double value;
    Variable* v;
    Equation eqn;

    SetConstant(Variable& v, double value) : v{&v}, value{value}, eqn{{&v}} {}

    void add_cost_function(ceres::Problem& problem);
};

struct Distance : Constraint {
    Point* p1;
    Point* p2;
    Variable* d;
    Equation eqn;

    Distance(Point& p1, Point& p2, Variable& d)
        : p1{&p1}, p2{&p2}, d{&d}, eqn{{&p1.x, &p1.y, &p2.x, &p2.y, &d}} {}

    void add_cost_function(ceres::Problem& problem);
};

struct Equate : Constraint {
    Variable* v1;
    Variable* v2;
    Equation eqn;

    Equate(Variable& v1, Variable& v2) : v1{&v1}, v2{&v2}, eqn{{&v1, &v2}} {}

    void add_cost_function(ceres::Problem& problem);
};

struct Difference : Constraint {
    Variable* v1;
    Variable* v2;
    Variable* d;
    Equation eqn;

    Difference(Variable& v1, Variable& v2, Variable& d)
        : v1{&v1}, v2{&v2}, d{&d}, eqn{{&v1, &v2, &d}} {}

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

}  //  namespace GCS

#endif  // GCS_GEOMETRY_H_
