#ifndef GCS_GEOMETRY_H_
#define GCS_GEOMETRY_H_


#include <vector>

#include <gcs/constraints/constraints2d_unsigned.h>
#include <gcs/solver/solve_elements.h>


// geom 2D


namespace GCS {


struct Geometry {
    virtual std::vector<Variable*> get_variables() = 0;
};


struct Point : Geometry {
    Variable x;
    Variable y;

    std::vector<Variable*> get_variables();
};


struct Line : Geometry {
    Point p1;
    Point p2;

    std::vector<Variable*> get_variables();
};


struct Circle : Geometry {
    Point p;
    Variable r;

    std::vector<Variable*> get_variables();
};


//-----------------------------------------------------------------------------


struct Constraint {
    //virtual std::vector<Equation> get_equations() = 0;
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
    double    value;
    Variable* v;
    Equation  eqn;

    SetConstant(Variable& v, double value)
        : v{ &v }
        , value{ value }
        , eqn{ {&v} }
    {}
};


struct Distance : Constraint {
    Point*    p1;
    Point*    p2;
    Variable* d;
    Equation  eqn;

    Distance(Point& p1, Point& p2, Variable& d)
        : p1{ &p1 }
        , p2{ &p2 }
        , d{ &d }
        , eqn{ {&p1.x, &p1.y, &p2.x, &p2.y, &d} }
    {}
};


struct Equate : Constraint {
    Variable* v1;
    Variable* v2;
    Equation  eqn;

    Equate(Variable& v1, Variable& v2)
        : v1{ &v1 }
        , v2{ &v2 }
        , eqn{ {&v1, &v2} }
    {}
};


struct Difference : Constraint {
    Variable* v1;
    Variable* v2;
    Variable* d;
    Equation  eqn;

    Difference(Variable& v1, Variable& v2, Variable& d)
        : v1{ &v1 }
        , v2{ &v2 }
        , d{ &d }
        , eqn{ {&v1, &v2, &d} }
    {}
};


struct PointOnLine : Constraint {
    Point*   p;
    Line*    L;
    Equation eqn;

    PointOnLine(Point& p, Line& L)
        : p{ &p }
        , L{ &L }
        , eqn{ {&p.x, &p.y, &L.p1.x, &L.p1.y, &L.p2.x, &L.p2.y} }
    {}
};


struct OffsetLinePoint : Constraint {
    Line*     L;
    Point*    p;
    Variable* d;
    Equation  eqn;

    OffsetLinePoint(Point& p, Line& L, Variable& d)
        : p{ &p }
        , L{ &L }
        , d{ &d }
        , eqn{ {&p.x, &p.y, &L.p1.x, &L.p1.y, &L.p2.x, &L.p2.y, &d} }
    {}
};


struct AngleBetweenLines : Constraint {
    Line*     L1;
    Line*     L2;
    Variable* a;

    std::vector<Equation> get_equations();
};


struct AngleThreePoints : Constraint {
    Point*    p1;
    Point*    p2;
    Point*    p3;
    Variable* a;

    std::vector<Equation> get_equations();
};


struct AngleLine : Constraint {
    Line*     L;
    Variable* a;

    std::vector<Equation> get_equations();
};


struct PointOnCircle : Constraint {
    Point*  p;
    Circle* c;

    std::vector<Equation> get_equations();
};


struct TangentLineCircle : Constraint {
    Line*   L;
    Circle* c;

    std::vector<Equation> get_equations();
};


struct LineLength : Constraint {
    Line*     L;
    Variable* d;

    std::vector<Equation> get_equations();
};


struct TangentCircles : Constraint {
    Circle* c1;
    Circle* c2;

    std::vector<Equation> get_equations();
};


} //  namespace GCS


#endif  // GCS_GEOMETRY_H_
