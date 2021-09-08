#include <ceres/ceres.h>

#include <iostream>

#include "gcs/basic/constraints.h"
#include "gcs/core/core.h"
#include "gcs/g2d/g2d.h"

int main(int argc, char** argv) {
    const double r0 = 1.5;
    const double d = 3.0;
    const double a = M_PI / 6.0;
    const double d_x = 3.0;
    const double d_y = 1.0;

    gcs::Variable v1{0.0};

    gcs::g2d::Point p0 = {0.0, 0.0};
    gcs::g2d::Point p1 = {1.0, 1.0};
    gcs::g2d::Point p2 = {2.0, 2.0};
    gcs::g2d::Point p3 = {3.0, 3.0};
    gcs::g2d::Circle c1 = {{0.0, 0.0}, 1.0};
    gcs::g2d::Line L1 = {{1.0, 1.0}, {3.0, 3.0}};

    gcs::Variable d1 = 1.0;
    gcs::Variable a1 = M_PI / 4.0;
    gcs::Variable dx = 2.0;
    gcs::Variable dy = 1.0;

    std::vector<gcs::Constraint*> constraints = {};

    constraints.push_back(new gcs::basic::SetConstant{p1.x, 0.0});      // f1
    constraints.push_back(new gcs::basic::SetConstant{p1.y, 0.0});      // f2
    constraints.push_back(new gcs::basic::SetConstant{c1.radius, r0});  // f3
    constraints.push_back(new gcs::basic::SetConstant{d1, d});          // f4
    constraints.push_back(new gcs::basic::SetConstant{a1, a});          // f5
    constraints.push_back(new gcs::basic::Equate{p0.x, c1.center.x});   // f6
    constraints.push_back(new gcs::basic::Equate{p0.y, c1.center.y});   // f7
    constraints.push_back(new gcs::basic::Difference{p0.x, p1.x, dx});  // f8
    constraints.push_back(new gcs::basic::Difference{p0.y, p1.y, dy});  // f9
    constraints.push_back(
        new gcs::g2d::AngleThreePoints{p1, p3, p2, a1});             // f10
    constraints.push_back(new gcs::g2d::TangentLineCircle{L1, c1});  // f11
    constraints.push_back(new gcs::g2d::PointOnCircle{p3, c1});      // f12
    constraints.push_back(new gcs::basic::Equate{L1.p1.x, p3.x});    // f13
    constraints.push_back(new gcs::basic::Equate{L1.p1.y, p3.y});    // f14
    constraints.push_back(new gcs::g2d::LineLength{L1, d1});         // f15
    constraints.push_back(new gcs::basic::Equate{L1.p2.x, p2.x});    // f16
    constraints.push_back(new gcs::basic::Equate{L1.p2.y, p2.y});    // f17
    constraints.push_back(new gcs::basic::SetConstant{dx, d_x});     // f18
    constraints.push_back(new gcs::basic::SetConstant{dy, d_y});     // f19

    ceres::Problem problem{};

    for (auto& constraint : constraints) {
        constraint->add_to_problem(problem);
    }

    ceres::Solver::Options options{};
    options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "p2.x: " << p2.x.value << std::endl;
    std::cout << "p2.y: " << p2.y.value << std::endl;
    std::cout << "p3.x: " << p3.x.value << std::endl;
    std::cout << "p3.y: " << p3.y.value << std::endl;
    std::cout << "c1.r:  " << c1.radius.value << std::endl;

    // test equation set splitting

    gcs::EquationSet equation_set = {};
    std::vector<gcs::Equation> equations;

    for (auto& constraint : constraints) {
        for (auto& eqn : constraint->get_equations()) {
            equations.push_back(std::move(eqn));
        }
    }

    for (auto& eqn : equations) {
        equation_set.add_equation(eqn);
    }

    auto split_equation_sets = gcs::split(equation_set);

    std::cout << std::endl
              << "Split equation sets: " << split_equation_sets.size()
              << std::endl;
    std::cout << "  number of equations: " << equation_set.equations.size()
              << std::endl;
    std::cout << "  number of variables: "
              << equation_set.get_variables().size() << std::endl;
    std::cout << "  degrees of freedom: " << equation_set.degrees_of_freedom()
              << std::endl;
}
