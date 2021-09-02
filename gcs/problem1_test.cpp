#include <ceres/ceres.h>

#include <iostream>

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

    constraints.push_back(new gcs::SetConstant{p1.x, 0.0});      // f1
    constraints.push_back(new gcs::SetConstant{p1.y, 0.0});      // f2
    constraints.push_back(new gcs::SetConstant{c1.r, r0});       // f3
    constraints.push_back(new gcs::SetConstant{d1, d});          // f4
    constraints.push_back(new gcs::SetConstant{a1, a});          // f5
    constraints.push_back(new gcs::Equate{p0.x, c1.p.x});        // f6
    constraints.push_back(new gcs::Equate{p0.y, c1.p.y});        // f7
    constraints.push_back(new gcs::Difference{p0.x, p1.x, dx});  // f8
    constraints.push_back(new gcs::Difference{p0.y, p1.y, dy});  // f9
    constraints.push_back(
        new gcs::g2d::AngleThreePoints{p1, p3, p2, a1});             // f10
    constraints.push_back(new gcs::g2d::TangentLineCircle{L1, c1});  // f11
    constraints.push_back(new gcs::g2d::PointOnCircle{p3, c1});      // f12
    constraints.push_back(new gcs::Equate{L1.p1.x, p3.x});           // f13
    constraints.push_back(new gcs::Equate{L1.p1.y, p3.y});           // f14
    constraints.push_back(new gcs::g2d::LineLength{L1, d1});         // f15
    constraints.push_back(new gcs::Equate{L1.p2.x, p2.x});           // f16
    constraints.push_back(new gcs::Equate{L1.p2.y, p2.y});           // f17
    constraints.push_back(new gcs::SetConstant{dx, d_x});            // f18
    constraints.push_back(new gcs::SetConstant{dy, d_y});            // f19

    ceres::Problem problem{};

    for (auto& constraint : constraints) {
        constraint->add_cost_function(problem);
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
    std::cout << "c1.r:  " << c1.r.value << std::endl;
}
