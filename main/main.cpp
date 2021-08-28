#include <ceres/ceres.h>
#include <gcs/constraints/constraints2d_unsigned.h>
#include <gcs/geometry/geometry.h>
#include <gcs/problems/problem.h>
#include <gcs/problems/problem1.h>
#include <gcs/solver/solve_elements.h>

#include <iostream>

//#include <gcs/geometry/geometry.h>

int main(int argc, char** argv) {
    // std::cout << x << std::endl;
    /*
        problems::Problem p1 = problems::problem1();

        GCS::EquationSet eqn_set;

        std::cout << "=====" << std::endl;

        for (auto& eqn : p1.eqns) {
            eqn_set.add_equation(eqn.second);

            std::cout << eqn.first << ": ";
            for (auto& var : eqn.second.variables) {
                std::cout << p1.var_names.at(var) << ", ";
            }
            std::cout << std::endl;

            std::cout << "DOF: " << eqn_set.degrees_of_freedom() << std::endl;
            std::cout << "-----" << std::endl;
        }

        std::cout << "=====" << std::endl;
        std::cout << "Final DOF: " << eqn_set.degrees_of_freedom() << std::endl;
        std::cout << "=====" << std::endl;

        for (auto& var : p1.vars) {
            std::cout << var.first << ": ";

            for (auto& eqn : var.second.equations) {
                std::cout << p1.eqn_names.at(eqn) << ", ";
            }

            std::cout << std::endl;
        }

        const auto eqn_sets = GCS::split(eqn_set);

        std::cout << "=====" << std::endl;

        for (auto& eqn_set : eqn_sets) {
            std::cout << "eqns solved: ";
            for (auto& eqn : eqn_set.equations) {
                std::cout << p1.eqn_names.at(eqn) << ", ";
            }
            std::cout << std::endl;

            std::unordered_set<GCS::Variable*> vars_req;
            for (auto& var_pair : p1.vars) {
                auto& var = var_pair.second;

                if (eqn_set.get_variables().count(&var)) {
                    continue;
                }

                for (auto& eqn : var.equations) {
                    if (eqn_set.equations.count(eqn)) {
                        vars_req.insert(&var);
                        break;
                    }
                }
            }

            std::cout << "vars required to solve this: ";
            for (auto& var : vars_req) {
                std::cout << p1.var_names.at(var) << ", ";
            }
            std::cout << std::endl;

            std::cout << "vars solved for: ";
            for (auto& var : eqn_set.get_variables()) {
                std::cout << p1.var_names.at(var) << ", ";
            }
            std::cout << std::endl;

            std::unordered_set<GCS::Equation*> eqns_req{};
            for (auto& var : eqn_set.get_variables()) {
                for (auto& eqn : var->equations) {
                    eqns_req.insert(eqn);
                }
            }

            std::cout << "eqns that use vars solved here: ";
            for (auto& eqn : eqns_req) {
                std::cout << p1.eqn_names.at(eqn) << ", ";
            }
            std::cout << std::endl;

            std::cout << "-----" << std::endl;
        }

        // Constraints

        auto x_test =
            constraints2d_unsigned::distance<double>(1.0, 2.0, 3.0, 4.0, 2.0);
        std::cout << x_test << std::endl;

        // Ceres Test

        namespace cstr = constraints2d_unsigned;

        // google::InitGoogleLogging(argv[0]);

        double x1 = 1.0;
        double y1 = 1.0;
        double x2 = 2.0;
        double y2 = 2.0;
        double d = 1.5;

        ceres::Problem problem{};

        problem.AddResidualBlock(
            cstr::create_scalar_autodiff(new cstr::distance_functor{}),
            nullptr,
            &x1,
            &y1,
            &x2,
            &y2,
            &d);

        problem.AddResidualBlock(
            cstr::create_scalar_autodiff(new cstr::set_const_functor{1.0}),
            nullptr,
            &x1);

        problem.AddResidualBlock(
            cstr::create_scalar_autodiff(new cstr::set_const_functor{2.0}),
            nullptr,
            &y1);

        problem.AddResidualBlock(
            cstr::create_scalar_autodiff(new cstr::set_const_functor{3.0}),
            nullptr,
            &x2);

        problem.AddResidualBlock(
            cstr::create_scalar_autodiff(new cstr::set_const_functor{4.0}),
            nullptr,
            &y2);

        ceres::Solver::Options options{};
        options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary;
        Solve(options, &problem, &summary);

        std::cout << summary.BriefReport() << std::endl;
        std::cout << "x1: " << x1 << std::endl;
        std::cout << "y1: " << y1 << std::endl;
        std::cout << "x2: " << x2 << std::endl;
        std::cout << "y2: " << y2 << std::endl;
        std::cout << "d:  " << d << std::endl;

        // ---------------------------------------------
    */
    ceres::Problem prb2{};

    const double r0 = 1.5;
    const double d = 3.0;
    const double a = M_PI / 6.0;
    const double d_x = 3.0;
    const double d_y = 1.0;

    GCS::Variable v1{0.0};

    GCS::Point p0 = {0.0, 0.0};
    GCS::Point p1 = {1.0, 1.0};
    GCS::Point p2 = {2.0, 2.0};
    GCS::Point p3 = {3.0, 3.0};
    GCS::Circle c1 = {{0.0, 0.0}, 1.0};
    GCS::Line L1 = {{1.0, 1.0}, {3.0, 3.0}};

    GCS::Variable d1 = 1.0;
    GCS::Variable a1 = M_PI / 4.0;
    GCS::Variable dx = 2.0;
    GCS::Variable dy = 1.0;

    std::vector<GCS::Constraint*> constraints = {};

    constraints.push_back(new GCS::SetConstant{p1.x, 0.0});            // f1
    constraints.push_back(new GCS::SetConstant{p1.y, 0.0});            // f2
    constraints.push_back(new GCS::SetConstant{c1.r, r0});             // f3
    constraints.push_back(new GCS::SetConstant{d1, d});                // f4
    constraints.push_back(new GCS::SetConstant{a1, a});                // f5
    constraints.push_back(new GCS::Equate{p0.x, c1.p.x});              // f6
    constraints.push_back(new GCS::Equate{p0.y, c1.p.y});              // f7
    constraints.push_back(new GCS::Difference{p0.x, p1.x, dx});        // f8
    constraints.push_back(new GCS::Difference{p0.y, p1.y, dy});        // f9
    constraints.push_back(new GCS::AngleThreePoints{p1, p3, p2, a1});  // f10
    constraints.push_back(new GCS::TangentLineCircle{L1, c1});         // f11
    constraints.push_back(new GCS::PointOnCircle{p3, c1});             // f12
    constraints.push_back(new GCS::Equate{L1.p1.x, p3.x});             // f13
    constraints.push_back(new GCS::Equate{L1.p1.y, p3.y});             // f14
    constraints.push_back(new GCS::LineLength{L1, d1});                // f15
    constraints.push_back(new GCS::Equate{L1.p2.x, p2.x});             // f16
    constraints.push_back(new GCS::Equate{L1.p2.y, p2.y});             // f17
    constraints.push_back(new GCS::SetConstant{dx, d_x});              // f18
    constraints.push_back(new GCS::SetConstant{dy, d_y});              // f19

    for (auto& constraint : constraints) {
        constraint->add_cost_function(prb2);
    }

    ceres::Solver::Options options{};
    options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    Solve(options, &prb2, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "p2.x: " << p2.x.value << std::endl;
    std::cout << "p2.y: " << p2.y.value << std::endl;
    std::cout << "p3.x: " << p3.x.value << std::endl;
    std::cout << "p3.y: " << p3.y.value << std::endl;
    std::cout << "c1.r:  " << c1.r.value << std::endl;
}
