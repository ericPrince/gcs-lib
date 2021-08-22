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
}
