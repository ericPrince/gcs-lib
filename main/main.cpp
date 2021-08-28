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

    namespace cstr = constraints2d_unsigned;

    // f1
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::set_const_functor{0.0}),
        nullptr,
        &p0.x.value);

    // f2
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::set_const_functor{0.0}),
        nullptr,
        &p0.y.value);

    // f3
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::set_const_functor{r0}),
        nullptr,
        &c1.r.value);

    // f4
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::set_const_functor{d}),
        nullptr,
        &d1.value);

    // f5
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::set_const_functor{a}),
        nullptr,
        &a1.value);

    // f67
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::equate_functor{}),
        nullptr,
        &p0.x.value,
        &c1.p.x.value);
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::equate_functor{}),
        nullptr,
        &p0.y.value,
        &c1.p.y.value);

    // f8
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::difference_functor{}),
        nullptr,
        &p0.x.value,
        &p1.x.value,
        &dx.value);

    // f9
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::difference_functor{}),
        nullptr,
        &p0.y.value,
        &p1.y.value,
        &dy.value);

    // f10
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::angle_point_3_functor{}),
        nullptr,
        &p1.x.value,
        &p1.y.value,
        &p3.x.value,
        &p3.y.value,
        &p2.x.value,
        &p2.y.value,
        &a1.value);

    // f11
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::tangent_line_circle_functor{}),
        nullptr,
        &L1.p1.x.value,
        &L1.p1.y.value,
        &L1.p2.x.value,
        &L1.p2.y.value,
        &c1.p.x.value,
        &c1.p.y.value,
        &c1.r.value);

    // f12
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::point_on_circle_functor{}),
        nullptr,
        &p3.x.value,
        &p3.y.value,
        &c1.p.x.value,
        &c1.p.y.value,
        &c1.r.value);

    // f1314
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::equate_functor{}),
        nullptr,
        &L1.p1.x.value,
        &p3.x.value);
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::equate_functor{}),
        nullptr,
        &L1.p1.y.value,
        &p3.y.value);

    // f15
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::line_length_functor{}),
        nullptr,
        &L1.p1.x.value,
        &L1.p1.y.value,
        &L1.p2.x.value,
        &L1.p2.y.value,
        &d1.value);

    // f1617
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::equate_functor{}),
        nullptr,
        &L1.p2.x.value,
        &p2.x.value);
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::equate_functor{}),
        nullptr,
        &L1.p2.y.value,
        &p2.y.value);

    // f18
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::set_const_functor{d_x}),
        nullptr,
        &dx.value);

    // f19
    prb2.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::set_const_functor{d_y}),
        nullptr,
        &dy.value);

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
