#include <ceres/ceres.h>

#include <iostream>

#include "gcs/basic/constraints.h"
#include "gcs/core/core.h"
#include "gcs/g2d/g2d.h"

namespace gcs {

ceres::Solver::Summary single_solve(EquationSet& eqn_set) {
    ceres::Problem problem = {};

    // add all residual blocks
    for (auto& eqn : eqn_set.equations) {
        eqn->make_residual_ftor(problem);
    }

    // hold parameter blocks constant if necessary
    std::unordered_set<double*> variable_parameter_blocks = {};
    for (auto& var : eqn_set.get_variables()) {
        variable_parameter_blocks.insert(&var->value);
    }

    std::vector<double*> all_parameter_blocks = {};
    problem.GetParameterBlocks(&all_parameter_blocks);

    for (auto param_block : all_parameter_blocks) {
        if (variable_parameter_blocks.find(param_block) ==
            variable_parameter_blocks.end()) {
            // parameter block isn't variable
            problem.AddParameterBlock(param_block, 1);
            problem.SetParameterBlockConstant(param_block);
        }
    }

    // apply settings
    ceres::Solver::Options options = {};
    options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    return summary;
}

struct GcsProblem {
    std::unordered_set<Variable*> variables;
    std::unordered_set<Geometry*> geoms;
    std::unordered_set<Constraint*> constraints;

    // std::unordered_map<Variable*, EquationSet*> solves_for_variable;
    // std::unordered_map<EquationSet*, std::unordered_set<Variable*>>
    // required_for_;

    // set of all EquationSets
    std::unordered_set<EquationSet*> equation_sets;
    // All items in the set must be solved before the key can be solved
    std::unordered_map<EquationSet*, std::unordered_set<EquationSet*>> prereqs;
    // The key must be solved before any of the values in the set can be solved
    std::unordered_map<EquationSet*, std::unordered_set<EquationSet*>>
        is_prereq_of;

    template <typename T>
    bool add(T& item);

    bool add_variable(Variable& var);
    bool add_geometry(Geometry& geom);
    bool add_constraint(Constraint& constraint);

    template <typename T>
    bool remove(T& item);

    bool remove_variable(Variable& var);
    bool remove_geometry(Geometry& geom);
    bool remove_constraint(Constraint& constraint);

    void solve() {
        auto solve_prereqs = prereqs;
        auto solve_is_prereq_of = is_prereq_of;

        std::queue<EquationSet*> ready_to_solve = {};

        for (auto& eq_pair : solve_prereqs) {
            auto& eq = eq_pair.first;
            auto& pre = eq_pair.second;

            if (pre.size() == 0) {
                ready_to_solve.push(eq);
            }
        }

        while (ready_to_solve.size() > 0) {
            auto& eq = ready_to_solve.front();
            ready_to_solve.pop();

            // TODO: actually solve the problem and update the variable values

            // once the equation set has been solved:
            for (auto& eq_pair : solve_is_prereq_of) {
                for (auto& req_by : eq_pair.second) {
                    // the just-solved equation is no longer holding up its
                    // dependencies
                    solve_is_prereq_of[req_by].erase(eq);

                    // if the dependency isn't waiting on anything else, it is
                    // ready to solve
                    if (solve_is_prereq_of[req_by].size() == 0) {
                        ready_to_solve.push(req_by);
                    }
                }
            }
        }

        for (auto& eq_pair : solve_is_prereq_of) {
            assert(eq_pair.second.size() == 0 &&
                   "There should be no equation sets waiting on prereqs");
        }
    }
};

}  // namespace gcs

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

    // build single equation set from problem
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

    // ceres::Problem problem{};

    // for (auto& constraint : constraints) {
    //     // constraint->add_to_problem(problem);
    //     for (auto& eqn : constraint->get_equations()) {
    //         eqn.make_residual_ftor(problem);
    //     }
    // }

    // ceres::Solver::Options options{};
    // options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
    // options.minimizer_progress_to_stdout = true;

    // ceres::Solver::Summary summary;
    // ceres::Solve(options, &problem, &summary);

    ceres::Solver::Summary summary;
    // note: commenting below line out to test solving smaller equation sets in
    // order

    // auto summary = gcs::single_solve(equation_set);

    // std::cout << summary.BriefReport() << std::endl;
    // std::cout << "p2.x: " << p2.x.value << std::endl;
    // std::cout << "p2.y: " << p2.y.value << std::endl;
    // std::cout << "p3.x: " << p3.x.value << std::endl;
    // std::cout << "p3.y: " << p3.y.value << std::endl;
    // std::cout << "c1.r:  " << c1.radius.value << std::endl;

    // test equation set splitting
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

    for (auto& sub_eqn_set : split_equation_sets) {
        summary = gcs::single_solve(sub_eqn_set);

        std::cout << summary.BriefReport() << std::endl;
        std::cout << "p2.x: " << p2.x.value << std::endl;
        std::cout << "p2.y: " << p2.y.value << std::endl;
        std::cout << "p3.x: " << p3.x.value << std::endl;
        std::cout << "p3.y: " << p3.y.value << std::endl;
        std::cout << "c1.r:  " << c1.radius.value << std::endl;
    }
}
