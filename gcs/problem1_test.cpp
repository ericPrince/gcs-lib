#include <ceres/ceres.h>

#include <boost/asio.hpp>
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

// TODO: move this to gcs/core
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

    ~GcsProblem() {
        for (auto eq : equation_sets) {
            delete eq;
        }
    }

    void split() {
        auto old_equation_sets = equation_sets;
        equation_sets = {};
        prereqs = {};
        is_prereq_of = {};

        for (auto& eq : old_equation_sets) {
            auto split_sets = gcs::split(*eq);
            delete eq;

            for (auto& eq2 : split_sets) {
                auto eq3 = new EquationSet{std::move(eq2)};
                equation_sets.insert(eq3);
                prereqs.emplace(eq3, decltype(prereqs)::mapped_type{});
                is_prereq_of.emplace(eq3,
                                     decltype(is_prereq_of)::mapped_type{});
            }
        }

        // which equation set does each equation belong to (TODO: track this in
        // the equation object?)
        std::unordered_map<Equation*, EquationSet*> containing_set = {};

        for (auto& eqn_set : equation_sets) {
            for (auto& eq : eqn_set->equations) {
                containing_set.emplace(eq, eqn_set);
            }
        }

        // fill out the dependencies/prereqs between equation sets
        for (auto& eqn_set : equation_sets) {
            // from EquationSet::set_solved():
            //   at the end, a variable will point to equations that require it
            //   in order to be solved and an equation will point to the
            //   variables it solves for

            for (auto& var : eqn_set->get_variables()) {
                // var is solved by this equation set
                for (auto& eq : var->equations) {
                    // eq needs var to be calculated in order to be solvable
                    // so the eqn set that uses eq has this eqn set as a dep
                    auto dep_eqn_set = containing_set[eq];
                    assert(dep_eqn_set != eqn_set);

                    prereqs[dep_eqn_set].insert(eqn_set);
                    is_prereq_of[eqn_set].insert(dep_eqn_set);
                }
            }
        }
    }

    void solve(size_t pool_size = 0) {
        if (pool_size == 0) {
            pool_size = std::thread::hardware_concurrency();
        }
        if (pool_size > equation_sets.size()) {
            pool_size = equation_sets.size();
        }

        // track equation set dependencies as they get solved
        auto solve_prereqs = prereqs;
        auto solve_is_prereq_of = is_prereq_of;
        auto not_solved = equation_sets;

        // set up a thread pool
        boost::asio::thread_pool pool{pool_size};
        std::mutex mtx;

        // this function will be passed to the thread pool
        //
        // It runs ceres solver on the problem, then updates the active equation
        // set dependencies. If this update makes it so another equation set
        // does not depend on anything else, it gets passed to the thread pool
        // to solve and do the same thing
        std::function<void(EquationSet*)> solve_func =
            [&](EquationSet* eqn_set) {
                single_solve(*eqn_set);

                // once the equation set has been solved:
                {
                    std::lock_guard<std::mutex> lock{mtx};

                    for (auto& req_by : solve_is_prereq_of[eqn_set]) {
                        // the just-solved equation is no longer holding up
                        // its dependencies
                        solve_prereqs[req_by].erase(eqn_set);

                        // if the dependency isn't waiting on anything
                        // else, it is ready to solve
                        if (solve_prereqs[req_by].size() == 0 &&
                            not_solved.find(req_by) != not_solved.end()) {
                            not_solved.erase(eqn_set);
                            boost::asio::post(pool,
                                              std::bind(solve_func, req_by));
                        }
                    }
                }
            };

        for (auto& eq_pair : solve_prereqs) {
            auto& eq = eq_pair.first;
            auto& pre = eq_pair.second;

            if (pre.size() == 0) {
                not_solved.erase(eq);
                boost::asio::post(pool, std::bind(solve_func, eq));
            }
        }

        pool.join();

        for (auto& eq_pair : solve_prereqs) {
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

    // make problem, split, and solve
    gcs::GcsProblem gcs_problem{};
    gcs_problem.equation_sets.insert(new gcs::EquationSet{equation_set});
    gcs_problem.split();
    std::cout << "Split using GcsProblem: " << gcs_problem.equation_sets.size()
              << std::endl;

    gcs_problem.solve();

    std::cout << "p2.x: " << p2.x.value << std::endl;
    std::cout << "p2.y: " << p2.y.value << std::endl;
    std::cout << "p3.x: " << p3.x.value << std::endl;
    std::cout << "p3.y: " << p3.y.value << std::endl;
    std::cout << "c1.r:  " << c1.radius.value << std::endl;
}
