#include "gcs/core/problem.h"

#include "gcs/core/split_equation_sets.h"

ceres::Solver::Summary gcs::single_solve(EquationSet& eqn_set) {
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

gcs::Problem::~Problem() {
    for (auto eq : equation_sets) {
        delete eq;
    }
}

template <>
bool gcs::Problem::add(Variable* var) {
    return add_variable(var);
}

template <>
bool gcs::Problem::add(Constraint* constraint) {
    return add_constraint(constraint);
}

template <>
bool gcs::Problem::add(Geometry* geom) {
    return add_geometry(geom);
}

bool gcs::Problem::add_variable(Variable* var) {
    variables.insert(var);
    return true;
}

bool gcs::Problem::add_constraint(Constraint* constraint) {
    constraints.insert(constraint);

    reset_to_single_equation_set();

    // for (auto& eq : constraint->get_equations()) {
    //     equations.push_back(std::move(eq));
    //     auto eq2 = &equations.back();

    //     for (auto& var : eq2->variables) {
    //         add_variable(var);
    //     }

    //     bool added = false;
    //     for (auto& eqn_set : equation_sets) {
    //         if (!eqn_set->is_constrained()) {
    //             eqn_set->add_equation(*eq2);
    //             added = true;
    //         }
    //     }
    //     if (!added) {
    //         auto eqn_set = new EquationSet{};
    //         equation_sets.insert(eqn_set);
    //         eqn_set->add_equation(*eq2);
    //         added = true;
    //     }
    // }

    split();
    solve();

    return true;
}

bool gcs::Problem::add_geometry(Geometry* geom) {
    geoms.insert(geom);
    return true;
}

template <>
bool gcs::Problem::remove(Variable* var) {
    return remove_variable(var);
}

template <>
bool gcs::Problem::remove(Constraint* constraint) {
    return remove_constraint(constraint);
}

template <>
bool gcs::Problem::remove(Geometry* geom) {
    // TODO: remove all constraints that use this geometry
    return remove_geometry(geom);
}

bool gcs::Problem::remove_variable(Variable* var) {
    variables.erase(var);
    // TODO: remove all constraints that use this variable
    return true;
}

bool gcs::Problem::remove_constraint(Constraint* constraint) {
    constraints.erase(constraint);
    reset_to_single_equation_set();
    split();
    solve();
    return true;
}

bool gcs::Problem::remove_geometry(Geometry* geom) {
    geoms.erase(geom);
    return true;
}

void gcs::Problem::reset_to_single_equation_set() {
    for (auto& eqs : equation_sets) {
        for (auto& var : eqs->get_variables()) {
            var->equations.clear();
        }
        delete eqs;
    }
    equation_sets.clear();

    auto eqn_set = new EquationSet{};
    equation_sets.insert(eqn_set);

    for (auto& constraint : constraints) {
        for (auto& eq : constraint->get_equations()) {
            eqn_set->add_equation(*eq);
        }
    }
    prereqs.clear();
    prereqs.emplace(eqn_set, decltype(prereqs)::mapped_type{});
    is_prereq_of.emplace(eqn_set, decltype(is_prereq_of)::mapped_type{});
}

void gcs::Problem::split() {
    auto old_equation_sets = equation_sets;
    equation_sets.clear();
    prereqs.clear();
    is_prereq_of.clear();

    for (auto& eq : old_equation_sets) {
        auto split_sets = gcs::split(*eq);
        delete eq;

        for (auto& eq2 : split_sets) {
            auto eq3 = new EquationSet{std::move(eq2)};
            equation_sets.insert(eq3);
            prereqs.emplace(eq3, decltype(prereqs)::mapped_type{});
            is_prereq_of.emplace(eq3, decltype(is_prereq_of)::mapped_type{});
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
                assert(eqn_set->equations.find(eq) == eqn_set->equations.end());
                assert(dep_eqn_set != eqn_set);

                // if (dep_eqn_set == eqn_set) {
                //     continue;
                // }

                prereqs[dep_eqn_set].insert(eqn_set);
                is_prereq_of[eqn_set].insert(dep_eqn_set);
            }
        }
    }
}

void gcs::Problem::solve(size_t pool_size) {
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
    std::function<void(EquationSet*)> solve_func = [&](EquationSet* eqn_set) {
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
                    boost::asio::post(pool, std::bind(solve_func, req_by));
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
