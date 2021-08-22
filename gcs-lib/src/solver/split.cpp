#include <gcs/solver/solve_elements.h>

namespace GCS {

std::vector<EquationSet> split(EquationSet& equation_set) {
    // set of split up equation sets - this will be returned
    std::vector<EquationSet> solve_sets{};

    // keeper of eqn sets generated in this function
    std::unordered_set<EquationSet> visited_equation_sets{};

    // priority queue where pop returns the equation set
    // that is "closest" to being solvable
    std::priority_queue<EquationSet,
                        std::vector<EquationSet>,
                        EquationSet::Compare>
        pq{};

    // unconstrained equation set of leftover equations
    EquationSet unconstrained_equation_set{equation_set.equations};

    // initialize the pq with single-equation sets
    for (auto& eqn : equation_set.equations) {
        auto eqn_set = EquationSet{}.add_equation(*eqn);

        visited_equation_sets.emplace(eqn_set);
        pq.push(std::move(eqn_set));
    }

    while (!pq.empty()) {
        auto current_equation_set = std::move(pq.top());
        pq.pop();

        if (current_equation_set.is_constrained()) {
            // empty the pq - all of its entries need to be updated
            std::vector<EquationSet> temp_eqn_sets{};

            while (!pq.empty()) {
                // remove from the pq
                auto eqn_set = std::move(pq.top());
                pq.pop();

                // remove all equations in the just-solved equation
                // set from all equation sets in the pq
                for (auto& equation : current_equation_set.equations) {
                    eqn_set.equations.erase(equation);
                }

                // place in the temp vector - which will all be added to
                // the pq after all updates have happened
                // note: don't add back to pq if there are no equations left
                if (!eqn_set.equations.empty()) {
                    temp_eqn_sets.push_back(std::move(eqn_set));
                }
            }

            // remove equations from unconstrained set
            for (auto& equation : current_equation_set.equations) {
                unconstrained_equation_set.equations.erase(equation);
            }

            // move this set to solve sets
            current_equation_set.set_solved();
            solve_sets.push_back(std::move(
                current_equation_set));  // TODO: note move happening here

            // repopulate the pq
            for (auto& eqn_set : temp_eqn_sets) {
                pq.push(std::move(eqn_set));
            }

            // reset visited equation sets
            visited_equation_sets.clear();
        } else {
            // add all equation sets in the frontier to the pq
            for (auto& eqn_set : current_equation_set.frontier()) {
                auto result = visited_equation_sets.insert(std::move(eqn_set));

                // only add an equation set if it hasn't been
                // visited yet
                if (result.second) {
                    pq.push(*result.first);
                }
            }
        }
    }

    // only add the unconstrained set if it isn't empty
    // TODO: break up into smaller independent groups (?)
    // TODO: unconstrained set could be last thing popped from pq (unless move
    // happened last)
    if (!unconstrained_equation_set.equations.empty()) {
        solve_sets.push_back(std::move(unconstrained_equation_set));
    }

    return solve_sets;
}

}  // namespace GCS
