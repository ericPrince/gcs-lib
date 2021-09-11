#ifndef GCS_CORE_PROBLEM
#define GCS_CORE_PROBLEM

#include <ceres/ceres.h>

#include <boost/asio.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gcs/core/constraints.h"
#include "gcs/core/geometry.h"
#include "gcs/core/solve_elements.h"

namespace gcs {

ceres::Solver::Summary single_solve(EquationSet& eqn_set);

struct Problem {
    std::unordered_set<Variable*> variables;
    std::unordered_set<Geometry*> geoms;
    std::unordered_set<Constraint*> constraints;

    std::vector<gcs::Equation> equations;

    // set of all EquationSets (has ownership of pointers)
    std::unordered_set<EquationSet*> equation_sets;
    // All items in the set must be solved before the key can be solved
    std::unordered_map<EquationSet*, std::unordered_set<EquationSet*>> prereqs;
    // The key must be solved before any of the values in the set can be solved
    std::unordered_map<EquationSet*, std::unordered_set<EquationSet*>>
        is_prereq_of;

    template <typename T>
    bool add(T* item);

    bool add_variable(Variable* var);
    bool add_geometry(Geometry* geom);
    bool add_constraint(Constraint* constraint);

    template <typename T>
    bool remove(T* item);

    bool remove_variable(Variable* var);
    bool remove_geometry(Geometry* geom);
    bool remove_constraint(Constraint* constraint);

    ~Problem();

    void reset_to_single_equation_set();

    void split();

    void solve(size_t pool_size = 0);
};

}  // namespace gcs

#endif  // GCS_CORE_PROBLEM
