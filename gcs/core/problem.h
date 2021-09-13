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

//! Run ceres to solve a single equation set
//!
//! @param eqn_set the equation set to solve
//! @returns the ceres solver summary
ceres::Solver::Summary single_solve(EquationSet& eqn_set);

//! Definition of a geometric constraint solving problem
struct Problem {
    //! All variables that aren't used to define a geometry component
    std::unordered_set<Variable*> variables;
    //! All geometry components relevant to this problem
    std::unordered_set<Geometry*> geoms;
    //! All contraints relevant to this problem
    std::unordered_set<Constraint*> constraints;

    // For internal use: (TODO: make proteceted?)

    //! Set of all EquationSets (this class has ownership of pointers)
    std::unordered_set<EquationSet*> equation_sets;
    // All items in the set must be solved before the key can be solved
    std::unordered_map<EquationSet*, std::unordered_set<EquationSet*>> prereqs;
    // The key must be solved before any of the values in the set can be solved
    std::unordered_map<EquationSet*, std::unordered_set<EquationSet*>>
        is_prereq_of;

    //! Add a component to this problem
    //! @see add_variable
    //! @see add_geometry
    //! @see add_constraint
    template <typename T>
    bool add(T* item);

    //! Add a variable to this problem
    //!
    //! Currently does not trigger any extra processing
    bool add_variable(Variable* var);
    //! Add geometry to this problem
    //!
    //! Currently does not trigger any extra processing
    bool add_geometry(Geometry* geom);
    //! Add a constraint to this problem
    //!
    //! Causes an update to the structure of the problem and triggers equation
    //! set splitting and re-solving
    bool add_constraint(Constraint* constraint);

    //! Remove a component from this problem
    //! @see remove_variable
    //! @see remove_geometry
    //! @see remove_constraint
    template <typename T>
    bool remove(T* item);

    //! Remove a variable from this problem
    //!
    //! Currently does not trigger any extra processing
    bool remove_variable(Variable* var);
    //! Remove geometry from this problem
    //!
    //! Currently does not trigger any extra processing
    bool remove_geometry(Geometry* geom);
    //! Remove a constraint from this problem
    //!
    //! Causes an update to the structure of the problem and triggers equation
    //! set splitting and re-solving
    bool remove_constraint(Constraint* constraint);

    ~Problem();

    //! Reinitializes this problem to use a single equation set
    //!
    //! Uses the Equations defined by all contraints and places them into a
    //! single EquationSet
    void reset_to_single_equation_set();

    //! Splits the equation sets for this problem into smaller constrained sets
    //!
    //! This algorithm searches through the equations defined by the constraints
    //! and tries to separate them into independently solveable equation sets.
    //! The result forms a graph where some equation sets may depend on other
    //! sets being solved first so that the variables that they solve for can be
    //! held constant.
    //!
    //! Note that an equation set becomes independently sovleable when the
    //! number of variables it has to solve for equals the number of equations
    //! contained in the set. Once an equation set that's fully constrained is
    //! found, the variables it solves for can be held constant in all other
    //! equation sets. Any subsequent equation set that is found which depends
    //! on one of these variables now becomes dependent on the original equation
    //! set.
    void split();

    //! Solves this problem
    //!
    //! If there are dependencies between multiple equation sets, this algorithm
    //! will wait until the dependencies are solved and computed before starting
    //! to solve the dependent equation set.
    //!
    //! @param pool_size The size to use for the thread pool. If not given,
    //! defaults to the hardware concurrency value. Multiple equation sets will
    //! only be solved concurrently if allowed by the structure of the equation
    //! set dependency graph.
    void solve(size_t pool_size = 0);
};

}  // namespace gcs

#endif  // GCS_CORE_PROBLEM
