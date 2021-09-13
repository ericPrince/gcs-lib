#ifndef GCS_CORE_SOLVE_ELEMENTS
#define GCS_CORE_SOLVE_ELEMENTS

#include <ceres/ceres.h>

#include <functional>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <unordered_set>

namespace gcs {

template <typename T>
using uptr = std::unique_ptr<T>;

struct Variable;
struct Equation;
struct EquationSet;

//! A variable, which takes on a value and tracks the equations it appears in
struct Variable {
    //! The value for this variable
    double value;
    //! Tracks the equations this variable appears in in a special way
    //!
    //! Before being split/solved: this set should contain all equations that
    //! reference this variable.
    //! After being split/solved: this set should contain all equations that
    //! require this variable to already be solved for and held constant until
    //! that equaiton can be used to solve for other variables.
    std::unordered_set<Equation*> equations;

    Variable() = default;
    Variable(double value);

    //! Set this variable as solved
    //!
    //! Removes this variable from the variables set of any equations in this
    //! variable's equations set. This has the effect of making it so at the end
    //! of splitting/solving, an equation only stores a reference to the
    //! variables that it solves for.
    void set_solved();
};

//! An equation, which tracks its variables and cost function
struct Equation {
    //! The variables tracked by this equation
    //!
    //! Before solving/splitting: contains all variables that are used in this
    //! equation.
    //! After solving/splitting: contains only the variables that this equation
    //! solves for (not ones that should be held constant and were assumed
    //! solved earlier).
    std::unordered_set<Variable*> variables;
    //! Function that adds a residual block to a ceres Problem
    //!
    //! This function should add exactly one residual block to the problem. That
    //! residual block should only use the variables refrenced by this equation
    //! when it was created.
    std::function<void(ceres::Problem&)> make_residual_ftor;

    Equation(Equation&& equation);

    Equation(const decltype(variables)& vars,
             decltype(make_residual_ftor) make_residual_ftor);
    Equation(decltype(variables)&& vars,
             decltype(make_residual_ftor) make_residual_ftor);

    void init();
};

//! A set of equations
struct EquationSet {
    //! The equations that make up this equation set.
    //!
    //! Generally, an equation set does not take ownership of the equations
    std::unordered_set<Equation*> equations;

    ~EquationSet();

    //! Greater than compare for two equation sets
    //!
    //! First compares based on degrees of freedom, then compares based on
    //! number of equations.
    //!
    //! @see degrees_of_freedom()
    struct Compare {
        //! @returns true iff a is greater than b
        bool operator()(const EquationSet& a, const EquationSet& b);
    };

    //! Two EquationSets are equal if their sets of equations are the same
    bool operator==(const EquationSet& other) const;

    //! Add a new equation to this set
    //!
    //! @param equation an equation to add to this set
    //! @returns this object
    EquationSet& add_equation(Equation& equation);

    //! Get all variablles referenced in this equation set
    //!
    //! If this equation set has been set as solved, then this function will get
    //! all variables that are solved by the equation set
    //!
    //! @returns pointers to the variables refrenced/solved by this equation set
    std::unordered_set<Variable*> get_variables() const;

    //! Get the degrees of freedom of this equation set
    //!
    //! Degrees of freedom is defined as number of equations minus number of
    //! variables that need to be solved
    //!
    //! @returns number of degrees of freedom
    int degrees_of_freedom() const;
    //! An equation set is constrained if it has zero degrees of freedom
    //!
    //! A constrained equation set generally has a finite number of valid
    //! solutions, while an under-constrained equation set may have infinite
    //! valid solutions. An over-constrained equation set will have 0 solutions
    //! unless some of the constraints are redundant but consistent.
    //!
    //! @returns true if the equation set has zero degrees of freedom
    bool is_constrained() const;

    //! Sets this equation set as solved
    //!
    //! After an equation set is marked as solved, each variable will point to
    //! equations that require it in order to be solved, and each equation will
    //! point to the variables it solves for.
    //!
    //! So this function does 2 things:
    //!   1. Removes each solved equation from the variables it solves for
    //!   2. Removes the solved variables from all unsolved equations (by
    //!   calling Variable::set_solved)
    void set_solved();

    //! Gives the set of EquationSets that are "adjacent" to this one
    //!
    //! The set of adjacent EquationSets consists of all equation sets that
    //! contain one more equation than this set, but only if the equation shares
    //! a variable with an equation in this set. The term "adjacent" is used
    //! because if a bipartite graph is made with Equations and Variables, the
    //! newly added equation comes from the set of equations that can be
    //! traversed to from somewhere on the span of the current equation set via
    //! 2 edges.
    //!
    //! This set is useful in searching for fully constrained equation sets.
    //!
    //! @returns the frontier set of adjacent equation sets to this one
    std::unordered_set<EquationSet> frontier() const;
};

}  // namespace gcs

namespace std {

//! Hash support for Eqn_set
//!
//! Note: this algorithm needs to be updated
template <>
struct hash<gcs::EquationSet> {
    size_t operator()(const gcs::EquationSet& eqn_set) const;
};

}  // namespace std

#endif  // GCS_CORE_SOLVE_ELEMENTS
