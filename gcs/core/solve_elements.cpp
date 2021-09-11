#include "gcs/core/solve_elements.h"

namespace gcs {

// Variable

Variable::Variable(double value) : value{value}, equations{} {}

void Variable::set_solved() {
    for (auto& eqn : this->equations) {
        // remove this variable from all equations
        eqn->variables.erase(this);
    }
}

// Equation

Equation::Equation(Equation&& equation)
    : variables{std::move(equation.variables)},
      make_residual_ftor{std::move(equation.make_residual_ftor)} {
    for (auto& var : this->variables) {
        var->equations.erase(&equation);
    }

    this->init();
}

Equation::Equation(const decltype(variables)& vars,
                   decltype(make_residual_ftor) make_residual_ftor)
    : variables{vars}, make_residual_ftor{make_residual_ftor} {
    this->init();
}

Equation::Equation(decltype(variables)&& vars,
                   decltype(make_residual_ftor) make_residual_ftor)
    : variables{vars}, make_residual_ftor{make_residual_ftor} {
    this->init();
}

void Equation::init() {
    // register this equation with its variables
    for (auto& var : this->variables) {
        var->equations.insert(this);
    }
}

// EquationSet

EquationSet::~EquationSet() {
    // for (auto& eq : equations) {
    //     delete eq;
    // }
}

bool EquationSet::Compare::operator()(const EquationSet& a,
                                      const EquationSet& b) {
    auto neq_a = a.equations.size();
    auto nvar_a = a.get_variables().size();
    int dof_a = nvar_a - neq_a;

    auto neq_b = b.equations.size();
    auto nvar_b = b.get_variables().size();
    int dof_b = nvar_b - neq_b;

    if (dof_a == dof_b) {
        return nvar_a > nvar_b;
    }

    return dof_a > dof_b;
}

bool EquationSet::operator==(const EquationSet& other) const {
    return this->equations == other.equations;
}

EquationSet& EquationSet::add_equation(Equation& equation) {
    this->equations.insert(&equation);
    return *this;
}

std::unordered_set<Variable*> EquationSet::get_variables() const {
    std::unordered_set<Variable*> variables;

    for (auto& eqn : this->equations) {
        for (auto& var : eqn->variables) {
            variables.insert(var);
        }
    }

    return variables;
}

int EquationSet::degrees_of_freedom() const {
    return this->get_variables().size() - this->equations.size();
}

bool EquationSet::is_constrained() const {
    return this->degrees_of_freedom() == 0;
}

void EquationSet::set_solved() {
    // at the end, a variable will point to equations that require it in order
    // to be solved and an equation will point to the variables it solves for

    // remove the solved equations from their variables
    for (auto& eqn : this->equations) {
        for (auto& var : eqn->variables) {
            var->equations.erase(eqn);
        }
    }

    // remove the solved variables from all (non-solved) equations
    for (auto& var : this->get_variables()) {
        var->set_solved();
    }
}

std::unordered_set<EquationSet> EquationSet::frontier() const {
    std::unordered_set<Equation*> frontier_equations{};

    // get all equations touching this set's variables
    for (auto& var : this->get_variables()) {
        for (auto* eqn : var->equations) {
            frontier_equations.insert(eqn);
        }
    }

    // remove equations that are already in this set's equations
    for (auto& eqn : this->equations) {
        frontier_equations.erase(eqn);
    }

    std::unordered_set<EquationSet> frontier_set;

    // make equation sets
    for (auto& eqn : frontier_equations) {
        frontier_set.insert(EquationSet{*this}.add_equation(*eqn));
    }

    return frontier_set;
}

}  // namespace gcs

size_t std::hash<gcs::EquationSet>::operator()(
    const gcs::EquationSet& eqn_set) const {
    // references:
    // - hash function for vector: https://stackoverflow.com/a/27216842
    // - iteration over unordered set: https://stackoverflow.com/q/36242103

    std::size_t seed = eqn_set.equations.size();
    for (auto& eqn : eqn_set.equations) {
        seed ^= reinterpret_cast<size_t>(eqn);
        // // TODO: old hash function was dependent on random order of iteration
        // seed ^= reinterpret_cast<size_t>(eqn) + 0x9e3779b9 + (seed << 6) +
        //         (seed >> 2);
    }
    return seed;
};
