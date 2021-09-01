#ifndef GCS_LIB_INCLUDE_GCS_CORE_SOLVE_ELEMENTS
#define GCS_LIB_INCLUDE_GCS_CORE_SOLVE_ELEMENTS

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

struct Variable {
    double value;
    std::unordered_set<Equation*> equations;

    Variable() = default;
    Variable(double value);

    void set_solved();
};

struct Equation {
    std::unordered_set<Variable*> variables;

    Equation(Equation&& equation);

    Equation(const decltype(variables)& vars);
    Equation(decltype(variables)&& vars);

    void init();
};

struct EquationSet {
    std::unordered_set<Equation*> equations;

    // note: greater compare
    struct Compare {
        bool operator()(const EquationSet& a, const EquationSet& b);
    };

    bool operator==(const EquationSet& other) const;

    EquationSet& add_equation(Equation& equation);

    std::unordered_set<Variable*> get_variables() const;

    int degrees_of_freedom() const;
    bool is_constrained() const;

    void set_solved();

    std::unordered_set<EquationSet> frontier() const;
};

}  // namespace gcs

namespace std {

//! Hash support for Eqn_set
template <>
struct hash<gcs::EquationSet> {
    size_t operator()(const gcs::EquationSet& eqn_set) const;
};

}  // namespace std

#endif  // GCS_LIB_INCLUDE_GCS_CORE_SOLVE_ELEMENTS
