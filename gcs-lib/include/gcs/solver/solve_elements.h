#ifndef GCS_SOLVE_ELEMENTS_
#define GCS_SOLVE_ELEMENTS_


#include <vector>
#include <map>
#include <set>
#include <unordered_set>
#include <queue>
#include <memory>


namespace GCS {


template <typename T> using uptr = std::unique_ptr<T>;


struct Variable;
struct Equation;
struct EquationSet;


struct Variable {
    std::unordered_set<Equation*> equations;

    void set_solved();
};


struct Equation {
    std::unordered_set<Variable*> variables;

    Equation(Equation&& equation);

    Equation(const decltype(variables)& vars);
    Equation(decltype(variables) && vars);

    void init();
};


struct EquationSet {
    std::unordered_set<Equation*> equations;

    // note: greater compare
    struct Compare {
        bool operator() (const EquationSet& a, const EquationSet& b);
    };

    bool operator== (const EquationSet& other) const;

    EquationSet& add_equation(Equation& equation);

    std::unordered_set<Variable*> get_variables() const;

    int degrees_of_freedom() const;
    bool is_constrained() const;

    void set_solved();

    std::unordered_set<EquationSet> frontier() const;
};


std::vector<EquationSet> split(EquationSet& equation_set);


}  // namespace GCS


namespace std {

//! Hash support for Eqn_set
template <>
struct hash<GCS::EquationSet>
{
    size_t operator()(const GCS::EquationSet& eqn_set) const;
};

}  // namespace std


#endif  // GCS_SOLVE_ELEMENTS_
