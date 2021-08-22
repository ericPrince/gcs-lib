#ifndef PROBLEMS_PROBLEM_H_
#define PROBLEMS_PROBLEM_H_

#include <map>
#include <string>
//#include <vector>

#include <gcs/solver/solve_elements.h>

namespace problems {

//! @brief Definition of a constraint solving problem
struct Problem {
    std::map<std::string, GCS::Variable> vars;
    std::map<GCS::Variable*, std::string> var_names;

    //    std::vector<solve::Eqn>            eqn_vec;
    std::map<std::string, GCS::Equation> eqns;
    std::map<GCS::Equation*, std::string> eqn_names;
};

}  // namespace problems

#endif  // PROBLEMS_PROBLEM_H_
