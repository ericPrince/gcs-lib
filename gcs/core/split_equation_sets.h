#ifndef GCS_CORE_SPLIT_EQUATION_SETS
#define GCS_CORE_SPLIT_EQUATION_SETS

#include <vector>

#include "gcs/core/solve_elements.h"

namespace gcs {

std::vector<EquationSet> split(EquationSet& equation_set);

}  // namespace gcs

#endif  // GCS_CORE_SPLIT_EQUATION_SETS
