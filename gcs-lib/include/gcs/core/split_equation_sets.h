#ifndef GCS_LIB_INCLUDE_GCS_CORE_SPLIT_EQUATION_SETS
#define GCS_LIB_INCLUDE_GCS_CORE_SPLIT_EQUATION_SETS

#include <gcs/core/solve_elements.h>

#include <vector>

namespace gcs {

std::vector<EquationSet> split(EquationSet& equation_set);

}  // namespace gcs

#endif  // GCS_LIB_INCLUDE_GCS_CORE_SPLIT_EQUATION_SETS
