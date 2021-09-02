#ifndef GCS_CORE_GEOMETRY
#define GCS_CORE_GEOMETRY

#include <vector>

#include "gcs/core/solve_elements.h"

namespace gcs {

struct Geometry {
    virtual std::vector<Variable*> get_variables() = 0;
};

}  // namespace gcs

#endif  // GCS_CORE_GEOMETRY
