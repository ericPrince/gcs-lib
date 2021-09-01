#ifndef GCS_LIB_INCLUDE_GCS_CORE_GEOMETRY
#define GCS_LIB_INCLUDE_GCS_CORE_GEOMETRY

#include <gcs/core/solve_elements.h>

#include <vector>

namespace gcs {

struct Geometry {
    virtual std::vector<Variable*> get_variables() = 0;
};

}  // namespace gcs

#endif  // GCS_LIB_INCLUDE_GCS_CORE_GEOMETRY
