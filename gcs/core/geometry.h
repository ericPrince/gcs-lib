#ifndef GCS_CORE_GEOMETRY
#define GCS_CORE_GEOMETRY

#include <vector>

#include "gcs/core/solve_elements.h"

namespace gcs {

//! Base class for geometry components
//!
//! Each subclass should define its own variables and/or dependent geometry as
//! struct/class members. The class should have ownership over its variables and
//! geometry
struct Geometry {
    //! Get all variables belonging to this object
    //!
    //! Note that this includes variables of sub-components of the geometry
    //!
    //! @returns Pointers to each variable that defines this geometry
    virtual std::vector<Variable*> get_variables() = 0;
};

}  // namespace gcs

#endif  // GCS_CORE_GEOMETRY
