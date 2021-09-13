#ifndef GCS_CORE_CONSTRAINTS
#define GCS_CORE_CONSTRAINTS

#include <ceres/ceres.h>

#include <boost/preprocessor.hpp>
#include <cmath>
#include <metal.hpp>

#include "gcs/core/solve_elements.h"

namespace gcs {

//! Definition of a constraint
//!
//! Constraint is a higher level class that may add multiple equations to be
//! solved.
struct Constraint {
    // virtual std::vector<Equation> get_equations() = 0;  // TODO

    //! Add equations to a ceres Problem
    virtual void add_to_problem(ceres::Problem& problem) = 0;

    //! Makes new equations based on the definition of this constraint
    //!
    //! @returns A vector of equations, which the caller is given ownership of
    //! @see gcs::Equation
    virtual std::vector<gcs::Equation*> get_equations() const = 0;
};

//! A macro that creates constraint functors
#ifndef CSTR_CREATE_FUNCTOR_
#define CSTR_CREATE_FUNCTOR_(func, n)                          \
    struct func##_functor {                                    \
        static const metal::int_ num_params = n;               \
                                                               \
        template <typename T>                                  \
        bool operator()(BOOST_PP_ENUM_PARAMS(n, const T* x)    \
                            BOOST_PP_COMMA_IF(n) T* r) const { \
            *r = func(BOOST_PP_ENUM_PARAMS(n, *x));            \
            return true;                                       \
        }                                                      \
    };
#endif

namespace detail {

template <typename Functor, typename... Args>
ceres::CostFunction* create_ad_impl_(Functor* functor, metal::list<Args...>) {
    return new ceres::AutoDiffCostFunction<Functor, Args::value...>(functor);
}

}  // namespace detail

//! Creates a ceres CostFunction from a properly implemented functor
//!
//! @param functor a properly implemented functor, such as one made from
//! CSTR_CREATE_FUNCTOR_
//! @returns a ceres AutoDiffCostFunction where each parameter block has size 1
//! and the residual block has size 1
template <typename Functor>
ceres::CostFunction* create_scalar_autodiff(Functor* functor) {
    using param_list =
        metal::repeat<metal::number<1>, metal::number<Functor::num_params + 1>>;

    return detail::create_ad_impl_(functor, param_list{});
}

}  // namespace gcs

#endif  // GCS_CORE_CONSTRAINTS
